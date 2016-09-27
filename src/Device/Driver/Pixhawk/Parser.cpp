#include "Device.hpp"
#include "NMEA/Info.hpp"
#include "Device/Port/Port.hpp"
#include "Util/ConvertString.hpp"
#include "Atmosphere/Temperature.hpp"

/*
 *
 */
static void ReceiveHeartbeat(mavlink_message_t msg, NMEAInfo &info)
{
  mavlink_heartbeat_t packet;

  mavlink_msg_heartbeat_decode(&msg, &packet);

  info.alive.Update(info.clock);
}

/*
 *
 */
static void ReceiveFastData(mavlink_message_t msg, NMEAInfo &info)
{
  double xacc, yacc, zacc;
  mavlink_pixhawk_hg_fast_t packet;

  mavlink_msg_pixhawk_hg_fast_decode(&msg, &packet);

  // GPS time
  info.ProvideTime((double)packet.gps_time * 1e-3);
  // G-force
  xacc = (double)packet.xacc;
  yacc = (double)packet.yacc;
  zacc = (double)packet.zacc;
  double load = sqrt(xacc*xacc + yacc*yacc + zacc*zacc)  / 9.80665; // in "G"s
  info.acceleration.ProvideGLoad( load, true);
  // pitch
  Angle pitch = Angle::Radians((double)packet.pitch);
  info.attitude.pitch_angle = pitch;
  info.attitude.pitch_angle_available.Update(info.clock);
  // roll
  Angle roll  = Angle::Radians((double)packet.roll);
  info.attitude.bank_angle = roll;
  info.attitude.bank_angle_available.Update(info.clock);
  // heading
  Angle hdg = Angle::Degrees((double)packet.hdg * 0.01);
  info.attitude.heading = hdg;
  info.attitude.heading_available.Update(info.clock);
  info.heading = hdg;
  info.heading_available.Update(info.clock);

  info.alive.Update(info.clock);
}

/*
 *
 */
static void ReceiveMedData(mavlink_message_t msg, NMEAInfo &info)
{
  mavlink_pixhawk_hg_med_t packet;

  mavlink_msg_pixhawk_hg_med_decode(&msg, &packet);
  // position
  Angle lat = Angle::Degrees((double)packet.lat * 1.e-7);
  Angle lon = Angle::Degrees((double)packet.lon * 1.e-7);
  GeoPoint loc(lon, lat);
  info.location = loc;
  info.location_available.Update(info.clock);
  // altitude
  double alt = (double)packet.alt * 1.e-3;
  info.ProvideBaroAltitudeTrue(alt);

  //ground speed
  SpeedVector v = SpeedVector((double)packet.vy*0.01, (double)packet.vx*0.01);
  info.ground_speed = v.norm;  // in m/s
  info.ground_speed_available.Update(info.clock);
  // ground track
  info.track = v.bearing;
  info.track_available.Update(info.clock);

  // vario
  info.ProvideNoncompVario((double)packet.vz * 0.01);
  // energy compensated vario
  info.ProvideTotalEnergyVario((double)packet.energy_vario * 0.01);
  // Airspeed
  info.ProvideBothAirspeeds((double)packet.indicated_airspeed,
                            (double)packet.true_airspeed);

  info.alive.Update(info.clock);
}

/*
 *
 */
static void ReceiveSlowData(mavlink_message_t msg, NMEAInfo &info)
{
  mavlink_pixhawk_hg_slow_t packet;

  mavlink_msg_pixhawk_hg_slow_decode(&msg, &packet);

  /* Hard coded UTC offset. Obviously this is not ideal but will have to do for now.
   * NZ offset is 12 hours, or 13 hours during daylight savings.
   * 12 hours is close enough unless we plan to fly around midnight.
   */
  unsigned utc_offset_seconds = 12 * 3600;

  // Date
  BrokenDateTime dt = BrokenDateTime::FromUnixTimeUTC(packet.gps_time_sec + utc_offset_seconds);
  info.ProvideDate(BrokenDate(dt.year,dt.month,dt.day));
  // GPS altitude
  info.gps_altitude = (double)packet.gps_alt * 1.e-3;
  info.gps_altitude_available.Update(info.clock);
  // horizontal dilution of precision (m)
  info.gps.hdop = (double)packet.eph * 0.01;
  // num satellites
  info.gps.satellites_used = (int)packet.satellites_visible;
  info.gps.satellites_used_available.Update(info.clock);
  // GPS signal OK?
  info.gps.fix_quality = packet.gps_fix_type >= 2 ? FixQuality::GPS : FixQuality::NO_FIX;
  info.gps.fix_quality_available.Update(info.clock);
  // temperature (deg K)
  info.temperature = Temperature::FromCelsius((double)packet.temperature);
  info.temperature_available = true;
  // humidity (%)
  info.humidity = (double)packet.humidity;
  //  info.humidity_available = true;
  // wind (m/s)
  info.ProvideExternalWind( SpeedVector( (double)packet.wind_e, (double)packet.wind_n));

  info.gps.real = true;
  info.alive.Update(info.clock);
}

/*
 *
 */
static void ReceiveText(mavlink_message_t msg, NMEAInfo &info)
{
  mavlink_statustext_t packet;
  int i;
  char char_buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];

  mavlink_msg_statustext_decode(&msg, &packet);

  memset((void *)char_buf, '\0', MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1); //necessary?
  for (i=0; i<MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN; i++){
    char_buf[i] = packet.text[i];
    if( char_buf[i] == '\0') break;
  }
  size_t len = (size_t)i+1;

  // convert to TCHAR
  UTF8ToWideConverter wide_buf(char_buf);
  if (wide_buf.IsValid()){
    info.ProvideDebugMsg( wide_buf, len);
  }
  info.alive.Update(info.clock);
}
  
/*
 *
 */
static void HandleMessage(mavlink_message_t msg, NMEAInfo &info)
{
  switch(msg.msgid)
  {
  case MAVLINK_MSG_ID_HEARTBEAT:
    {
      ReceiveHeartbeat(msg, info);
    }
    break;
  case MAVLINK_MSG_ID_PIXHAWK_HG_FAST:
    {
      ReceiveFastData(msg, info);
    }
    break;
  case MAVLINK_MSG_ID_PIXHAWK_HG_MED:
    {
      ReceiveMedData(msg, info);
    }
    break;
  case MAVLINK_MSG_ID_PIXHAWK_HG_SLOW:
    {
      ReceiveSlowData(msg, info);
    }
    break;
  case MAVLINK_MSG_ID_STATUSTEXT:
    {
      ReceiveText(msg, info);
    }
    break;
  default:
    //Do nothing
    break;
  }
}

/*
 *
 */
bool PixhawkDevice::DataReceived(const void *_data, size_t length,
                            struct NMEAInfo &info)
{
  uint8_t c;
  bool result=false;
  assert(_data != NULL);
  assert(length > 0);

  const uint8_t *data = (const uint8_t *)_data, *end = data + length;

  for(; data < end; data++)
  {
    // Try to get a new message
    c = *data;
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      HandleMessage(msg, info);
      result=true;
    }
  }

  return result;
//  return true; // I think this is correct, otherwise the same data may be passed back in. TODO check!
}
