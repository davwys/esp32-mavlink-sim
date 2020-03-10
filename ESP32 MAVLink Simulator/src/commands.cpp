#include <Arduino.h>
#include <ASLUAV/mavlink.h>
#include <BluetoothSerial.h>

#include <commands.h>
#include <settings.h>
/************************************************************
* @brief Sends a MAVLink heartbeat message, needed for the system to be recognized
* @param Basic UAV parameters, as defined above
* @return void
*************************************************************/

void command_heartbeat(uint8_t system_id, uint8_t component_id, uint8_t system_type, uint8_t autopilot_type, uint8_t system_mode, uint32_t custom_mode, uint8_t system_state) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_heartbeat_pack(system_id,component_id, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message
  Serial.write(buf, len);

  // Send via Bluetooth (if enabled)
  if(bluetooth_enabled){
    BTSerial.write(buf, len);
  }
}

/************************************************************
* @brief Sends a MAVLink parameter message, needed for the system to be recognized automatically in Mission Planner/QGroundcontrol
* @param Basic UAV parameters, as defined above
* @return void
*************************************************************/

void command_parameters(int8_t system_id, uint8_t component_id) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_param_value_pack(system_id, component_id, &msg, "RC_SPEED", 50, 1, 1, 0);
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message
  Serial.write(buf, len);

  // Send via Bluetooth (if enabled)
  if(bluetooth_enabled){
    BTSerial.write(buf, len);
  }
}


/************************************************************
* @brief Send some system data parameters (battery, etc)
* @param
* @return void
*************************************************************/

void command_status(uint8_t system_id, uint8_t component_id, float battery_remaining, float voltage_battery, float current_battery) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
    mavlink_msg_sys_status_pack(system_id, component_id, &msg, 32767, 32767, 32767, 500, voltage_battery * 1000.0, current_battery * 100.0, battery_remaining, 0, 0, 0, 0, 0, 0);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message (.write sends as bytes)
  Serial.write(buf, len);

  // Send via Bluetooth (if enabled)
  if(bluetooth_enabled){
    BTSerial.write(buf, len);
  }
}

/************************************************************
* @brief Sends Integer representation of location, only for ijnternal use (do not call directly)
* @param lat: latitude, lon: longitude, alt: altitude, gps_alt: altitude above MSL, heading: heading
* @return void
*************************************************************/

void command_globalgps(int8_t system_id, int8_t component_id, int32_t upTime, float lat, float lon, float alt, float gps_alt, uint16_t heading) {

  int16_t velx = 0; //x speed
  int16_t vely = 0; //y speed
  int16_t velz = 0; //z speed


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_global_position_int_pack(system_id, component_id, &msg, upTime, lat * 10000000.0, lon * 10000000.0, gps_alt * 1000.0, alt * 1000.0, velx, vely, velz, heading);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);

  // Send via Bluetooth (if enabled)
  if(bluetooth_enabled){
    BTSerial.write(buf, len);
  }
}


/************************************************************
* @brief Sends current geographical location (GPS position), altitude and heading
* @param lat: latitude in degrees, lon: longitude in degrees, alt: altitude, heading: heading
* @return void
*************************************************************/

void command_gps(int8_t system_id, int8_t component_id, int32_t upTime, int8_t fixType, float lat, float lon, float alt, float gps_alt, int16_t heading, float groundspeed, float gps_hdop, int16_t gps_sats) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_gps_raw_int_pack(system_id, component_id, &msg, upTime, fixType, lat * 10000000.0, lon * 10000000.0, alt * 1000.0, gps_hdop * 100.0, 65535, groundspeed, 65535, gps_sats, 0, 0, 0, 0, 0);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  //Send globalgps command
  command_globalgps(system_id, component_id, upTime, lat, lon, alt, gps_alt, heading);

  // Send the message (.write sends as bytes)
  Serial.write(buf, len);

  // Send via Bluetooth (if enabled)
  if(bluetooth_enabled){
    BTSerial.write(buf, len);
  }
}

/************************************************************
* @brief Send some core parameters such as speed to the MAVLink ground station HUD
* @param
* @return void
*************************************************************/

void command_hud(int8_t system_id, int8_t component_id, float airspeed, float groundspeed, int16_t heading, float throttle, float alt, float climbrate) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_vfr_hud_pack(system_id, component_id, &msg, airspeed, groundspeed, heading, throttle, alt * 1000.0, climbrate);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message (.write sends as bytes)
  Serial.write(buf, len);

  // Send via Bluetooth (if enabled)
  if(bluetooth_enabled){
    BTSerial.write(buf, len);
  }
}

/************************************************************
* @brief Send attitude and heading data to the primary flight display (artificial horizon)
* @param roll: roll in degrees, pitch: pitch in degrees, yaw: yaw in degrees, heading: heading in degrees
* @return void
*************************************************************/

void command_attitude(int8_t system_id, int8_t component_id, int32_t upTime, float roll, float pitch, float yaw) {

  //Radian -> degree conversion rate
  float radian = 57.2958;

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_attitude_pack(system_id, component_id, &msg, upTime, roll/radian, pitch/radian, yaw/radian, 0, 0, 0);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);

  // Send via Bluetooth (if enabled)
  if(bluetooth_enabled){
    BTSerial.write(buf, len);
  }
}
