#include <ASLUAV/mavlink.h>

#ifndef COMMANDS_H
#define COMMANDS_H


// Send MAVLink heartbeat
void command_heartbeat(uint8_t system_id, uint8_t component_id, uint8_t system_type, uint8_t autopilot_type, uint8_t system_mode, uint32_t custom_mode, uint8_t system_state);

// Send parameters
void command_parameters(int8_t system_id, uint8_t component_id);

// Send battery status
void command_status(uint8_t system_id, uint8_t component_id, float battery_remaining, float voltage_battery, float current_battery);

// Send GPS and altitude data
void command_gps(int8_t system_id, int8_t component_id, int32_t upTime, int8_t fixType, float lat, float lon, float alt, float gps_alt, int16_t heading, float groundspeed, float gps_hdop, int16_t gps_sats);

// Send HUD data (speed, heading, climbrate etc.)
void command_hud(int8_t system_id, int8_t component_id, float airspeed, float groundspeed, int16_t heading, float throttle, float alt, float climbrate);

// Send attitude data to artificial horizon
void command_attitude(int8_t system_id, int8_t component_id, int32_t upTime, float roll, float pitch, float yaw);

#endif
