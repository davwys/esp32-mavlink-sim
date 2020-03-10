#include <Arduino.h>

#ifndef INPUT_H
#define INPUT_H

//Basic UAV Parameters
extern uint8_t system_id;
extern uint8_t component_id;
extern uint8_t system_type;
extern uint8_t autopilot_type;
extern uint8_t system_mode;
extern uint32_t custom_mode;
extern uint8_t system_state;
extern uint32_t upTime;

// Flight parameters
extern float roll;
extern float pitch;
extern float yaw;
extern int16_t heading;
extern float lat;
extern float lon;
extern float alt;
extern float groundspeed;
extern float airspeed;
extern float climbrate; //currently not working
extern float throttle;

// GPS parameters
extern int16_t gps_sats;
extern int32_t gps_alt;
extern float gps_hdop;
extern uint8_t fixType;

// Battery parameters
extern float battery_remaining;
extern float voltage_battery;
extern float current_battery;

#endif
