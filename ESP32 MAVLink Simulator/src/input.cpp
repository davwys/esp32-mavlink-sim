#include <Arduino.h>
#include <input.h>

// Parameter setup
// These parameters may be entered manually, parsed from another protocol or simulated locally.
// If your input protocol does not provide values for all of these, parameters may be left at their default values.
// All parameters set here are passed onward through serial output in MAVLink format.

//Basic UAV Parameters
uint8_t system_id = 1;        // MAVLink system ID. Leave at 0 unless you need a specific ID.
uint8_t component_id = 0;     // Should be left at 0. Set to 190 to simulate mission planner sending a command
uint8_t system_type = 1;      // UAV type. 0 = generic, 1 = fixed wing, 2 = quadcopter, 3 = helicopter
uint8_t autopilot_type = 0;   // Autopilot type. Usually set to 0 for generic autopilot with all capabilities
uint8_t system_mode = 64;     // Flight mode. 4 = auto mode, 8 = guided mode, 16 = stabilize mode, 64 = manual mode
uint32_t custom_mode = 0;     // Usually set to 0
uint8_t system_state = 4;     // 0 = unknown, 3 = standby, 4 = active
uint32_t upTime = 0;          // System uptime, usually set to 0 for cases where it doesn't matter

// Flight parameters
float roll = 0;         // Roll angle in degrees
float pitch = 0;        // Pitch angle in degrees
float yaw = 0;          // Yaw angle in degrees
int16_t heading = 0;      // Geographical heading angle in degrees
float lat = 47.379945;    // GPS latitude in degrees (example: 47.123456)
float lon = 8.539970;     // GPS longitude in degrees
float alt = 0.0;        // Relative flight altitude in m
float groundspeed = 0.0; // Groundspeed in m/s
float airspeed = 0.0;    // Airspeed in m/s
float climbrate = 0.0;    // Climb rate in m/s, currently not working
float throttle = 0.0;     // Throttle percentage

// GPS parameters
int16_t gps_sats = 0;    // Number of visible GPS satellites
int32_t gps_alt = 0.0;  // GPS altitude (Altitude above MSL)
float gps_hdop = 100.0;     // GPS HDOP
uint8_t fixType = 0;      // GPS fix type. 0-1: no fix, 2: 2D fix, 3: 3D fix

// Battery parameters
float battery_remaining = 0.0;  // Remaining battery percentage
float voltage_battery = 0.0;    // Battery voltage in V
float current_battery = 0.0;    // Battery current in A
