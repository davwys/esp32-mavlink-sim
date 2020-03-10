// ESP32 MAVLink Simulation
//
// (c) 2020 David Wyss
//
// Base Arduino to MAVLink code from https://github.com/alaney/arduino-mavlink

#include <ASLUAV/mavlink.h>
#include <Arduino.h>
#include <BluetoothSerial.h>

#include <commands.h>
#include <input.h>

// Serial Baud rate
//Default: 57600 baud
#define BAUD_RATE 57600

// Telemetry message rate in Hz
//Default: 10hz
//Maximum: 40hz (depending on what additional processing is being done)
#define TELEMETRY_HZ 10


BluetoothSerial BTSerial;
uint16_t wait = 1000/TELEMETRY_HZ;


void setup() {
   // Enable serial output.
   Serial.begin(BAUD_RATE);
   // Enable bluetooth serial output
   BTSerial.begin("MAVLink");
}



//Main loop: Send generated MAVLink data via USB (and bluetooth, if enabled)
void loop() {

  // Send MAVLink heartbeat
  command_heartbeat(system_id, component_id, system_type, autopilot_type, system_mode, custom_mode, system_state);

  // Send parameters (needed for detection by some GCS software)
  command_parameters(system_id, component_id);

  //Send battery status
  command_status(system_id, component_id, battery_remaining, voltage_battery, current_battery);

  // Send GPS and altitude data
  command_gps(system_id, component_id, upTime, fixType, lat, lon, alt, gps_alt, heading, groundspeed, gps_hdop, gps_sats);

  // Send HUD data (speed, heading, climbrate etc.)
  command_hud(system_id, component_id, airspeed, groundspeed, heading, throttle, alt, climbrate);

  // Send attitude data to artificial horizon
  command_attitude(system_id, component_id, upTime, roll, pitch, yaw);

  //Delay: send messages at set rate
  delay(wait);
}
