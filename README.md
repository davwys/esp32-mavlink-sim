# ESP32 MAVLink Simulator
Simulate MAVLink commands for use with groundstation apps on an ESP32

(ESP32 port of https://github.com/davwys/arduino-mavlink-sim)

## Introduction

This is a general-purpose ESP32 program for simulating MAVLink telemetry data that can be used by common groundstation applications such as Mission Planner or APM Planner 2.0.
The primary idea behind this is to serve as a base for using an ESP32 to convert other telemetry protocols (OpenTelemetry, BST, S.Port) to the very common MAVLink V2 format. This then allows the use of MAVLink-based groundstation applications with non-supported telemetry protocols.

The program can also be used to simulate any type of MAVLink vehicle from an ESP32, as the parsed telemetry data can be sourced from any source connected to the ESP32 (for example, this would allow the use of Mission Planner to view telemetry from a simple ESP32-based rover).

Note that the program does **NOT** convert any telemetry protocol or provide a data source - it only acts as an interface for converting raw input variables into MAVLink format.

## Installation & Testing

Using Platformio, flash the program to your ESP32. You might need to change the settings in the platformio.ini file to suit the board you're using and choose the correct COM port. I used a Lolin D32 Pro board, but others should work as well.

You will need the MAVLink library, provided in .zip format here or downloadable directly in Platformio.

The ESP32 will then provide telemetry data through its USB port, as well as via Bluetooth. The default baud rate for both is set to 57600 and can be changed from the main.cpp file. To disable Bluetooth, change bluetooth_enabled to false in settings.cpp.

Since the ESP32 offers more performance than an Arduino, the telemetry rate (hz) can be increased to up to 40hz. However, if you're doing additional processing (e.g. conversion of incoming data), I'd recommend leaving it at the default 10hz.

From your groundstation application, simply choose the appropriate COM port, set baud rate to 57600 and hit "connect".
If you are using Mission Planner, you might have to skip parameter fetching, as the program currently does not output any parameter data.

## Currently supported applications

The following groundstation applications have been tested and are known to be working with this sketch:

- [x] Mission Planner (Windows)
- [x] APM Planner 2.0 (macOS/Windows)
- [ ] QGroundcontrol (macOS/Windows) -> coming soon

## Known issues

- Climb rate and acceleration are currently unsupported


If you find any further problems, feel free to open an issue!

## Future plans

- Create separate versions for converting OpenTelemetry, BST and S.Port telemetry data into MAVLink format
- Add support for QGroundcontrol


#### (c) 2020 David Wyss
