# AESS-DRONE
AESS-DRONE is a small quadcopter firmware project for an ESP32-S3 development board using an MPU6050 IMU and ESP‑NOW receiver. The repo contains the flight firmware, peripherals (IMU, motors, receiver), a native simulation target for local testing, and example transmitter code.

## Features
- Attitude estimation from MPU6050
- PID-based attitude control (roll, pitch, yaw)
- Motor mixing for X-config quadcopters
- ESP‑NOW receiver integration for remote control
- Native simulation/build target to test logic without hardware
- PlatformIO build system (Arduino framework)

## Repository layout
- firmware/
  - src/
    - main.cpp — main flight loop, control and mixing
    - imu.* — IMU init, read, calibration
    - pid.* — PID controller implementation
    - motors.* — PWM/ESC interface and motor helpers
    - receiver.* — ESP‑NOW receiver and payload parsing
  - platformio.ini — build environments (rymcu-esp32-s3-devkitc-1, native)
- hardware/ — pcb design , BOM 
- docs/ — architecture, calibration, tuning notes (optional)
- tests/ — unit tests / native simulation tests (PlatformIO test runner)
- LICENSE — project license

## Requirements
- VS Code with PlatformIO extension (tested on Windows)
- ESP32-S3 dev board (rymcu-esp32-s3-devkitc-1 board config)
- MPU6050 IMU
- ESCs and brushless motors (appropriate power supply / battery)
- Optional: second ESP32 or transmitter supporting ESP‑NOW

## Quick start (VS Code + PlatformIO)
1. Open project in VS Code.
2. Install PlatformIO extension if not installed.
3. Build for ESP32-S3:
   - Open terminal in project root and run:
     pio run -e rymcu-esp32-s3-devkitc-1
4. Upload to board (connect board via USB, put in boot mode if needed):
   - pio run -e rymcu-esp32-s3-devkitc-1 -t upload
5. Serial monitor:
   - pio device monitor -e rymcu-esp32-s3-devkitc-1
6. Native simulation:
   - pio run -e native
   - Run the produced executable from PowerShell or CMD:
     .\.pio\build\native\<executable>.exe

## Configuration
- Build environments and dependencies are defined in firmware/platformio.ini.
- PID initial gains are set in firmware/src/main.cpp — tune these progressively.
- Receiver packet format and mapping live in firmware/src/receiver.*; adjust mapping if your transmitter differs.

## Hardware & wiring notes
- Verify IMU SDA/SCL pins match board wiring and firmware defines.
- Motor pins, PWM frequency, and PWM resolution are in firmware/src/motors.* — confirm with your ESCs.
- Use a common ground between flight controller and receiver/transmitter modules.
- Always remove props when testing initialization, ESC calibration, or motor direction.

## IMU calibration & PID tuning
- Run IMU calibration routine (see imu.*) on a level surface to capture gyro offsets.
- Tune PID gains one axis at a time:
  1. Start with low P, I=0, D=0; increase P until oscillation begins, then back off.
  2. Add D to damp oscillation.
  3. Add small I to correct steady-state error.
- Use the native simulation to iterate faster before testing on hardware.

## Safety
- Test with propellers removed until motor directions, ESC calibration, and basic control loops are verified.
- Start with low throttle and fly in a controlled environment.
- Ensure a reliable kill switch / quick power cutoff method is available.

## Development & testing
- Use PlatformIO test runner for unit/native tests:
  pio test -e native
- Add unit tests for control logic and PID when changing algorithms.
- Keep commits focused and include tests for behavioral changes.

## Contributing
- Open issues for bugs or enhancement suggestions.
- Submit PRs with clear descriptions and tests where applicable.
- Keep code style consistent with existing sources.

## License
See LICENSE in repository root for licensing details.
