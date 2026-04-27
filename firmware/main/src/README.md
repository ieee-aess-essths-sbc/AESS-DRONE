# AESS Drone Firmware: `/src` Module Documentation

## System Overview

The AESS drone firmware is a **250Hz (4ms) control loop** for a brushed quad-rotor that runs on ESP32-S3. The architecture follows a clean separation of concerns:

```
Raw IMU Data (I2C) 
    ↓
    Sensor Fusion (Complementary Filter)
    ↓
    Fused Attitude Angles + Rates
    ↓
    Safety Gate Check (FlightSystem)
    ↓
    PID Control (3 axes: roll, pitch, yaw)
    ↓
    Motor Mixing (Quad-X configuration)
    ↓
    PWM Output to ESCs (8520 brushed motors)
```

---

## File Directory Structure & Explanations

### **Core Root Files**

#### `main.cpp`
- **Purpose**: Main entry point and primary control loop
- **Key Functions**:
  - `setup()`: Initializes receiver, IMU, motors, stabilizer, flight system, power manager
  - `loop()`: 250Hz control loop that reads IMU, updates stabilizer, writes motor commands
- **Loop Timing**: Fixed at 4ms (LOOP_TIME_US = 4000µs) = 250Hz
- **Dependencies**: receiver.h, imu.h, motors.h, stabilizer.h, flight_system.h, pm_manager.h
- **Important**: Loop time is monitored; if exceeded, dt is clamped to 0.002s default to prevent integration instability

#### `imu.cpp / imu.h`
- **Purpose**: MPU-6050 accelerometer & gyroscope raw data acquisition
- **Hardware**: 
  - Sensor: InvenSense MPU-6050 (6-axis IMU)
  - I2C Address: 0x68
  - Configuration: ±8g accelerometer, ±500°/s gyroscope, 500Hz sample rate
  
- **Key Functions**:
  - `initIMU()`: Configures MPU-6050 registers, initializes I2C at 400kHz
  - `calibrateIMU()`: Collects 500 samples (~1 second) to measure static bias offsets
  - `readIMU()`: Burst-reads 14 bytes (6 accel + 2 temp + 6 gyro) from MPU
  
- **Calibration Process**:
  - **Gyro offsets**: Drone must be still; gyro output = bias (zeroed in readIMU)
  - **Accel offsets**: Drone flat on ground; Z-axis should read +1g (4096 LSB at ±8g), X/Y should be 0g
  - **Formulas**:
    - Accel = (raw - offset) / 4096.0 [G units]
    - Gyro = (raw - offset) / 65.5 [deg/s]

- **Output**: Returns `IMUData` struct with calibrated ax, ay, az (G), gx, gy, gz (°/s)

- **Safety Features**:
  - `dt` guard: if elapsed time > 50ms (< 20Hz), frame is marked invalid
  - Health flag: set false if I2C read fails or accel magnitude deviates

#### `motors.cpp / motors.h`
- **Purpose**: Low-level PWM control for 8520 brushed motors via MOSFETs
- **Hardware**:
  - Motors: 8520 brushed motors × 4 (CCW pair: M1 front-left, M3 rear-right; CW pair: M2 front-right, M4 rear-left)
  - Driver: ESP32-S3 LEDC PWM on pins 5–8 (configurable)
  - PWM Frequency: 20kHz (ultrasonic, no audible whine)
  - PWM Resolution: 8-bit (0–255 duty)
  
- **Motor Safety Limits**:
  - `MOTOR_MIN = 30` (12% duty): minimum spinning speed under load
  - `MOTOR_MAX = 200` (78% duty): safety cap to prevent over-current
  - `MOTOR_ARM_TARGET = 40` (16% duty): idle spin during arming ramp
  
- **Key Functions**:
  - `initMotors()`: Attaches all pins to LEDC at 20kHz/8-bit, all at 0
  - `armMotors()`: Soft ramp from 0→40 over ~400ms (safety against inrush current)
  - `stopAllMotors()`: Immediately zero all motors, clear armed flag
  - `setMotorSpeed(motorNum, speed)`: Set individual motor (1–4), applies MOTOR_MIN clamp
  - `setMotorSpeeds(m1, m2, m3, m4)`: Set all four at once (called every loop by stabilizer)
  
- **Important**: Motors only respond if `motorsArmed == true` (safety gate)

#### `receiver.cpp / receiver.h`
- **Purpose**: Handles RC receiver input (PPM or PWM protocol)
- **Role in System**: Provides remote control setpoints to main loop
- **Note**: Implementation details vary; refer to receiver module documentation

---

### **Flight Control Subsystem** (`flight/`)

#### `flight_types.h`
- **Purpose**: Shared data structures (no logic)
- **Key Structs**:
  - `AttitudeDeg`: roll, pitch, yaw in degrees (output from sensor fusion)
  - `AttitudeRateDps`: angular rates in deg/s (from raw gyro)
  - `Setpoint`: target attitude + rates (from control loop input)
  - `ControlOutput`: signed PID corrections (roll, pitch, yaw deltas)
  - `MotorMix`: final PWM duty values for each motor (0–255)

#### `stabilizer.cpp / stabilizer.h`
- **Purpose**: Main control pipeline hub; orchestrates all sub-systems
- **Control Loop Pipeline**:
  1. Read live `dt` from `micros()` (clamped 0.5ms–50ms)
  2. Feed raw IMU into `SensFusion6` → fused attitude
  3. Check `FlightSystem::shouldCutPower()` safety gate
  4. Check IMU health (`imu.isHealthy`)
  5. Run `ControllerPid::update()` → get signed corrections
  6. Call `PowerDistribution::mixX(HOVER_THRUST, corrections)` → motor mix
  7. Return `MotorMix` to main() for motor writes

- **Critical Parameter**: 
  - `HOVER_THRUST = 120` (duty units): base upward force for hover
  - Tune this until drone just lifts off, then add ~10% headroom
  - For 8520 at 3.7V on ~35g frame: 110–130 typical range

- **Key Methods**:
  - `init(FlightSystem& fs)`: Initialize fusion, PID, mixer; seed attitude at level
  - `update(Setpoint, IMUData)`: Run one control cycle (called 250Hz)
  - `emergencyStop(reason)`: Zero mix + trigger FlightSystem halt

---

#### `sensfusion6.cpp / sensfusion6.h` — Complementary Filter
- **Purpose**: Fuse 6-axis gyro + accel data into stable attitude angles
- **Theory**: 
  - **Gyro**: accurate short-term, drifts over time
  - **Accel**: noisy, always knows gravity direction
  - **Blend**: trust gyro 98%, correct with accel 2% per frame

- **Filter Equations**:
  ```
  accelRoll = atan2(ay, sqrt(ax² + az²)) * 180/π
  accelPitch = atan2(-ax, sqrt(ay² + az²)) * 180/π
  
  roll_fused = 0.98 * (roll_prev + gx * dt) + 0.02 * accelRoll
  pitch_fused = 0.98 * (pitch_prev + gy * dt) + 0.02 * accelPitch
  yaw_fused += gz * dt  (no accel correction; drifts without magnetometer)
  ```

- **Accel Magnitude Guard**: Skip accel correction if total accel deviates from 1G by >30%
  - Min: 0.7G (freefall/spike detection)
  - Max: 1.3G (hard impact/vibration detection)
  - Prevents motor vibration from corrupting angle estimate

- **Limitation**: Yaw drifts over time without magnetometer (acceptable for 15s missions)

---

#### `controller_pid.cpp / controller_pid.h` — PID Gains
- **Purpose**: Three independent PID loops (roll angle, pitch angle, yaw rate)
- **Control Strategy**:
  - **Roll/Pitch**: Angle control (PID on error between setpoint and fused attitude)
  - **Yaw**: Rate control (PID on error between desired yaw rate and gyro rate)

- **PID Gain Values** (tuned for 8520 brushed motors at 3.7V):
  
  | Axis | Kp | Ki | Kd | Integral Limit | Output Limit |
  |------|-----|-------|--------|----------------|--------------|
  | Roll | 4.5 | 1.5 | 0.08 | 15 duty | 80 duty |
  | Pitch | 4.5 | 1.5 | 0.08 | 15 duty | 80 duty |
  | Yaw | 5.0 | 0.8 | 0.0 | 10 duty | 60 duty |

- **D-term Strategy**: 
  - Roll/Pitch: D-term fed from gyro rate (not finite difference) → cleaner, less noise
  - Yaw: No D-term (rate control, gyro noise too high relative to benefit)

- **Anti-Windup**: Integral clamped to prevent runaway
  - Roll/Pitch: ±15 duty units max integral
  - Yaw: ±10 duty units max integral

- **Output Authority**:
  - Roll/Pitch: ±80 duty can increase left/right or front/rear motors
  - Yaw: ±60 duty can spin CCW or CW motors faster

- **Important Methods**:
  - `init(float dt)`: Set up three PidAdvanced instances with gains above
  - `update(Setpoint, AttitudeDeg, AttitudeRateDps)`: Run PID, return corrections
  - `reset()`: Clear integral state on arm/disarm (prevents transient bias)

---

#### `pid_advanced.cpp / pid_advanced.h` — Single-Axis PID
- **Purpose**: Core PID implementation for one axis (reused 3 times)
- **Features**:
  - Standard `update(measured)`: P + I + integral clamping + output limiting
  - Preferred `updateWithRate(measured, measuredRate)`: D-term from external gyro rate
  - Live `dt` update support (for variable loop timing)

- **Equations**:
  ```
  error = desired - measured
  integral = integral + error * dt  (clamped to ±limit)
  derivative = measuredRate  (D-term fed from gyro, not finite difference)
  
  output = Kp * error + Ki * integral + Kd * derivative
  output = clamp(output, -outputLimit, +outputLimit)
  ```

---

#### `power_distribution.cpp / power_distribution.h` — Motor Mixer
- **Purpose**: Convert PID corrections + base thrust into per-motor duty cycles
- **Quad-X Configuration**:
  ```
      M1 (CCW) ──────── M2 (CW)     Front
           \          /
            \        /
            /        \
           /          \
      M4 (CW) ──────── M3 (CCW)     Rear
      Left                Right
  ```

- **Mixing Algebra**:
  ```
  m1 = thrust - roll_correction - pitch_correction + yaw_correction  (front-left)
  m2 = thrust + roll_correction - pitch_correction - yaw_correction  (front-right)
  m3 = thrust + roll_correction + pitch_correction + yaw_correction  (rear-right)
  m4 = thrust - roll_correction + pitch_correction - yaw_correction  (rear-left)
  ```

- **Clamping**: 
  - All values clamped to [MOTOR_MIN, MOTOR_MAX] range (30–200)
  - If any motor would exceed limits, all are de-saturated uniformly to prevent instability

---

### **System Safety** (`system/`)

#### `flight_system.cpp / flight_system.h`
- **Purpose**: State machine + safety manager
- **Flight Phases**:
  - `BOOT`: System just powered on, IMU calibrating
  - `READY`: Calibration done, waiting for arm delay (default 3s)
  - `ARMING`: Motors ramping up
  - `HOVERING`: Active flight, PID loop running
  - `DISARMING`: Timed mission complete, motors ramping down
  - `HALTED`: Terminal state, all motors off until reboot

- **Safety Checks**:
  - Tilt limits: if roll/pitch > threshold, cut motors
  - IMU health: if I2C fails or accel anomaly, cut motors
  - Hover duration: if flight exceeds timeout, auto-disarm
  - Emergency stop: immediate motor cut + halt state

- **Key Methods**:
  - `init(armDelayMs, hoverDurationMs)`: Set timing (default 3s arm, 15s hover)
  - `update(SafetyInputs)`: Run state machine (call every loop)
  - `shouldCutPower()`: returns true if motors must be zeroed
  - `triggerEmergencyStop(reason)`: Immediate halt

---

### **Hardware Drivers** (`drivers/`)

#### `i2c_bus.cpp / i2c_bus.h`
- **Purpose**: Centralized I2C access layer for all peripherals
- **All I2C communication goes through here** — never call `Wire.beginTransmission()` directly
- **Configuration**:
  - Pins: SDA=GPIO 3, SCL=GPIO 4 (ESP32-S3 Zero defaults)
  - Frequency: 400kHz (Fast Mode, safe for MPU-6050)
  - Timeout: 50ms (long enough for valid transactions, short enough to not stall flight loop)

- **Key Functions**:
  - `init(sdaPin, sclPin, frequencyHz, timeoutMs)`: Set up I2C bus once in setup()
  - `readBytes(deviceAddr, regAddr, outData, len)`: Burst-read with retry logic on transient failure
  - `writeByte(deviceAddr, regAddr, value)`: Single register write
  - `scan()`: Debug function to detect I2C devices (remove in production)

- **Retry Logic**: Transient I2C failures are retried once before returning false

#### `usec_time.cpp / usec_time.h`
- **Purpose**: Microsecond-level timing utility for dt measurement

#### `compat/runtime_compat.cpp / compat/runtime_compat.h`
- **Purpose**: Platform compatibility layer for ESP32-S3 specific APIs

---

### **Power Management** (`power/`)

#### `pm_manager.cpp / pm_manager.h`
- **Purpose**: Battery voltage monitoring, low-battery cutoff, power state tracking
- **Typical Functions**: 
  - `init()`: Initialize ADC for battery voltage
  - `update()`: Read voltage, trigger emergency stop if battery too low
  - Voltage threshold for safe landing (typically 3.0V for LiPo)

---

## Important Calculations & Constants

### **Timing**
- **Control Loop**: 4ms (250Hz)
- **IMU Sample Rate**: 500Hz (SMPLRT_DIV=1, with DLPF enabled Gyro Output Rate = 1kHz)
- **DLPF Configuration**: Accel 44Hz BW, Gyro 42Hz BW (filters motor vibration ~80–200Hz)
- **Safe dt Range**: 0.5ms–50ms (outside this range, use 0.002s default)
- **Arm Delay**: 3000ms (configurable)
- **Hover Duration**: 15000ms (configurable)

### **Sensor Scaling**
- **Accelerometer**: ±8g full-scale → 4096 LSB/g (MPU-6050 datasheet Table 1)
  - Formula: `accel_g = (raw_counts - offset) / 4096.0`
- **Gyroscope**: ±500°/s full-scale → 65.5 LSB/dps (MPU-6050 datasheet Table 2)
  - Formula: `gyro_dps = (raw_counts - offset) / 65.5`

### **Motor Authority**
- **Hover Thrust**: 120/255 = 47% duty (tune per frame weight)
- **Max PID Correction**: ±80 duty for roll/pitch, ±60 duty for yaw
- **Total Authority Ceiling**: thrust + correction ≤ 200 (MOTOR_MAX)
- **Minimum Spin Threshold**: 30 (MOTOR_MIN, enforced to prevent stalling)

### **Complementary Filter**
- **Gyro Trust Factor** (α): 0.98 (trusts gyro 98% per frame)
- **Accel Correction Factor** (1-α): 0.02 (corrects with accel 2% per frame)
- **Accel Magnitude Guard**: 0.7G–1.3G (outside range = skip accel correction)
  - Prevents vibration spikes from corrupting angle estimate

### **PID Saturation**
- **Roll/Pitch Integral Limits**: ±15 duty units (anti-windup)
- **Yaw Integral Limit**: ±10 duty units
- **Roll/Pitch Output Limits**: ±80 duty units (31% authority per axis)
- **Yaw Output Limit**: ±60 duty units (24% authority)

---

## One Control Iteration Flow

```
Main Loop (250Hz, 4ms per cycle)
│
├─ Read IMU from I2C burst
│  ├─ Parse 14 bytes: accel[6] + temp[2] + gyro[6]
│  ├─ Apply calibration offsets
│  └─ Return IMUData struct {ax, ay, az, gx, gy, gz, isHealthy}
│
├─ Call Stabilizer::update(setpoint, imuData)
│  │
│  ├─ Compute live dt from micros() (clamped 0.5–50ms)
│  │
│  ├─ Call SensFusion6::update(gx, gy, gz, ax, ay, az, dt)
│  │  ├─ accelRoll = atan2(ay, √(ax²+az²)) * 180/π
│  │  ├─ accelPitch = atan2(-ax, √(ay²+az²)) * 180/π
│  │  ├─ roll_fused = 0.98*(roll_prev + gx*dt) + 0.02*accelRoll
│  │  ├─ pitch_fused = 0.98*(pitch_prev + gy*dt) + 0.02*accelPitch
│  │  └─ yaw_fused += gz*dt
│  │
│  ├─ Check FlightSystem::shouldCutPower()
│  │  └─ if true → return {0,0,0,0} (safety cut)
│  │
│  ├─ Check imu.isHealthy
│  │  └─ if false → emergencyStop() → return {0,0,0,0}
│  │
│  ├─ Build AttitudeRateDps from raw gyro
│  │
│  ├─ Call ControllerPid::update(setpoint, attitude, rate)
│  │  ├─ rollError = 0° - roll_fused
│  │  ├─ pitchError = 0° - pitch_fused
│  │  ├─ yawRateError = 0°/s - gz
│  │  ├─ Apply PID with anti-windup + output limits
│  │  └─ return {rollCorr, pitchCorr, yawCorr}
│  │
│  ├─ Call PowerDistribution::mixX(HOVER_THRUST=120, corrections)
│  │  ├─ m1 = 120 - rollCorr - pitchCorr + yawCorr (clamp to [30, 200])
│  │  ├─ m2 = 120 + rollCorr - pitchCorr - yawCorr
│  │  ├─ m3 = 120 + rollCorr + pitchCorr + yawCorr
│  │  └─ m4 = 120 - rollCorr + pitchCorr - yawCorr
│  │
│  └─ return {m1, m2, m3, m4}
│
└─ Call setMotorSpeeds(m1, m2, m3, m4)
   └─ Write PWM duty cycles to MOSFET gates via ledcWrite()
```

---

## Compilation & Build System

- **Framework**: PlatformIO
- **Platform**: esp32-s3
- **Arduino Core**: ESP32 Arduino core (≥3.x for `ledcAttach()` API)
- **CPU**: Dual-core 240MHz Xtensa (no FPU)
- **Battery Voltage**: 3.7V nominal (LiPo)
- **Build**: `platformio run --target upload` or use VS Code PlatformIO extension

---

## Tuning Guide

### **IMU Calibration**
1. Place drone flat and level on stable surface
2. Power on — `calibrateIMU()` runs automatically
3. Keep absolutely still for ~1 second (500 samples)
4. If calibration fails, check I2C wiring and MPU-6050 I2C address (should be 0x68)

### **Motor Minimum Threshold (MOTOR_MIN)**
1. Start with MOTOR_MIN = 20
2. Set throttle to 50% and ramp up slowly
3. Increase MOTOR_MIN until motors spin smoothly without stalling under moderate load
4. Typical range: 25–40 for 8520 motors at 3.7V

### **Hover Thrust (HOVER_THRUST)**
1. Start with `HOVER_THRUST = 60` duty
2. Slowly increase in steps of 5 until drone just barely lifts off ground
3. Add 10–20% headroom above liftoff point
4. Typical range: 110–130 for ~35g frame with 8520 motors at 3.7V
5. Record this value and use as new HOVER_THRUST constant

### **PID Tuning** (start with Kp, then Ki, then Kd)
1. **Kp (Proportional Gain)**:
   - Start with Kp = 2 for roll/pitch, Ki = 0, Kd = 0
   - Slowly increase Kp until oscillation begins
   - Back off Kp by 30% (this is your stable starting Kp)
   
2. **Ki (Integral Gain)**:
   - Add Ki ≈ 30% of your final Kp
   - Tests for steady-state lean correction (drone shouldn't drift during hover)
   
3. **Kd (Derivative Gain)**:
   - Fed from gyro rate, not finite differencing
   - Add small Kd (~5% of Kp) for damping if oscillation persists
   - Too high Kd = jerky response to disturbances

4. **Anti-Windup**:
   - Integral limit should be 1/5 to 1/3 of your output limit
   - Prevents integrator saturation

5. **Output Limits**:
   - Keep output limit well below MOTOR_MAX (80 duty = 31% of 255 is typical)
   - Limits how much one axis can dominate

---

## Known Limitations & Future Improvements

- **Yaw Drift**: Without magnetometer, yaw integrates from gyro only and drifts (~1°/s typical). For 15s missions, expect 15° total yaw drift — acceptable for short hover tests
- **Accel Vibration Sensitivity**: Brushed motor vibration can spike accel to 1.3–1.5G; magnitude guard prevents angle corruption but may cause accel correction to lag during aggressive maneuvers
- **No Altitude Hold**: Throttle is entirely manual; stabilizer only does attitude hold (roll/pitch/yaw leveling)
- **No GPS/Outdoor Guidance**: Hover is dead-reckoned; wind and IMU drift cause position drift
- **Motor Nonlinearity**: 8520 brushed motors exhibit dead-band (~10% duty) and velocity-dependent back-EMF; gains are tuned empirically and may need adjustment for different battery voltage/motor model
- **Computational Cost**: Floating-point math on Xtensa (no FPU) means ~250Hz loop is near the practical limit; avoid adding complex algorithms

---

## Summary Table: File Responsibilities

| File | Responsibility | Key Output |
|------|-----------------|-----------|
| main.cpp | Control loop orchestration, timing, lifecycle | MotorMix commands |
| imu.cpp | Raw sensor acquisition, calibration | IMUData (accel, gyro) |
| motors.cpp | PWM output, safety gates | GPIO HIGH/LOW via LEDC |
| receiver.cpp | RC input handling | Setpoint (desired angles) |
| stabilizer.cpp | Pipeline hub, sensor fusion → motor mix | MotorMix (4 duty values) |
| sensfusion6.cpp | Complementary filter (gyro + accel fusion) | AttitudeDeg (roll, pitch, yaw) |
| controller_pid.cpp | Attitude/rate PID loops | ControlOutput (corrections) |
| pid_advanced.cpp | Single-axis PID core | float output (1 axis correction) |
| power_distribution.cpp | Quad-X mixer algebra | MotorMix (4 duty values) |
| flight_system.cpp | State machine, safety checks | Safety gate, phase state |
| i2c_bus.cpp | Centralized I2C driver | bool (read/write success) |
| pm_manager.cpp | Power/battery management | Voltage, low-battery flag |

---

## Quick Start Checklist

- [ ] Verify I2C wiring: SDA=GPIO 3, SCL=GPIO 4, MPU-6050 at 0x68
- [ ] Calibrate IMU: place drone flat, keep still, let calibration run (~1s)
- [ ] Set MOTOR_MIN to spin threshold without stalling
- [ ] Set HOVER_THRUST: start 60, ramp until liftoff, add 10–20%
- [ ] Tune Kp: increase until oscillation, back off 30%
- [ ] Add Ki: ~30% of final Kp
- [ ] Add Kd if needed: for damping (gyro rate, not finite diff)
- [ ] Verify battery voltage monitoring for safe landing
- [ ] Test on rig before free flight

