# Drone Testing Protocol: Project Hop
This document outlines the systematic testing procedure for the ESP32-based flight controller. Following this bottom-up approach is mandatory to ensure hardware integrity and operator safety.

## Prerequisites
#### Hardware: 
    MPU6050 calibrated and rigidly mounted to the frame.
#### Firmware: 
    ESP32 flashed with the latest control logic.
#### Safety: 
    Remove all propellers for Phases 1 through 3.
#### Power: 
    Use a high-discharge LiPo battery. Bench power supplies often fail to provide the current bursts required for motor activation.

## Phase 1: Sensor Fusion and Logic Validation
*Objective* : Confirm the Inertial Measurement Unit (IMU) data accurately reflects physical orientation.
I2C Verification: Run an I2C scanner to ensure the MPU6050 is communicating on the correct address (typically 0x68).
Axis Mapping: * Rotate the nose down: Confirm the Pitch value increases.
Roll the frame to the right: Confirm the Roll value increases.
Note: If values are inverted or swapped, update the axis mapping in your sensor processing function before proceeding.
Loop Timing: Verify the system cycle time. For stable flight on an ESP32, the PID loop must execute at a consistent frequency between 200Hz and 400Hz.

## Phase 2: Motor Mixing and PID Directionality
*Objective*: Ensure the PID controller stabilizes the craft rather than amplifying errors.
WARNING: PROPELLERS MUST BE REMOVED.
Static Motor Test: Spin each motor individually at low throttle to verify:
Correct physical position (e.g., Motor 1 is Front-Right).
Correct rotation direction (Clockwise vs. Counter-Clockwise) according to your frame configuration.
Response Verification:
Arm the drone and set throttle to 15%.
Physically tilt the drone forward by hand.
*Requirement* : The Front motors must increase in RPM to counteract the movement.
*Failure State* : If the back motors increase, the PID logic is inverted and will cause a "death roll" upon takeoff.

## Phase 3: Stabilization and Safety Failsafes
*Objective* : Test the "stiffness" of the stabilization and the reliability of the kill-switch.
The Stiffness Test:
Firmly grip the drone from the bottom of the frame (keep clear of motor shafts).
Increase throttle until the drone feels "light."
Introduce manual disturbance (tilt the drone). The drone should fight your hand to return to level. If it vibrates or over-corrects, the Proportional (Kp) gain is too high.
Communication Failsafe:
While motors are spinning, simulate a signal loss (e.g., turn off the controller or disconnect the Serial link).
The motors must stop immediately. Do not attempt flight until this is 100% reliable.

## Phase 4: Controlled Vertical Hop
*Objective* : Achieve a short, stabilized vertical lift-off.
Tethering: Secure the drone using a 4-point tether system. Tie each arm to a heavy stationary base with approximately 10cm of slack. This prevents the drone from drifting into obstacles or flipping if the tune is unstable.
Execution:
Place the drone on a flat, level surface.
Increase throttle smoothly until the drone hovers within the slack of the tethers.
Observe for oscillations. If the craft "hunts" for level, refine the PID Integral and Derivative terms.

## Directory Structure: 
