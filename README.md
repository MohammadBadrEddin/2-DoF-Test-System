# 2-DoF-robot-arm
## Status
🚧 Work in Progress

## Overview
This project provides the software for controlling a **2-DoF robotic arm** driven by stepper motors.
The setup includes:
- **2 × Stepper motors** (controlled via A4988 drivers)
- **1 × AS5600 magnetic rotary sensor** (I²C)
- **1 × Digital Hall sensor** (used temporarily for yaw axis feedback)
- Modular C++ codebase for easy extension

> **I²C multiplexer (Goal):** A TCA9548A will be required when using two AS5600 sensors on the same I²C bus.
Currently, the code runs with **one AS5600 sensor + one digital Hall sensor**.

## Features
- Object-oriented motor control (`PitchMotor`, `YawMotor`)
- Position feedback via magnetic angle sensing (AS5600)
- Homing routines for both axes (recommended on first startup to ensure reference positions)
- Angle-to-step and step-to-angle conversion
- Ready for integration with multiple sensors and drivers
- Current motion range is limited for both axes to prevent hardware damage during testing

## Build & Upload
This project uses **PlatformIO** in VS Code

1. Install [PlatformIO](https://platformio.org/install/ide?install=vscode)
2. Connect your Arduino (e.g., Nano with ATmega328P)
3. Build the code with the ✔️ (checkmark) button in VS Code
4. Upload with the → (arrow) button

## Status
🚧 **In Progress** – This codebase works with:
- Pitch: stepper + AS5600
- Yaw: stepper + digital Hall sensor
Support for **dual AS5600 sensors with a TCA9548A multiplexer** is planned.

## TODO
- [ ] Add TCA9548A I²C multiplexer support for dual AS5600 sensors
- [ ] Replace digital Hall sensor with a second AS5600
- [ ] Add schematics and wiring documentation  
- [ ] Upload 3D CAD model of the test system  
