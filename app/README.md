# Hexapod Robot Control Application

## Overview

This application provides a comprehensive control system for a 6-legged (hexapod) robot. It implements advanced kinematics, gait generation, sensor integration, and balance control to enable smooth and adaptive movement capabilities.

## Features

- **Multiple Gait Patterns**: Tripod, Wave, and Ripple gaits with configurable parameters
- **Adaptive Movement**: Dynamic speed, direction, and height control
- **Sensor Integration**: Support for MPU6050 (6-axis) and ADXL345 (3-axis) IMU sensors
- **Obstacle Detection**: Ultrasonic sensor integration with automatic avoidance
- **Balance Mode**: Self-stabilizing capability using IMU feedback
- **Interactive Control**: Real-time keyboard control interface
- **Diagnostics**: Performance monitoring and hardware verification tools
- **Calibration**: Joint calibration system for mechanical adjustments

## Architecture

The application follows a modular, object-oriented design with several key components:

- **Hexapod**: Hardware abstraction layer for the robot platform
- **Kinematics**: Mathematical model for leg positioning and inverse kinematics
- **Gait**: Movement pattern generation and coordination
- **Controller**: High-level control and command interpretation
- **Calibration**: Servo calibration management
- **Application**: Main program and user interface

## Building the Application

### Prerequisites

- ARM cross-compiler (arm-linux-gnueabihf-g++)
- C++17 compatible compiler
- Standard development libraries

### Compilation

The project uses a Makefile build system with the following targets:

```bash
# Standard build (optimized)
./scripts/build.sh -b app

# Debug build with additional debug information
./scripts/build.sh -b app DEBUG=1

# Run tests for help
./scripts/test.sh -h

# Clean build artifacts
./scripts/build.sh -c app

# Install to target location
./scripts/deploy.sh
```

## Usage

### Controls

| Key     | Function                                 |
|---------|------------------------------------------|
| W / S   | Move forward/backward                    |
| A / D   | Rotate left/right                        |
| I / K   | Raise/lower body                         |
| J / L   | Tilt left/right                          |
| 1 - 3   | Select gait (1=Tripod, 2=Wave, 3=Ripple) |
| + / -   | Increase/decrease speed                  |
| SPACE   | Stop and center legs                     |
| B       | Toggle balance mode                      |
| \[ / \] | Decrease/increase balance sensitivity    |
| U       | Toggle ultrasonic sensor                 |
| T       | Toggle telemetry display                 |
| P       | Toggle performance monitoring            |
| M       | Run servo diagnostics                    |
| H       | Show help                                |
| Q       | Quit                                     |

### Telemetry Display

When enabled, the telemetry shows:
- IMU sensor readings
- Current controller state
- Performance metrics
- Balance mode status

### Balance Mode

The balance mode uses IMU sensor data to automatically adjust the robot's posture in response to tilting or uneven surfaces. This can be fine-tuned using the response factor and deadzone settings.

## Test Programs

The `test/` directory contains individual test programs for specific components:

- **servo.cpp**: Test servo motor control
- **mpu6050.cpp**: Test MPU6050 sensor readings
- **adxl345.cpp**: Test ADXL345 accelerometer readings
- **movement.cpp**: Test movement patterns
- **hcsr04.cpp**: Test ultrasonic distance sensor
- **calibration.cpp**: Test calibration procedures
- **balance.cpp**: Test balance mode functionality

## Safety Features

- Automatic obstacle detection and avoidance
- Graceful signal handling for clean shutdown
- Leg centering on startup and shutdown
- Configurable movement speed limits
