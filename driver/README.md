# Hexapod Robot Driver

## Overview

This directory contains the kernel-level driver for the hexapod robot platform. The driver provides a hardware abstraction layer that interfaces with the servo controllers, IMU sensors, and other hardware components, exposing them through a unified device interface to user applications.

## Features

- **Servo Motor Control**: Precise control of 18 servo motors through the PCA9685 PWM controller
- **Dual IMU Support**:
  - MPU6050 (6-axis: accelerometer + gyroscope)
  - ADXL345 (3-axis accelerometer)
  - Auto-detection capability
- **Joint Calibration**: Hardware-level servo calibration to compensate for mechanical variations
- **Safe Mode Operation**: Automatic protection against invalid positions and commands

## Hardware Components

The driver interfaces with the following hardware components:

| Component | Description               | Interface | Address (configurable) |
|-----------|---------------------------|-----------|------------------------|
| PCA9685_1 | 16-channel PWM controller | I2C       | 0x40                   |
| PCA9685_2 | 16-channel PWM controller | I2C       | 0x41                   |
| MPU6050   | 6-axis IMU sensor         | I2C       | 0x68                   |
| ADXL345   | 3-axis accelerometer      | I2C       | 0x53                   |

## Directory Structure

```
driver/
├── inc/
│   ├── mpu6050.h          # Public header file with IOCTL definitions
│   └── pca9685.h          # PCA9685 PWM controller data structures
├── src/
│   ├── mpu6050.c          # Core driver code
│   ├── pca9685.c          # PCA9685 PWM controller handling code
│   └── main.c             # Main application code
├── Makefile               # Build instructions
└── README.md              # This file
```

## Building and Installation

### Prerequisites

- Linux kernel headers
- Cross-compilation toolchain
- BeagleBone AI device tree (maybe not)

### Build Commands

```bash
./scripts/build.sh -b driver
```

### Installation Steps

1. Copy module:
   ```bash
   ./scripts/deploy.sh
   ```

2. Load module:
   ```bash
   ./scripts/deploy.sh -i
   ```

3. Verify:
   ```bash
   dmesg | grep hexapod
   ls -l /dev/hexapod
   ```

## Driver Interface

### IOCTL Commands

- `SET_LEG_POSITION`: Set servo angles for a specific leg
- `GET_IMU_DATA`: Read IMU data (accelerometer and gyroscope)
- `CALIBRATE`: Set servo calibration offsets
- `CENTER_ALL`: Center all servos to neutral position

### Data Structures

- `hexapod_leg_joint`: Contains hip, knee, ankle angles
- `hexapod_leg_cmd`: Contains leg number and joint angles
- `hexapod_calibration`: Contains leg number and joint offsets
- `hexapod_imu_data`: Contains accelerometer and gyroscope data

### Device Node

- Path: `/dev/hexapod`
- Permissions: 666
- Mode: Character device

## Debugging

Enable debug messages:

```bash
echo 1 > /sys/module/hexapod_driver/parameters/debug
```

## Development

### Adding Features

1. Define IOCTL command in `src/main.c`
2. Implement handler in `src/main.c`
3. Add to user documentation

### Testing

```bash
# Load with debug
./install.sh debug=1

# Monitor messages
dmesg -w
```

### Error Handling

- I2C communication errors
- Invalid parameters
- Resource cleanup
- User input validation
