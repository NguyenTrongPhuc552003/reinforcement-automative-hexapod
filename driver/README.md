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

The driver interfaces with the following hardware components connected via I2C Bus 2 on BeagleBone Black:

| Component | Description               | Interface | Address (configurable) |
|-----------|---------------------------|-----------|------------------------|
| PCA9685_1 | 16-channel PWM controller | I2C2      | 0x40                   |
| PCA9685_2 | 16-channel PWM controller | I2C2      | 0x41                   |
| MPU6050   | 6-axis IMU sensor         | I2C2      | 0x68                   |
| ADXL345   | 3-axis accelerometer      | I2C2      | 0x53                   |

### BeagleBone Black I2C Pin Configuration

I2C Bus 2 is used for all sensor and actuator communication:
- **SCL (Clock)**: P9_19
- **SDA (Data)**: P9_20
- **VCC**: P9_3 (3.3V) or P9_5/P9_6 (VDD_5V)
- **GND**: P9_1 or P9_2

## Directory Structure

```t
driver/
├── dts/
│   └── Black/
│       ├── BBB-MPU6050-overlay.dts    # MPU6050 device tree overlay for BeagleBone Black
│       └── BBB-PCA9685-overlay.dts    # PCA9685 device tree overlay for BeagleBone Black
├── inc/
│   ├── mpu6050.h          # MPU6050 IMU sensor data structures
│   └── pca9685.h          # PCA9685 PWM controller data structures
│   └── adxl345.h          # ADXL345 accelerometer data structures
│   └── servo.h            # Servo motor data structures
├── src/
│   ├── mpu6050.c          # MPU6050 IMU sensor handling code
│   ├── pca9685.c          # PCA9685 PWM controller handling code
│   ├── adxl345.c          # ADXL345 accelerometer handling code
│   └── servo.c            # Servo motor handling code
│   └── main.c             # Main application code
├── Makefile               # System build instructions
└── README.md              # This file
```

## Building and Installation

### Prerequisites

- Linux kernel headers
- Cross-compilation toolchain
- BeagleBone Black device tree overlays

### Build Commands

```bash
./scripts/build.sh -b driver
```

### Device Tree Overlay Setup

Before loading the driver, install and enable the device tree overlays:

```bash
# Copy overlay files to /lib/firmware
sudo cp driver/dts/Black/*.dtbo /lib/firmware/

# Enable overlays in /boot/uEnv.txt
sudo echo "dtb_overlay=/lib/firmware/BBB-PCA9685-overlay.dtbo" >> /boot/uEnv.txt
sudo echo "dtb_overlay=/lib/firmware/BBB-MPU6050-overlay.dtbo" >> /boot/uEnv.txt

# Reboot to apply overlays
sudo reboot
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
