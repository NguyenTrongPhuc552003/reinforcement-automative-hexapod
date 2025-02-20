# Hexapod Kernel Driver

This directory contains the kernel driver for the hexapod robot. The driver provides interfaces for controlling servos via PCA9685 PWM controllers and reading sensor data from the MPU6050 IMU, all communicating over I2C.

## Directory Structure

```
kernel_driver/
├── include/           # Header files
│   ├── hexapod.h     # Main driver interface
│   ├── mpu6050.h     # MPU6050 registers and functions
│   ├── pca9685.h     # PCA9685 registers and functions
│   └── servo.h       # Servo control definitions
├── src/              # Source files
│   ├── hexapod.c     # Main character device driver
│   ├── mpu6050.c     # MPU6050 sensor driver
│   ├── pca9685.c     # PCA9685 PWM controller driver
│   └── servo.c       # Servo control logic
└── Makefile
```

## Hardware Requirements

### I2C Configuration
The driver uses I2C bus 3 on the BeagleBone Black:
- SCL: P9_21
- SDA: P9_22
- VDD: P9_3 (3.3V)
- GND: P9_1

### Connected Devices
1. MPU6050 IMU Sensor
   - I2C Address: 0x68
   - Provides accelerometer and gyroscope data
   - Temperature sensor included

2. PCA9685 PWM Controllers
   - Controller #1: I2C Address 0x40 (Legs 0-2)
   - Controller #2: I2C Address 0x41 (Legs 3-5)
   - 16 channels each
   - PWM frequency: 50Hz for servo control

3. Servo Configuration
   - 18 servos total (3 per leg × 6 legs)
   - Operating voltage: 5V
   - PWM range: 150-600 (corresponds to -90° to +90°)
   - Default frequency: 50Hz

## Module Architecture

The driver consists of four main components:

1. **Main Driver (hexapod.c)**
   - Character device interface
   - IOCTL command handling
   - Module initialization and cleanup
   - Device node: /dev/hexapod

2. **MPU6050 Driver (mpu6050.c)**
   - Sensor initialization
   - Register reading/writing
   - Data conversion and processing
   - Error handling and recovery

3. **PCA9685 Driver (pca9685.c)**
   - PWM controller initialization
   - Channel control
   - Frequency setting
   - Multiple device support

4. **Servo Control (servo.c)**
   - Angle to PWM conversion
   - Joint limit enforcement
   - Smooth movement control
   - Multi-servo coordination

## Building

The driver is built using a Docker-based cross-compilation environment:
```bash
./scripts/build.sh
```

This creates:
- hexapod_driver.ko (Kernel module)
- servo_test (User space test program)

## Installation

Deploy and install on the BeagleBone:
```bash
./scripts/deploy.sh
```

The installation script will:
1. Check for I2C bus availability
2. Install required packages (i2c-tools)
3. Load the kernel module
4. Create device node (/dev/hexapod)
5. Set appropriate permissions
6. Install user space test program

## IOCTL Interface

The driver provides the following IOCTL commands:

1. **IOCTL_SET_SERVO**
   - Set individual servo angle
   - Parameters: leg_id, joint_id, angle
   - Angle range: -90 to +90 degrees

2. **IOCTL_SET_PATTERN**
   - Set movement pattern
   - Parameters: pattern_id, speed, direction
   - Patterns: TRIPOD (0), WAVE (1), RIPPLE (2)

3. **IOCTL_GET_MPU6050**
   - Read sensor data
   - Returns: accelerometer, gyroscope, temperature
   - Data in raw format (needs conversion)

See `include/hexapod.h` for detailed command definitions and structures.

## Error Handling

The driver includes comprehensive error handling:
- I2C bus validation
- Device detection and recovery
- Invalid parameter checking
- Resource cleanup
- Detailed error reporting via dmesg

## Debugging

1. Check module status:
```bash
lsmod | grep hexapod
dmesg | grep hexapod
```

2. Verify I2C devices:
```bash
i2cdetect -y -r 0
```

3. Monitor device node:
```bash
ls -l /dev/hexapod
```

## License
This driver is licensed under the GPL License.
