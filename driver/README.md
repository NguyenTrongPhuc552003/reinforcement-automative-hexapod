# Hexapod Kernel Driver

Linux kernel driver for hexapod robot control, handling PWM servo control and IMU sensor integration.

## Components

- MPU6050 IMU driver
- PCA9685 PWM controller driver
- Character device interface
- I2C communication layer

## Hardware Interface

### I2C Configuration
- Bus: I2C-3
- Pins: P9_19 (SCL), P9_20 (SDA)
- Addresses:
  - MPU6050: 0x68
  - PCA9685 #1: 0x40
  - PCA9685 #2: 0x41

### PWM Configuration
- Frequency: 50Hz
- Resolution: 12-bit
- Pulse range: 150-600 (corresponds to -90° to +90°)

## Building

Prerequisites:
- Linux kernel headers
- Cross-compilation toolchain
- BeagleBone AI device tree (maybe not)

Build commands:
```bash
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
```

## Installation

1. Copy module:
   ```bash
   scp hexapod_driver.ko debian@beaglebone:~/
   ```

2. Load module:
   ```bash
   sudo insmod hexapod_driver.ko
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
1. Define IOCTL command in `include/hexapod.h`
2. Implement handler in `src/hexapod.c`
3. Add to user documentation

### Testing
```bash
# Load with debug
sudo insmod hexapod_driver.ko debug=1

# Monitor messages
dmesg -w
```

### Error Handling
- I2C communication errors
- Invalid parameters
- Resource cleanup
- User input validation
