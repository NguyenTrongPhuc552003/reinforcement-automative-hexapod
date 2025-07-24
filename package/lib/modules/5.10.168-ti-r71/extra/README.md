# Hexapod Driver Kernel Module

This directory contains the hexapod_driver.ko kernel module for the Hexapod Robot Control System.

## Purpose
The kernel module provides low-level hardware interface for:
- Servo motor control via PCA9685 PWM controller
- IMU sensor data reading (MPU6050/ADXL345)
- Device file interface (/dev/hexapod)

## Installation
The kernel module is automatically installed and loaded during package installation.

## Manual Operations
```bash
# Load module manually
sudo insmod hexapod_driver.ko

# Unload module
sudo rmmod hexapod_driver

# Check if loaded
lsmod | grep hexapod_driver

# View module information
modinfo hexapod_driver.ko
```

## Device Interface
Once loaded, the module creates `/dev/hexapod` device file for user space communication.

## Troubleshooting
- Ensure I2C is enabled in device tree
- Check dmesg for loading errors: `dmesg | grep hexapod`
- Verify hardware connections before loading module
