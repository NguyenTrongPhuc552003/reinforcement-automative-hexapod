# Hexapod Robot Control System

A comprehensive control system for hexapod robots featuring kernel-level hardware drivers, user space applications, and reinforcement learning capabilities.

## Package Contents

This installation package includes:

### Kernel Driver (`/lib/modules/*/extra/`)
- **hexapod_driver.ko**: Low-level hardware interface kernel module
- Provides device file interface at `/dev/hexapod`
- Supports PCA9685 PWM controller and IMU sensors

### Applications (`/usr/local/bin/`)  
- **hexapod_app**: Main interactive control application
- **test_***: Hardware testing and diagnostic utilities
- Complete keyboard-controlled interface for robot operation

### Utilities (`/opt/hexapod/`)
- **install.sh**: Kernel module management
- **monitor.sh**: System monitoring and diagnostics
- Configuration and calibration tools

## Quick Start

1. **Verify Installation**
   ```bash
   lsmod | grep hexapod_driver
   ls -l /dev/hexapod
   ```

2. **Run Main Application**
   ```bash
   sudo hexapod_app
   ```

3. **Test Hardware Components**
   ```bash
   sudo test_mpu6050    # Test IMU sensor
   sudo test_servo      # Test servo motors  
   sudo test_balance    # Test balance system
   ```

## Hardware Requirements

- BeagleBone AI or compatible ARM platform
- PCA9685 16-channel PWM driver
- MPU6050 or ADXL345 IMU sensor
- 18 servo motors (3 per leg × 6 legs)
- HC-SR04 ultrasonic sensor (optional)

## System Requirements

- Linux kernel 4.14+ with I2C support
- ARM architecture (armhf/arm64)
- Root access for hardware control
- I2C interface enabled in device tree

## Documentation

Detailed documentation available in:
- `/usr/local/bin/README.md` - Application usage
- `/opt/hexapod/README.md` - Utility scripts  
- `/lib/modules/*/extra/README.md` - Kernel module

## Troubleshooting

**Module Loading Issues:**
```bash
sudo dmesg | grep hexapod
sudo modprobe hexapod_driver
```

**Permission Errors:**
```bash
sudo chmod 666 /dev/hexapod
sudo usermod -a -G dialout $USER
```

**Hardware Detection:**
```bash
sudo i2cdetect -r 1
sudo /opt/hexapod/monitor.sh
```

## Updates and Support

- Project repository: https://github.com/your-repo/hexapod
- Issue tracker: https://github.com/your-repo/hexapod/issues
- Documentation: https://hexapod-docs.example.com

## Version Information

Check installed version:
```bash
dpkg -l | grep hexapod
modinfo hexapod_driver | grep version
```

---
© 2024 Hexapod Development Team. Licensed under MIT License.
