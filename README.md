# Hexapod Robot Control System

A comprehensive control system for a six-legged robot (hexapod) using BeagleBone Black, featuring both kernel-space drivers and user-space applications.

![Hexapod Robot](resource/overview.jpg)

## Project Overview

This project implements a complete software stack for controlling a 6-legged robot with 18 servo motors (3 per leg), including:

- **Kernel-Space Components**:
  - Linux kernel driver for servo control (PCA9685 PWM controllers)
  - IMU sensor driver (MPU6050) for orientation sensing
  - Hardware abstraction layer for unified device access
  
- **User-Space Components**: 
  - Kinematics library for precise leg positioning
  - Gait generation for various walking patterns
  - Calibration system for mechanical offset compensation
  - Interactive control interface

## System Architecture

```
.
├── driver/          # Linux kernel device drivers
├── app/             # User applications and libraries
├── docs/            # Documentation
└── scripts/         # Build and utility scripts
```

## Features

- Multiple gait patterns (tripod, wave, ripple)
- Real-time IMU feedback
- Inverse kinematics for precise leg control
- Hardware abstraction layer
- Comprehensive test suite
- Interactive debugging tools

## Prerequisites

- BeagleBone Black running Linux 4.14+
- I2C enabled (bus 3)
- 18x servo motors (MG996R recommended)
- MPU6050 IMU sensor
- 2x PCA9685 PWM controllers

## Quick Start

1. Build the kernel module:
   ```bash
   cd driver
   make
   ```

2. Install the kernel module:
   ```bash
   sudo insmod hexapod_driver.ko
   ```

3. Build user-space applications:
   ```bash
   cd app
   make
   ```

4. Run the test utility:
   ```bash
   sudo ./test_servo
   ```

## Development

See individual README files in subdirectories for detailed development guides:
- [Kernel Driver README](driver/README.md)
- [User Space README](app/README.md)
- [Documentation](docs/README.md)

## Testing

Run the test suite:
```bash
cd app/test
make test
```

Individual tests:
```bash
./test_servo     # Test servo control
./test_mpu6050   # Test IMU sensor
./test_gait      # Test gait patterns
```

## Documentation

- [Hardware Setup](docs/hardware.md)
- [Software Architecture](docs/architecture.md)
- [API Documentation](docs/api/README.md)
- [Testing Guide](docs/testing.md)

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the GPL License - see the [LICENSE](LICENSE) file for details.

## Authors

- [StrongFood]

## Acknowledgments

- BeagleBoard.org
- Linux Kernel Community