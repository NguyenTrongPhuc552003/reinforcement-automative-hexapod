# Hexapod Robot Control System

A comprehensive control system for a six-legged robot (hexapod) using BeagleBone Black, featuring both kernel-space drivers and user-space applications.

![Hexapod Robot](docs/images/hexapod.jpg)

## Project Overview

This project implements a complete software stack for controlling a 6-legged robot with 18 servo motors (3 per leg), including:

- Kernel drivers for hardware communication (PCA9685 PWM controllers and MPU6050 IMU)
- User-space libraries for kinematics and gait generation
- Calibration system for precise servo adjustment
- Test utilities for system validation
- Comprehensive documentation

### Key Features

- **Multiple Gait Patterns**: Tripod, Wave, and Ripple locomotion patterns
- **Real-time IMU Feedback**: Motion sensing for balance and orientation
- **Advanced Kinematics**: Precise leg positioning using forward and inverse kinematics
- **Calibration System**: Easy adjustment for hardware variations
- **Hardware Abstraction**: Clean separation between application and hardware layers
- **Testing Tools**: Comprehensive validation of all components

## System Architecture

```
.
├── kernel_driver/    # Linux kernel device drivers
├── user_space/      # User applications and libraries
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
   cd kernel_driver
   make
   ```

2. Install the kernel module:
   ```bash
   sudo insmod hexapod_driver.ko
   ```

3. Build user-space applications:
   ```bash
   cd user_space
   make
   ```

4. Run the test utility:
   ```bash
   sudo ./test_servo
   ```

## Development

See individual README files in subdirectories for detailed development guides:
- [Kernel Driver README](kernel_driver/README.md)
- [User Space README](user_space/README.md)
- [Documentation](docs/README.md)

## Testing

Run the test suite:
```bash
cd user_space/test
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