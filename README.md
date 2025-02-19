# Hexapod Robot Project

This project implements a hexapod robot control system using BeagleBone AI. The system consists of kernel drivers for hardware control and user-space applications for high-level control.

## Project Structure

```
.
├── kernel_driver/     # Kernel-space drivers
│   ├── include/      # Header files
│   └── src/         # Source files for kernel modules
├── user_space/       # User-space applications
│   ├── include/     # User-space headers
│   └── src/        # User-space source files
├── scripts/         # Build and deployment scripts
├── deploy/          # Deployment artifacts
├── docs/           # Documentation
│   ├── hardware/   # Hardware specifications
│   ├── software/   # Software architecture
│   └── api/        # API documentation
└── tools/          # Development tools and utilities

## Components

### Kernel Driver
The kernel driver provides low-level hardware control through multiple loadable kernel modules:
- I2C communication for sensors and servo controller
- UART communication for debugging
- MPU6050 sensor interface
- PCA9685 PWM controller
- Servo control interface
See [kernel_driver/README.md](kernel_driver/README.md) for details.

### User Space Application
The user-space application provides:
- High-level robot control
- Movement pattern generation
- Sensor data processing
- Debug interface
See [user_space/README.md](user_space/README.md) for details.

## Getting Started

1. Set up the development environment:
```bash
./scripts/setup_env.sh
```

2. Build the kernel drivers:
```bash
./scripts/build.sh
```

3. Deploy to BeagleBone:
```bash
./scripts/deploy.sh
```

## Development

### Prerequisites
- Docker for build environment
- BeagleBone AI board
- MPU6050 sensor
- PCA9685 PWM controller
- 18 servos (MG996R or compatible)

### Building
All builds are done in a Docker container to ensure consistency:
```bash
./scripts/build.sh      # Build all components
./scripts/test.sh       # Run tests
./scripts/deploy.sh     # Deploy to BeagleBone
```

## Documentation
- [Hardware Setup](docs/hardware/README.md)
- [Software Architecture](docs/software/README.md)
- [API Documentation](docs/api/README.md)

## License
This project is licensed under the GPL License - see the [LICENSE](LICENSE) file for details.