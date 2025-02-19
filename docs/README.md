# Hexapod Documentation

This directory contains comprehensive documentation for the hexapod robot project.

## Directory Structure

```
docs/
├── hardware/           # Hardware documentation
│   ├── schematics/    # Circuit diagrams and PCB layouts
│   └── assembly/      # Assembly instructions and photos
├── software/          # Software documentation
│   ├── architecture/  # System architecture and design
│   └── protocols/     # Communication protocols
└── api/               # API documentation
    ├── kernel/        # Kernel driver API
    └── user/          # User-space API
```

## Hardware Documentation

### Specifications
- BeagleBone AI board
- 18x MG996R servos
- 2x PCA9685 PWM controllers
- MPU6050 IMU sensor
- Power supply requirements

### Connections
- I2C bus configuration
- Servo wiring diagram
- Power distribution

## Software Documentation

### Architecture
- Kernel driver design
- User-space application design
- Communication flow
- Data structures

### Protocols
- I2C protocol details
- UART protocol
- IOCTL interface

## API Documentation

### Kernel Driver API
- Device node interface
- IOCTL commands
- Error codes

### User-Space API
- Library functions
- Configuration options
- Movement patterns
