# Hexapod Robot Control System

A BeagleBone AI-based hexapod robot control system with kernel-level drivers for precise servo control and motion sensing.

## Features

- Kernel-level driver for precise control of 18 servos via dual PCA9685 PWM controllers
- MPU6050 IMU integration for motion sensing and stability control
- UART communication for remote control
- User-space utilities for testing and control
- Docker-based cross-compilation environment

## Hardware Requirements

- BeagleBone AI board
- 2x PCA9685 PWM controllers (I2C addresses: 0x40, 0x41)
- MPU6050 IMU sensor
- 18x Servo motors (recommended: MG996R or similar)
- Power supply (recommended: 6-7.4V for servos, separate 5V for logic)

## Directory Structure

```
.
├── kernel_driver/    # Kernel module source code
├── user_space/      # User space control programs
├── scripts/         # Build and utility scripts
├── deploy/          # Deployment artifacts
└── docs/           # Documentation
```

## Building

### Prerequisites

- Docker
- Make
- Git

### Build Instructions

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd reinforcement-automative-hexapod
   ```

2. Build the kernel module and user space programs:
   ```bash
   ./scripts/build.sh
   ```

3. Clean build artifacts:
   ```bash
   ./scripts/build.sh clean
   ```

## Installation

1. Copy the deployment package to your BeagleBone AI:
   ```bash
   scp deploy/hexapod-install.tar.gz debian@<beaglebone-ip>:~
   ```

2. On the BeagleBone AI, extract and install:
   ```bash
   tar xf hexapod-install.tar.gz
   cd hexapod-install
   sudo ./install.sh
   ```

## Usage

### Loading the Kernel Module

```bash
sudo modprobe hexapod
```

### Testing Servo Movement

```bash
# Test individual servo
servo_test -s <servo-id> -a <angle>

# Run wave pattern test
servo_test -w

# Run sequential movement test
servo_test -q
```

## Development

### Adding New Movement Patterns

1. Define the pattern in `kernel_driver/include/servo.h`
2. Implement the pattern in `kernel_driver/src/servo.c`
3. Add user space control in `user_space/src/servo_test.c`

### Debugging

- Check kernel logs: `dmesg | tail`
- Monitor I2C traffic: `i2cdump -y 2 0x40`
- Test servo response: `servo_test -d`

## Troubleshooting

### Common Issues

1. Servos not responding:
   - Check I2C addresses
   - Verify power supply voltage
   - Check servo signal connections

2. MPU6050 errors:
   - Verify I2C connection
   - Check sensor orientation
   - Validate initialization sequence

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the GPL License - see the LICENSE file for details.

## Authors

- Your Name
- Contributors