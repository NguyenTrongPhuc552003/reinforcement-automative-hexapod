# Hexapod Robot Project

This project implements a hexapod robot control system using BeagleBone Black. The system consists of kernel drivers for hardware control and user-space applications for high-level control.

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
└── docs/           # Documentation
```

## Hardware Requirements

### BeagleBone Black
- Linux kernel version: 4.14.108-ti-r144
- I2C bus 0 enabled (P9_17 and P9_18 pins)
- 5V power supply for servos

### Sensors and Actuators
- MPU6050 IMU sensor (I2C address: 0x68)
- 2x PCA9685 PWM controllers (I2C addresses: 0x40, 0x41)
- 18x servos (MG996R or compatible)

### Connections
1. I2C Bus 0:
   - SCL: P9_17
   - SDA: P9_18
   - VDD: P9_3 (3.3V)
   - GND: P9_1

2. Servo Power:
   - 5V external power supply
   - Common ground with BeagleBone

## Software Components

### Kernel Driver
The kernel driver (`hexapod_driver.ko`) provides:
- Character device interface (/dev/hexapod)
- I2C communication for MPU6050 and PCA9685
- Servo control and calibration
- IMU data reading and processing

### User Space Application
The user-space application provides:
- Servo testing and control
- Movement pattern generation
- IMU data reading
- Basic debugging interface

## Building and Installation

### Prerequisites
- Docker for cross-compilation
- SSH access to BeagleBone Black
- I2C tools on BeagleBone (`apt-get install i2c-tools`)

### Building
```bash
# Build everything (kernel module and user space)
./scripts/build.sh
```

### Deployment
1. Configure BeagleBone IP in `scripts/deploy.sh`:
```bash
BEAGLEBONE_IP="192.168.1.9"  # Change to your BeagleBone's IP
```

2. Deploy and install:
```bash
./scripts/deploy.sh
```

### Verification
After installation, verify:
1. I2C devices:
```bash
i2cdetect -y -r 0
```

2. Kernel module:
```bash
lsmod | grep hexapod
ls -l /dev/hexapod
```

3. Test servo movement:
```bash
servo_test
```

## Development

### Adding New Features
1. Kernel Driver:
   - Add new functionality in `kernel_driver/src/`
   - Update IOCTL commands in `kernel_driver/include/hexapod.h`
   - Rebuild using `./scripts/build.sh`

2. User Space:
   - Add new programs in `user_space/src/`
   - Update Makefile if needed
   - Rebuild using `./scripts/build.sh`

### Debugging
1. Kernel messages:
```bash
dmesg | grep hexapod
```

2. I2C communication:
```bash
i2cdetect -y -r 0
i2cdump -y 0 0x68  # MPU6050
i2cdump -y 0 0x40  # PCA9685 #1
```

## License
This project is licensed under the GPL License - see the [LICENSE](LICENSE) file for details.