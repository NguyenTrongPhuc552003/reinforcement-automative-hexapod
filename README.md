# Hexapod Robot Driver

This project implements a Linux kernel driver for controlling a hexapod robot using BeagleBone Black. The driver interfaces with an MPU6050 IMU sensor for orientation sensing and PCA9685 PWM controller for servo control.

## Features

- MPU6050 IMU driver for orientation sensing
- PCA9685 PWM controller driver for servo control
- Character device interface for user-space communication
- Platform driver integration
- I2C bus communication
- Automated build system using Docker

## Prerequisites

- BeagleBone Black running Linux kernel 4.14.108-ti-r144
- Docker (for cross-compilation)
- Git
- Make
- I2C enabled on bus 3 (P9_19 and P9_20 pins)

## Hardware Setup

1. Connect MPU6050 to I2C bus 3:
   - SDA -> P9_20
   - SCL -> P9_19
   - VCC -> 3.3V
   - GND -> GND

2. Connect PCA9685 to I2C bus 3:
   - SDA -> P9_20
   - SCL -> P9_19
   - VCC -> 5V
   - GND -> GND

## Development Setup

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/hexapod-driver.git
   cd hexapod-driver
   ```

2. Run the setup script:
   ```bash
   ./scripts/setup_env.sh
   ```

   This will:
   - Check for required tools
   - Create necessary directories
   - Build the Docker image for cross-compilation
   - Set up git hooks
   - Create build scripts

3. Build the driver:
   ```bash
   ./build.sh
   ```

## Deployment

1. Copy the built files to your BeagleBone:
   ```bash
   scp deploy/* debian@beaglebone:~/hexapod_driver/
   ```

2. SSH into your BeagleBone:
   ```bash
   ssh debian@beaglebone
   ```

3. Install the driver:
   ```bash
   cd ~/hexapod_driver
   sudo ./install.sh
   ```

## Usage

### Reading Sensor Data

The driver creates a character device at `/dev/hexapod`. Read from this device to get sensor data:

```bash
cat /dev/hexapod
```

Output format:
```
ax:1234 ay:5678 az:9012 gx:3456 gy:7890 gz:1234 temp:2345
```

### User Space Programs

The `user_space` directory contains example programs:
- `servo_test`: Test servo movements
- `mpu6050_test`: Test IMU readings
- `movement_test`: Test hexapod movements

Build and run:
```bash
cd user_space
make
./bin/servo_test
```

## Development

### Project Structure

```
.
├── kernel_driver/
│   ├── include/         # Header files
│   ├── src/            # Driver source files
│   └── device_tree/    # Device tree overlays
├── user_space/
│   ├── include/        # User space headers
│   └── src/           # Test programs
├── scripts/
│   ├── setup_env.sh   # Development setup
│   ├── build.sh       # Build script
│   └── clean.sh       # Cleanup script
└── deploy/            # Built files
```

### Adding New Features

1. Modify driver files in `kernel_driver/src/`
2. Update headers in `kernel_driver/include/`
3. Build using `./build.sh`
4. Test on BeagleBone using `install.sh`

## Troubleshooting

### Common Issues

1. Module not loading:
   - Check dmesg: `dmesg | tail`
   - Verify I2C bus: `i2cdetect -y 3`
   - Check kernel version: `uname -r`

2. No sensor data:
   - Check connections
   - Verify device permissions
   - Check I2C addresses

### Debug Output

Enable debug messages:
```bash
echo 1 > /sys/module/hexapod_driver/parameters/debug
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the GPL License - see the LICENSE file for details.

## Authors

- Your Name - Initial work

## Acknowledgments

- BeagleBoard.org for the excellent platform
- Linux kernel community for documentation