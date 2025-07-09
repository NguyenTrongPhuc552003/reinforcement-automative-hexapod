# Hexapod Robot - Quick Start Guide

This guide will help you quickly set up and operate the reinforcement learning-enabled hexapod robot.

## Prerequisites

- BeagleBone Black or compatible single-board computer
- Properly assembled hexapod with 18 servos (3 per leg)
- IMU sensor (MPU6050 or ADXL345)
- Optional: Ultrasonic distance sensor
- Power supply (7.4V-12V recommended)

## Installation

### Option 1: Using Pre-built Image

1. Download the pre-built image from our repository
2. Flash the image to a microSD card
3. Insert the card into your BeagleBone and power on
4. The system will boot automatically

### Option 2: Manual Installation

1. Install dependencies:
   ```bash
   sudo apt update
   sudo apt install -y build-essential cmake git docker
   ```

2. Clone the repository:
   ```bash
   git clone https://github.com/NguyenTrongPhuc552003/reinforcement-automative-hexapod.git
   cd reinforcement-automative-hexapod
   ```

3. Build the project:
   ```bash
   ./scripts/build.sh
   ```

4. Deploy the applications to your BeagleBone:
   ```bash
   ./scripts/deploy.sh
   ```

## Hardware Setup

1. Connect the servo controller (PCA9685) to I2C bus
2. Connect the IMU sensor (MPU6050 or ADXL345) to I2C bus
3. Connect ultrasonic sensor (optional) to designated GPIO pins
4. Connect power supply to the servo power rail

## Running the Hexapod

1. Start the hexapod driver:
   ```bash
   ./install.sh
   ```

2. Launch the hexapod application:
   ```bash
   ./hexapod_app
   ```

3. Test the hexapod's movement:
   ```bash
   ./test_movement
   ```

## Basic Controls

Once the application is running, use the following keyboard controls:

| Key       | Function                         |
|-----------|----------------------------------|
| W         | Walk forward                     |
| S         | Walk backward                    |
| A         | Rotate left                      |
| D         | Rotate right                     |
| I / K     | Raise/lower body                 |
| J / L     | Tilt left/right                  |
| 1 / 2 / 3 | Switch gait (Tripod/Wave/Ripple) |
| +/-       | Increase/decrease speed          |
| Space     | Stop and center legs             |
| B         | Toggle balance mode              |
| [ / ]     | Adjust balance sensitivity       |
| U         | Toggle ultrasonic sensor         |
| T         | Toggle telemetry display         |
| P         | Toggle performance monitoring    |
| H         | Display help                     |
| Q         | Quit application                 |

## Advanced Features

### Balance Mode

Enable automatic balancing with the 'B' key. The hexapod will use IMU data to maintain stability:
- Use '[' and ']' to adjust sensitivity
- Balance works in all movement modes

### Sensor Options

The system supports multiple sensor configurations:
- MPU6050: 6-axis IMU with accelerometer and gyroscope
- ADXL345: 3-axis accelerometer
- Switch between them with 'X' and 'C' keys

### Obstacle Avoidance

When ultrasonic sensor is enabled ('U' key):
- Automatically slows down when approaching obstacles
- Stops and backs away from close obstacles

## Troubleshooting

### Common Issues

1. **Servo jittering**
   - Check power supply voltage
   - Verify calibration values
   - Ensure I2C connection is stable

2. **Unresponsive controls**
   - Check if terminal is in the correct mode
   - Verify driver is loaded (`lsmod | grep hexapod`)
   - Restart the application

3. **IMU errors**
   - Verify I2C connections
   - Try switching sensor type (MPU6050/ADXL345)
   - Run diagnostics with 'P' command

### Running Diagnostics

Use the 'M' key to run comprehensive diagnostics on:
- Servo mapping
- Controller connectivity
- Sensor readings

## Next Steps

After getting familiar with basic operation:

1. Explore the reinforcement learning module in `pytd3/`
2. Customize gait patterns in the `gait.cpp` file in `app/src/`
3. Create your own sequences and behaviors
4. See the full documentation for advanced API usage

## Support

For additional help, please:
- Check the complete documentation in the `docs/` directory
- File issues on our GitHub repository
- Join our community Discord server for real-time assistance
