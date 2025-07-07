# Hexapod Executables

This directory contains the main applications and test programs for the Hexapod Robot Control System.

## Main Application
- **hexapod_app**: Main hexapod control application with keyboard interface

## Test Programs
- **test_mpu6050**: Test MPU6050 6-axis IMU sensor functionality
- **test_adxl345**: Test ADXL345 3-axis accelerometer  
- **test_servo**: Test individual servo motor control
- **test_movement**: Test basic movement patterns and gaits
- **test_balance**: Test balance control system with IMU feedback
- **test_calibration**: Interactive servo calibration utility
- **test_hcsr04**: Test HC-SR04 ultrasonic distance sensor

## Usage
All programs require root privileges to access hardware:

```bash
# Run main application
sudo hexapod_app

# Test specific hardware
sudo test_mpu6050
sudo test_servo
sudo test_balance

# Interactive calibration
sudo test_calibration
```

## Main Application Controls
- **WASD**: Movement (forward/back/rotate)
- **IJKL**: Height and tilt control  
- **123**: Gait selection (tripod/wave/ripple)
- **+/-**: Speed adjustment
- **Space**: Stop and center
- **Q**: Quit

## Prerequisites
- Kernel module must be loaded (hexapod_driver)
- Hardware must be properly connected
- I2C interface must be enabled

## Troubleshooting
- Check kernel module: `lsmod | grep hexapod_driver`
- Verify device file: `ls -l /dev/hexapod`
- Monitor system logs: `dmesg | tail`
