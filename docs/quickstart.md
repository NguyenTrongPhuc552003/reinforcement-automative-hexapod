# Hexapod Quick-Start Guide

This guide helps you get your hexapod robot up and running quickly.

## Prerequisites

- BeagleBone AI/Black with Debian installed
- I2C enabled on bus 2
- 18 servos connected to PCA9685 controllers
- Power supply connected

## Initial Setup

1. **Clone the repository**
   ```bash
   git clone https://github.com/NguyenTrongPhuc552003/reinforcement-automative-hexapod.git
   cd reinforcement-automative-hexapod
   ```

2. **Build all components**
   ```bash
   ./scripts/build.sh
   ```
   This builds both kernel and user-space components and prepares the deployment package.

3. **Install the drivers**
   ```bash
   cd deploy
   sudo ./install.sh
   ```
   
   Verify the driver is loaded:
   ```bash
   lsmod | grep hexapod_driver
   ```

4. **Initialize calibration**
   ```bash
   sudo ./test_calibration reset
   ```
   This creates a default calibration file with all offsets set to zero.

## First Movement Test

1. **Center all servos**
   ```bash
   sudo ./test_servo
   ```
   This will center all servos one by one.

2. **Test individual leg movement**
   ```bash
   sudo ./test_movement leg
   ```
   Each leg will move through its range of motion.

3. **Try walking with tripod gait**
   ```bash
   sudo ./test_movement tripod
   ```
   The robot will walk forward for 15 seconds using the tripod gait pattern.

## Testing Components

### Test IMU Sensor
```bash
sudo ./test_mpu6050
```

This will continuously read and display data from the MPU6050 sensor. The sensor will automatically wake from sleep mode as needed, so you can run this test immediately after other tests without needing to reload the driver.

### Test Servo Motion
```bash
sudo ./test_servo
```

## Monitoring and Debugging

The hexapod comes with a comprehensive monitoring script for debugging:

```bash
# Basic system monitoring
sudo ./monitor.sh

# Focus on IMU sensor monitoring
sudo ./monitor.sh --mode imu

# Monitor IOCTL calls in real-time
sudo ./monitor.sh --mode live

# Check for a specific issue with a specific process
sudo ./monitor.sh --mode ioctl --pid $(pgrep test_mpu6050)
```

The monitor script can help you diagnose:
- Driver loading issues
- MPU6050 sleep/wake cycle problems
- Communication errors
- Resource usage

## Next Steps

- Customize the gait parameters in `app/src/gait.cpp`
- Create a custom control program based on `app/src/main.cpp`
- Fine-tune the servo calibration with `test_calibration apply`
