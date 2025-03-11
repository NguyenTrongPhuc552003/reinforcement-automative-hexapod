# Hexapod Hardware Documentation

## Overview

The hexapod robot uses the following key hardware components:
- BeagleBone Black as the control board
- MPU6050 6-axis IMU sensor
- PCA9685 PWM controller(s) for servo control
- MG996R servo motors (18 units, 3 per leg)

## I2C Bus Configuration

The I2C bus 3 on BeagleBone Black is used for connecting sensors and actuators:

| Device            | I2C Address | Bus | Description                          |
|-------------------|-------------|-----|--------------------------------------|
| MPU6050           | 0x68        | 3   | 6-axis IMU sensor                    |
| PCA9685 Primary   | 0x40        | 3   | 16-channel PWM controller            |
| PCA9685 Secondary | 0x70        | 3   | 16-channel PWM controller (optional) |

## MPU6050 IMU Sensor

The MPU6050 is a 6-axis motion tracking device combining a 3-axis gyroscope and 3-axis accelerometer.

### Configuration
- Accelerometer Range: ±2g
- Gyroscope Range: ±500 degrees/second
- Sample Rate: 100Hz
- Digital Low-Pass Filter: 10Hz

### Pin Connections
- VCC: 3.3V
- GND: Ground
- SCL: P9_19 (I2C2_SCL)
- SDA: P9_20 (I2C2_SDA)
- INT: Not connected

### Data Interpretation
- Accelerometer data is in raw 16-bit format (divide by 16384 to get values in g's)
- Gyroscope data is in raw 16-bit format (divide by 65.5 to get values in degrees/second)

## PCA9685 PWM Controller

The PCA9685 is a 16-channel, 12-bit PWM controller.

### Configuration
- PWM Frequency: 50Hz (standard for servos)
- I2C Address: 0x40 (Primary), 0x70 (Secondary, optional)
- Output Mode: Push-pull
- Internal oscillator: 25MHz

### Pin Connections
- VCC: 3.3V
- GND: Ground
- SCL: P9_19 (I2C2_SCL)
- SDA: P9_20 (I2C2_SDA)
- OE: Not connected (enabled by default)

## Servo Motors

MG996R metal gear servos are used for all joints.

### Specifications
- Operating Voltage: 4.8-6.6V
- Torque: 9.4 kg-cm (4.8V) to 11 kg-cm (6.6V)
- Speed: 0.17 sec/60° (4.8V) to 0.14 sec/60° (6.6V)
- Range: 180° (controlled as ±90°)
- PWM Signal: 1-2ms pulse width (50Hz frequency)

### Joint Assignment
- Hip Joints: Controls horizontal movement (±90°)
- Knee Joints: Controls vertical movement (±90°)
- Ankle Joints: Controls final foot position (±90°)

## Servo Mapping

Below is the mapping of leg numbers, joints, and PWM channels:

| Leg | Joint   | PWM Channel | Controller |
|-----|---------|-------------|------------|
| 0   | Hip     | 0           | Primary    |
| 0   | Knee    | 1           | Primary    |
| 0   | Ankle   | 2           | Primary    |
| 1   | Hip     | 3           | Primary    |
| 1   | Knee    | 4           | Primary    |
| 1   | Ankle   | 5           | Primary    |
| 2   | Hip     | 6           | Primary    |
| 2   | Knee    | 7           | Primary    |
| 2   | Ankle   | 8           | Primary    |
| 3   | Hip     | 9           | Primary    |
| 3   | Knee    | 10          | Primary    |
| 3   | Ankle   | 11          | Primary    |
| 4   | Hip     | 12          | Primary    |
| 4   | Knee    | 13          | Primary    |
| 4   | Ankle   | 14          | Primary    |
| 5   | Hip     | 15          | Primary    |
| 5   | Knee    | 0           | Secondary  |
| 5   | Ankle   | 1           | Secondary  |

## Power Requirements

- Logic Power: 5V from BeagleBone or USB
- Servo Power: 6V, 10A power supply recommended
- Separate power rails for logic and servos with common ground
