# Hexapod Kernel Driver

This directory contains the kernel driver modules for the hexapod robot. The driver provides interfaces for controlling servos, reading sensor data, and communicating over I2C and UART.

## Directory Structure

```
kernel_driver/
├── include/           # Header files
│   ├── gpio_control.h
│   ├── hexapod_ioctl.h
│   ├── hexapod_main.h
│   ├── i2c_comm.h
│   ├── mpu6050.h
│   ├── pca9685.h
│   ├── pwm_control.h
│   ├── servo.h
│   └── uart_comm.h
├── src/              # Source files
│   ├── gpio_control.c
│   ├── hexapod_main.c
│   ├── i2c_comm.c
│   ├── mpu6050.c
│   ├── pca9685.c
│   ├── pwm_control.c
│   ├── servo.c
│   └── uart_comm.c
└── Makefile
```

## Module Architecture

The driver is split into multiple modules for better maintainability:

- **hexapod_main**: Main character device driver and IOCTL handler
- **i2c_comm**: I2C communication interface
- **uart_comm**: UART communication interface
- **mpu6050**: MPU6050 sensor driver
- **pca9685**: PCA9685 PWM controller driver
- **servo**: Servo control logic
- **gpio_control**: GPIO interface
- **pwm_control**: PWM control interface

Each module is independent but can be used by other modules through exported symbols.

## Building

The driver can be built using the Docker environment:
```bash
./scripts/build.sh
```

## Installation

After building, deploy and install the modules on the BeagleBone:
```bash
./scripts/deploy.sh
```

This will:
1. Copy the built modules to the BeagleBone
2. Load all modules in the correct order
3. Create the device node `/dev/hexapod`

## Usage

The driver creates a character device `/dev/hexapod` that accepts the following IOCTL commands:
- `IOCTL_SET_SERVO`: Set individual servo angle
- `IOCTL_MOVE_LEG`: Move a leg (multiple servos)
- `IOCTL_SET_PATTERN`: Set movement pattern
- `IOCTL_GET_MPU6050`: Read MPU6050 sensor data
- `UART_SEND`: Send data over UART
- `UART_RECEIVE`: Receive data over UART

See `include/hexapod_ioctl.h` for detailed command definitions.
