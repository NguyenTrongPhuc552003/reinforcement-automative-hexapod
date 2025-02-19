#ifndef HEXAPOD_IOCTL_USER_H
#define HEXAPOD_IOCTL_USER_H

#include <stdint.h>
#include <stdbool.h>
#include <sys/ioctl.h>

// Define the magic number for IOCTL commands
#define HEXAPOD_IOC_MAGIC 'H'

// I2C transfer structure
struct i2c_transfer {
    uint8_t addr;
    uint8_t reg;
    uint8_t data[32];
    size_t len;
    bool is_read;
};

// Servo control structure
struct servo_control {
    uint8_t leg_id;     // Leg number (0-5)
    uint8_t joint_id;   // Joint number (0-2)
    int16_t angle;      // Angle in degrees
};

// Movement pattern structure
struct movement_pattern {
    uint8_t pattern_id;  // Pattern type (TRIPOD, WAVE, etc.)
    uint8_t speed;       // Movement speed (1-10)
    int8_t direction;    // Forward/backward/left/right
};

// MPU6050 data structure
struct mpu6050_data {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
};

// UART message structure
struct uart_message {
    uint8_t data[32];
    size_t len;
};

// Movement patterns
#define PATTERN_TRIPOD      0
#define PATTERN_WAVE        1
#define PATTERN_RIPPLE      2

// Movement directions
#define DIRECTION_FORWARD   1
#define DIRECTION_BACKWARD -1
#define DIRECTION_LEFT      2
#define DIRECTION_RIGHT    -2
#define DIRECTION_STOP      0

// IOCTL commands
#define IOCTL_I2C_TRANSFER   _IOWR(HEXAPOD_IOC_MAGIC, 1, struct i2c_transfer)
#define IOCTL_SET_SERVO      _IOW(HEXAPOD_IOC_MAGIC, 2, struct servo_control)
#define IOCTL_UART_WRITE     _IOW(HEXAPOD_IOC_MAGIC, 3, struct uart_message)
#define IOCTL_GET_MPU6050    _IOR(HEXAPOD_IOC_MAGIC, 4, struct mpu6050_data)
#define IOCTL_UART_READ      _IOR(HEXAPOD_IOC_MAGIC, 5, struct uart_message)
#define IOCTL_MOVE_LEG       _IOW(HEXAPOD_IOC_MAGIC, 6, struct servo_control)
#define IOCTL_SET_PATTERN    _IOW(HEXAPOD_IOC_MAGIC, 7, struct movement_pattern)

#endif /* HEXAPOD_IOCTL_USER_H */
