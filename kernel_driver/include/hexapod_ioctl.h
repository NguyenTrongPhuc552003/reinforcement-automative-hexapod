#ifndef HEXAPOD_IOCTL_H
#define HEXAPOD_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

// IOCTL magic number
#define HEXAPOD_IOC_MAGIC 'H'

// Servo control structure
struct servo_control {
    unsigned int leg_id;     // Leg ID (0-5)
    unsigned int joint_id;   // Joint ID (0-2: COXA, FEMUR, TIBIA)
    unsigned int angle;      // Angle in degrees (0-180)
};

// Movement pattern structure
struct movement_pattern {
    unsigned int pattern_id;  // Pattern ID (0: TRIPOD, 1: WAVE, 2: RIPPLE)
    unsigned int speed;       // Movement speed (1-10)
    int direction;           // Direction (-1: backward, 0: stop, 1: forward)
};

// I2C transfer structure
struct i2c_transfer {
    u8 addr;
    u8 reg;
    u8 data[32];
    size_t length;
    bool is_read;
};

// UART message structure
struct uart_msg {
    char data[256];
    int length;
};

// IOCTL commands
#define IOCTL_SET_SERVO _IOW(HEXAPOD_IOC_MAGIC, 1, struct servo_control)
#define IOCTL_MOVE_LEG _IOW(HEXAPOD_IOC_MAGIC, 2, struct servo_control)
#define IOCTL_SET_PATTERN _IOW(HEXAPOD_IOC_MAGIC, 3, struct movement_pattern)
#define IOCTL_GET_MPU6050 _IOR(HEXAPOD_IOC_MAGIC, 4, struct mpu6050_data)
#define UART_SEND _IOW(HEXAPOD_IOC_MAGIC, 5, struct uart_msg)
#define UART_RECEIVE _IOWR(HEXAPOD_IOC_MAGIC, 6, struct uart_msg)

#endif /* HEXAPOD_IOCTL_H */