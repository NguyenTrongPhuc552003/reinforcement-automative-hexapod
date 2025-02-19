#ifndef _HEXAPOD_IOCTL_H
#define _HEXAPOD_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

// IOCTL magic number
#define HEXAPOD_IOC_MAGIC       'H'

// Movement patterns
typedef enum {
    MOVE_FORWARD,
    MOVE_BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    MOVE_UP,
    MOVE_DOWN,
    MOVE_STOP
} movement_type_t;

// Servo control structure
struct servo_control {
    u8 leg_id;         // Leg identifier (0-5)
    u8 joint_id;       // Joint identifier (0-2)
    s16 angle;         // Target angle (-90 to +90)
} __attribute__((packed));

// MPU6050 data structure
struct mpu6050_data {
    s16 accel_x;       // Accelerometer X-axis
    s16 accel_y;       // Accelerometer Y-axis
    s16 accel_z;       // Accelerometer Z-axis
    s16 temp;          // Temperature
    s16 gyro_x;        // Gyroscope X-axis
    s16 gyro_y;        // Gyroscope Y-axis
    s16 gyro_z;        // Gyroscope Z-axis
} __attribute__((packed));

// Movement control structure
struct movement_control {
    movement_type_t type;   // Movement type
    u16 speed;             // Movement speed (0-1000)
    u16 duration;          // Duration in milliseconds
} __attribute__((packed));

// IOCTL commands
#define HEXAPOD_IOC_SET_SERVO    _IOW(HEXAPOD_IOC_MAGIC, 0, struct servo_control)
#define HEXAPOD_IOC_GET_MPU6050  _IOR(HEXAPOD_IOC_MAGIC, 1, struct mpu6050_data)
#define HEXAPOD_IOC_SET_MOVEMENT _IOW(HEXAPOD_IOC_MAGIC, 2, struct movement_control)

#endif /* _HEXAPOD_IOCTL_H */