#ifndef _TEST_IOCTL_H
#define _TEST_IOCTL_H

#include <linux/ioctl.h>

// Define the IOCTL magic number
#define HEXAPOD_IOC_MAGIC 'H'

// Hexapod configuration
#define NUM_LEGS                6
#define NUM_JOINTS_PER_LEG      3
#define TOTAL_SERVOS            (NUM_LEGS * NUM_JOINTS_PER_LEG)

// Servo control structure
struct servo_command {
    unsigned char leg_id;
    unsigned char joint_id;
    short angle;
};

// Leg movement structure
struct leg_movement {
    unsigned char leg_id;
    short coxa_angle;
    short femur_angle;
    short tibia_angle;
};

// IOCTL commands
#define IOCTL_SET_SERVO_ANGLE _IOW(HEXAPOD_IOC_MAGIC, 1, struct servo_command)
#define IOCTL_MOVE_LEG        _IOW(HEXAPOD_IOC_MAGIC, 2, struct leg_movement)
#define IOCTL_RESET_ALL       _IO(HEXAPOD_IOC_MAGIC, 3)

// Leg identifiers
#define LEG_RIGHT_FRONT     0
#define LEG_RIGHT_MIDDLE    1
#define LEG_RIGHT_BACK      2
#define LEG_LEFT_FRONT      3
#define LEG_LEFT_MIDDLE     4
#define LEG_LEFT_BACK       5

// Joint identifiers
#define JOINT_COXA         0
#define JOINT_FEMUR        1
#define JOINT_TIBIA        2

// Angle limits
#define ANGLE_MIN         -90
#define ANGLE_MAX          90

#endif /* _TEST_IOCTL_H */