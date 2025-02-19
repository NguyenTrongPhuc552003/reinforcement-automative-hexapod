#ifndef _SERVO_H
#define _SERVO_H

#include <linux/types.h>

// Hexapod Configuration
#define NUM_LEGS             6
#define NUM_JOINTS_PER_LEG   3
#define TOTAL_SERVOS         (NUM_LEGS * NUM_JOINTS_PER_LEG)

// Leg IDs
#define LEG_RIGHT_FRONT     0
#define LEG_RIGHT_MIDDLE    1
#define LEG_RIGHT_BACK      2
#define LEG_LEFT_FRONT      3
#define LEG_LEFT_MIDDLE     4
#define LEG_LEFT_BACK       5

// Joint IDs
#define JOINT_COXA          0    // Hip joint
#define JOINT_FEMUR         1    // Thigh joint
#define JOINT_TIBIA         2    // Knee joint

// Servo Angle Limits
#define ANGLE_MIN           -90  // -90 degrees
#define ANGLE_MAX           90   // +90 degrees

// PWM Configuration
#define PWM_FREQ           50    // 50Hz
#define PWM_PERIOD         20000 // 20ms in microseconds
#define PWM_MIN_DUTY       1000  // 1ms in microseconds
#define PWM_MAX_DUTY       2000  // 2ms in microseconds
#define PWM_RESOLUTION     4096  // 12-bit resolution

// Function declarations
int servo_init_all(void);
int servo_set_angle(u8 leg_id, u8 joint_id, s16 angle);
int servo_move_leg(u8 leg_id, s16 coxa_angle, s16 femur_angle, s16 tibia_angle);
void servo_cleanup(void);

#endif /* _SERVO_H */