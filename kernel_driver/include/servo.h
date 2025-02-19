#ifndef SERVO_H
#define SERVO_H

#include <linux/types.h>

// Servo configuration for hexapod legs
#define NUM_LEGS            6
#define SERVOS_PER_LEG     3
#define MAX_SERVOS         (NUM_LEGS * SERVOS_PER_LEG)

// Leg identifiers
#define LEG_FRONT_RIGHT    0
#define LEG_MIDDLE_RIGHT   1
#define LEG_BACK_RIGHT     2
#define LEG_FRONT_LEFT     3
#define LEG_MIDDLE_LEFT    4
#define LEG_BACK_LEFT      5

// Joint identifiers per leg
#define JOINT_COXA        0  // Hip joint - horizontal movement
#define JOINT_FEMUR       1  // Thigh joint - vertical movement
#define JOINT_TIBIA       2  // Knee joint - vertical movement

// Servo angle limits (in degrees)
#define SERVO_MIN_ANGLE    0
#define SERVO_MAX_ANGLE    180

// Default servo positions (in degrees)
#define COXA_DEFAULT      90   // Center position
#define FEMUR_DEFAULT     90   // Horizontal position
#define TIBIA_DEFAULT     90   // Straight position

struct servo_config {
    u8 channel;         // PCA9685 channel
    u8 min_angle;       // Minimum allowed angle
    u8 max_angle;       // Maximum allowed angle
    u8 default_angle;   // Default position angle
    bool inverted;      // Whether servo direction is inverted
};

struct servo_data {
    struct servo_config config;
    bool is_initialized;
    u8 current_angle;
};

// Function declarations
int servo_init(struct servo_data *servo);
int servo_set_angle(unsigned int leg_id, unsigned int joint_id, unsigned int angle);
void servo_cleanup(void);
int servo_init_all(void);
int servo_reset_all_to_default(void);
int servo_get_current_angle(unsigned int leg_id, unsigned int joint_id);

// Movement helper functions
int servo_move_leg(unsigned int leg_id, u8 coxa_angle, u8 femur_angle, u8 tibia_angle);
int servo_move_leg_relative(unsigned int leg_id, s8 coxa_delta, s8 femur_delta, s8 tibia_delta);

#endif /* SERVO_H */