#ifndef _SERVO_H_
#define _SERVO_H_

#include <linux/types.h>
#include "pca9685.h"

/* Hexapod Configuration */
#define NUM_LEGS            6
#define JOINTS_PER_LEG      3
#define TOTAL_SERVOS        (NUM_LEGS * JOINTS_PER_LEG)

/* Servo Angle Limits */
#define SERVO_MIN_ANGLE    -90
#define SERVO_MAX_ANGLE     90
#define SERVO_CENTER_ANGLE  0

/* Servo Joint Types */
#define JOINT_HIP    0
#define JOINT_KNEE   1
#define JOINT_ANKLE  2

/* Function Declarations */
int servo_init(void);
void servo_cleanup(void);
int servo_set_angle(u8 channel, s16 angle);
int servo_set_pulse_ms(u8 channel, u16 ms);
int servo_center(u8 channel);
int leg_set_position(u8 leg_num, s16 hip_angle, s16 knee_angle, s16 ankle_angle);
int leg_center_all(void);

#endif /* _SERVO_H_ */