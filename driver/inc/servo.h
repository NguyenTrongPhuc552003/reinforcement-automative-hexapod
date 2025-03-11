#ifndef _SERVO_H_
#define _SERVO_H_

#include <linux/types.h>
#include "hexapod.h"

/* Servo angle limits */
#define SERVO_MIN_ANGLE -90
#define SERVO_MAX_ANGLE 90

/* Function declarations */
int servo_init(void);
void servo_cleanup(void);
int servo_set_angle(u8 leg, u8 joint, s16 angle);
int servo_set_calibration(u8 leg, s16 hip_offset, s16 knee_offset, s16 ankle_offset);
int servo_center_all(void);

#endif /* _SERVO_H_ */
