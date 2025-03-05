#ifndef HEXAPOD_SERVO_H
#define HEXAPOD_SERVO_H

#include <linux/types.h>
#include "protocol.h"

/* Servo parameters */
#define SERVO_MIN_ANGLE -90
#define SERVO_MAX_ANGLE 90
#define SERVO_CHANNELS_PER_CONTROLLER 16
#define SERVO_NUM_CONTROLLERS 2

/* Function declarations */
int servo_init(void);
void servo_cleanup(void);
int servo_set_angle(uint8_t leg, uint8_t joint, int16_t angle);
// int servo_get_angle(uint8_t leg, uint8_t joint, int16_t *angle);
int servo_center_all(void);

#endif /* HEXAPOD_SERVO_H */