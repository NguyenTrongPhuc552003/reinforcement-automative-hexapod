#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include <linux/types.h>

// Function declarations
int pca9685_init(u8 i2c_addr);
int pwm_init_servo(u8 i2c_addr, u8 channel);
int pwm_set_servo_angle(u8 servo_id, u8 angle);
void pwm_cleanup(void);

#endif /* PWM_CONTROL_H */
