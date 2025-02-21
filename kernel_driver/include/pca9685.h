#ifndef _PCA9685_H_
#define _PCA9685_H_

#include <linux/types.h>

/* Device and PWM Configuration */
#define PCA9685_I2C_ADDR    0x40    /* Default I2C address */
#define PWM_RESOLUTION      4096    /* 12-bit resolution */
#define PWM_FREQUENCY       50      /* Default 50Hz for servos */
#define PWM_PERIOD_MS       20      /* Period in ms (1000/50Hz) */

/* Register Addresses */
#define PCA9685_MODE1       0x00
#define PCA9685_MODE2       0x01
#define PCA9685_LED0        0x06    /* Start of LED0 registers */
#define PCA9685_PRESCALE    0xFE    /* Prescaler for PWM output frequency */

/* Mode Register Bits */
#define MODE1_RESTART       0x80
#define MODE1_EXTCLK        0x40
#define MODE1_AI            0x20    /* Auto-Increment enabled */
#define MODE1_SLEEP         0x10
#define MODE2_OUTDRV        0x04    /* Totem pole structure */

/* PWM Timing Limits (in microseconds) */
#define PWM_MIN_TIME        500     /* 0.5ms */
#define PWM_MAX_TIME        2500    /* 2.5ms */
#define PWM_CENTER_TIME     1500    /* 1.5ms */

/* Function Declarations */
int pca9685_init(void);
void pca9685_cleanup(void);
int pca9685_set_pwm(u8 channel, u16 on_time, u16 off_time);
int pca9685_set_pwm_ms(u8 channel, u16 ms);
int pca9685_set_pwm_freq(u16 freq);

#endif /* _PCA9685_H_ */