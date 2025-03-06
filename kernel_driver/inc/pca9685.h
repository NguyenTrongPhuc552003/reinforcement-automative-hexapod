#ifndef _PCA9685_H_
#define _PCA9685_H_

#include <linux/types.h>
#include <linux/i2c.h>
#include "hexapod.h"

/* I2C Bus and Addresses */
#define PCA9685_I2C_BUS 3     /* BeagleBone I2C3 */
#define PCA9685_I2C_ADDR 0x40 /* Default address */

/* PCA9685 registers */
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_ALL_LED_ON_L 0xFA
#define PCA9685_ALL_LED_ON_H 0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD
#define PCA9685_PRESCALE 0xFE

/* Mode register bits */
#define MODE1_RESTART 0x80
#define MODE1_EXTCLK 0x40
#define MODE1_AI 0x20 /* Auto-increment */
#define MODE1_SLEEP 0x10
#define MODE1_ALLCALL 0x01

#define MODE2_INVRT 0x10
#define MODE2_OCH 0x08
#define MODE2_OUTDRV 0x04
#define MODE2_OUTNE1 0x02
#define MODE2_OUTNE0 0x01

/* Clock and PWM parameters */
#define PCA9685_CLOCK_FREQ 25000000UL /* 25 MHz */
#define PCA9685_PWM_FREQ 50           /* 50 Hz default */
#define PCA9685_PWM_RES 4096          /* 12-bit resolution */

/* Servo pulse width limits (microseconds) */
#define PWM_MIN_US 1000 /* 1ms = -90 degrees */
#define PWM_MID_US 1500 /* 1.5ms = 0 degrees */
#define PWM_MAX_US 2000 /* 2ms = +90 degrees */

/* Function prototypes */
int pca9685_init(void);
void pca9685_cleanup(void);
int pca9685_set_pwm_freq(u16 freq_hz);
int pca9685_set_pwm(u8 channel, u16 on, u16 off);
int pca9685_set_pwm_us(u8 channel, u16 us);

#endif /* _PCA9685_H_ */
