#ifndef HEXAPOD_PCA9685_H
#define HEXAPOD_PCA9685_H

#include <linux/types.h>

/* Device parameters */
#define PCA9685_I2C_ADDR 0x40
#define PCA9685_I2C_BUS 3

/* Register addresses */
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_LED0 PCA9685_LED0_ON_L
#define PCA9685_PRESCALE 0xFE

/* LED offsets */
#define LED_ON_L 0x00
#define LED_ON_H 0x01
#define LED_OFF_L 0x02
#define LED_OFF_H 0x03

/* Mode register bits */
#define MODE1_RESTART 0x80
#define MODE1_EXTCLK 0x40
#define MODE1_AI 0x20
#define MODE1_SLEEP 0x10
#define MODE1_ALLCALL 0x01
#define PCA9685_OUTDRV 0x04

/* PWM configuration */
#define OSC_CLOCK 25000000     /* 25MHz internal oscillator */
#define PWM_RESOLUTION 4096    /* 12-bit resolution */
#define PCA9685_FREQ 50        /* 50Hz for servos */
#define PCA9685_MIN_PULSE 500  /* 0.5ms */
#define PCA9685_MAX_PULSE 2500 /* 2.5ms */
#define PCA9685_MID_PULSE 1500 /* 1.5ms (neutral) */
#define PWM_CENTER_TIME 1500   /* Center position in microseconds */

/* Function declarations */
int pca9685_init(void);
void pca9685_cleanup(void);
int pca9685_set_pwm_freq(u16 freq);
int pca9685_set_pwm(u8 channel, u16 on_time, u16 off_time);
int pca9685_set_pwm_ms(u8 channel, u16 ms);

#endif /* HEXAPOD_PCA9685_H */