#ifndef _PCA9685_H
#define _PCA9685_H

#include <linux/types.h>

// PCA9685 I2C Addresses
#define PCA9685_I2C_ADDR1      0x40
#define PCA9685_I2C_ADDR2      0x41

// PCA9685 Register Addresses
#define PCA9685_MODE1          0x00
#define PCA9685_MODE2          0x01
#define PCA9685_SUBADR1        0x02
#define PCA9685_SUBADR2        0x03
#define PCA9685_SUBADR3        0x04
#define PCA9685_PRESCALE       0xFE
#define PCA9685_LED0_ON_L      0x06
#define PCA9685_LED0_ON_H      0x07
#define PCA9685_LED0_OFF_L     0x08
#define PCA9685_LED0_OFF_H     0x09

// PCA9685 Mode Bits
#define PCA9685_RESTART        0x80
#define PCA9685_SLEEP          0x10
#define PCA9685_ALLCALL        0x01
#define PCA9685_INVRT          0x10
#define PCA9685_OUTDRV         0x04

// PCA9685 Configuration
#define MAX_DEVICES            2
#define PWM_CHANNELS          16     // Channels per device
#define PWM_RESOLUTION        4096   // 12-bit resolution
#define INTERNAL_CLOCK_FREQ   25000000  // 25MHz

// Function declarations
int pca9685_device_init(u8 i2c_addr);
int pca9685_set_pwm_freq(u8 i2c_addr, u8 freq);
int pca9685_set_pwm(u8 i2c_addr, u8 channel, u16 on_time, u16 off_time);
void pca9685_cleanup(void);

#endif /* _PCA9685_H */