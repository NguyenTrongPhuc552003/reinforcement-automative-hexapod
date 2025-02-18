#ifndef PCA9685_H
#define PCA9685_H

#include <linux/types.h>

#define MAX_PCA9685_DEVICES 2  // We have 2 PCA9685 chips

// Function declarations
int pca9685_device_init(u8 i2c_addr);  // Renamed from pca9685_init
int pca9685_set_pwm(u8 i2c_addr, u8 channel, u16 on_time, u16 off_time);  // Updated signature
void pca9685_reset(void);

// Set PWM frequency
int pca9685_set_pwm_freq(u8 i2c_addr, unsigned int freq_hz);

// Cleanup all PCA9685 devices
void pca9685_cleanup(void);

#endif /* PCA9685_H */ 