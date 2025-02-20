#ifndef _PCA9685_H_
#define _PCA9685_H_

#include <linux/i2c.h>

/* PCA9685 registers */
#define PCA9685_MODE1       0x00
#define PCA9685_MODE2       0x01
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06

/* PCA9685 mode bits */
#define PCA9685_RESTART     0x80
#define PCA9685_SLEEP       0x10
#define PCA9685_ALLCALL     0x01
#define PCA9685_OUTDRV      0x04
#define PCA9685_AI          0x20

/* PCA9685 configuration */
#define PCA9685_BASE_ADDR   0x40
#define PCA9685_NUM_CHANNELS 16
#define PCA9685_PRESCALE_50HZ 0x79  // For 50Hz PWM frequency

/* Function prototypes */
int pca9685_register(struct i2c_adapter *adapter, u8 addr);
void pca9685_cleanup(void);
int pca9685_set_pwm(struct i2c_client *client, u8 channel, u16 on_time, u16 off_time);

#endif /* _PCA9685_H_ */