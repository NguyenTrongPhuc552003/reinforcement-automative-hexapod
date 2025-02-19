#ifndef HEXAPOD_MAIN_H
#define HEXAPOD_MAIN_H

#include <linux/types.h>
#include "hexapod_ioctl.h"

// Function declarations for component initialization
int servo_init_all(void);
void servo_cleanup(void);

// These functions use default configuration
int uart_init(void);
void uart_cleanup(void);

int i2c_init(void);
void i2c_cleanup(void);

int mpu6050_init(void);
void mpu6050_cleanup(void);

#endif /* HEXAPOD_MAIN_H */
