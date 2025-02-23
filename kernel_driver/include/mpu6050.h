#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <linux/types.h>

/* Function declarations */
int mpu6050_init(void);
void mpu6050_cleanup(void);
int mpu6050_read_all(s16 *accel_x, s16 *accel_y, s16 *accel_z,
                     s16 *gyro_x, s16 *gyro_y, s16 *gyro_z);

#endif /* _MPU6050_H_ */
