#ifndef DRIVER_MPU6050_H_
#define DRIVER_MPU6050_H_

#include <linux/types.h>

typedef struct xyz_data {
  int16_t x;
  int16_t y;
  int16_t z;
}xyz_data;

typedef struct mpu6050 {
  uint32_t sensitivity;
}mpu6050;

#define READ_ACCELEROMETER _IOR('a', 'a', struct xyz_data)
#define MPU_INFO _IOR('a', 'b', struct mpu6050)

/* MPU6050 Functions */
int mpu6050_init(void);
void mpu6050_cleanup(void);
int mpu6050_read_all(s16 *accel_x, s16 *accel_y, s16 *accel_z,
                     s16 *gyro_x, s16 *gyro_y, s16 *gyro_z);

#endif /* DRIVER_MPU6050_H_ */
