#ifndef DRIVER_MPU6050_H_
#define DRIVER_MPU6050_H_

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

#endif /* DRIVER_MPU6050_H_ */
