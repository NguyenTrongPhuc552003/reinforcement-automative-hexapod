#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <linux/i2c.h>

/* MPU6050 Register Map */
#define MPU6050_WHO_AM_I          0x75
#define MPU6050_DEVICE_ID         0x68

#define MPU6050_PWR_MGMT_1        0x6B
#define MPU6050_PWR_MGMT_2        0x6C
#define MPU6050_CONFIG            0x1A
#define MPU6050_GYRO_CONFIG       0x1B
#define MPU6050_ACCEL_CONFIG      0x1C

#define MPU6050_ACCEL_XOUT_H      0x3B
#define MPU6050_ACCEL_XOUT_L      0x3C
#define MPU6050_ACCEL_YOUT_H      0x3D
#define MPU6050_ACCEL_YOUT_L      0x3E
#define MPU6050_ACCEL_ZOUT_H      0x3F
#define MPU6050_ACCEL_ZOUT_L      0x40

#define MPU6050_TEMP_OUT_H        0x41
#define MPU6050_TEMP_OUT_L        0x42

#define MPU6050_GYRO_XOUT_H       0x43
#define MPU6050_GYRO_XOUT_L       0x44
#define MPU6050_GYRO_YOUT_H       0x45
#define MPU6050_GYRO_YOUT_L       0x46
#define MPU6050_GYRO_ZOUT_H       0x47
#define MPU6050_GYRO_ZOUT_L       0x48

/* MPU6050 Configuration Values */
#define MPU6050_RESET             0x80
#define MPU6050_SLEEP             0x40
#define MPU6050_PLL_XGYRO        0x01

#define MPU6050_GYRO_FS_250      0x00
#define MPU6050_GYRO_FS_500      0x08
#define MPU6050_GYRO_FS_1000     0x10
#define MPU6050_GYRO_FS_2000     0x18

#define MPU6050_ACCEL_FS_2       0x00
#define MPU6050_ACCEL_FS_4       0x08
#define MPU6050_ACCEL_FS_8       0x10
#define MPU6050_ACCEL_FS_16      0x18

/* Function Prototypes */
int mpu6050_init(void);
void mpu6050_cleanup(void);
int mpu6050_read_all(s16 *accel_x, s16 *accel_y, s16 *accel_z,
                     s16 *gyro_x, s16 *gyro_y, s16 *gyro_z,
                     s16 *temp);

#endif /* _MPU6050_H_ */
