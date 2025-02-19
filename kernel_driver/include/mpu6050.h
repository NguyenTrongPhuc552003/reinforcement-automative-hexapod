#ifndef _MPU6050_H
#define _MPU6050_H

#include <linux/types.h>
#include "hexapod_ioctl.h"  // For mpu6050_data structure

// MPU6050 I2C Address
#define MPU6050_I2C_ADDR        0x68
#define MPU6050_DEVICE_ID       0x68

// MPU6050 Register Addresses
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B

// MPU6050 Configuration Bits
#define MPU6050_RESET           0x80
#define MPU6050_SLEEP           0x40
#define MPU6050_PLL_XGYRO       0x01

// MPU6050 Gyro Configuration
#define MPU6050_GYRO_FS_250     0x00
#define MPU6050_GYRO_FS_500     0x08
#define MPU6050_GYRO_FS_1000    0x10
#define MPU6050_GYRO_FS_2000    0x18

// MPU6050 Accelerometer Configuration
#define MPU6050_ACCEL_FS_2      0x00
#define MPU6050_ACCEL_FS_4      0x08
#define MPU6050_ACCEL_FS_8      0x10
#define MPU6050_ACCEL_FS_16     0x18

// Function declarations
int mpu6050_init(void);
int mpu6050_read_all(s16 *accel_x, s16 *accel_y, s16 *accel_z,
                     s16 *gyro_x, s16 *gyro_y, s16 *gyro_z,
                     s16 *temp);
void mpu6050_cleanup(void);

#endif /* _MPU6050_H */
