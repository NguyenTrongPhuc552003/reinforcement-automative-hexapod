#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <linux/i2c.h>
#include <linux/types.h>
#include "hexapod.h"

/* Register addresses */
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_WHO_AM_I 0x75

/* Configuration values */
#define MPU6050_RESET 0x80
#define MPU6050_SLEEP_MODE 0x40
#define MPU6050_CLOCK_PLL 0x01
#define MPU6050_GYRO_FS_500 0x01 /* ±500 degrees/second */
#define MPU6050_ACCEL_FS_2G 0x00 /* ±2g */
#define MPU6050_DLPF_10HZ 0x05   /* 10Hz low-pass filter */

/* Device identification */
#define MPU6050_DEVICE_ID 0x98

/* Function declarations */
int mpu6050_init(struct i2c_client *client);
void mpu6050_remove(struct i2c_client *client);
int mpu6050_read_sensors(struct i2c_client *client, struct hexapod_imu_data *data);

#endif /* _MPU6050_H_ */
