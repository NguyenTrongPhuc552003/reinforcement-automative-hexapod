#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <linux/i2c.h>
#include <linux/types.h>
#include "protocol.h"

/* Device identification */
#define MPU6050_I2C_ADDR 0x68
#define MPU6050_DEVICE_ID 0x68
#define MPU6050_DEVICE_ID_ALT 0x72 /* Some MPU6050 variants return this ID */
#define MPU6050_WHO_AM_I 0x75

/* Register addresses */
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43

/* Configuration values */
#define MPU6050_RESET 0x80
#define MPU6050_SLEEP_MODE 0x40
#define MPU6050_CLOCK_PLL 0x01
#define MPU6050_GYRO_FS_500 0x08 /* ±500 degrees/second */
#define MPU6050_ACCEL_FS_2G 0x00 /* ±2g */
#define MPU6050_DLPF_10HZ 0x05   /* 10Hz low-pass filter */

/* Scaling factors as fixed point for kernel (16.16 format) */
#define MPU6050_ACCEL_SCALE_2G_FIXP 1073741824 /* 16384.0f << 16 */
#define MPU6050_GYRO_SCALE_500_FIXP 4292870    /* 65.5f << 16 */

/* Function declarations */
int mpu6050_init(struct i2c_client *client);
void mpu6050_remove(struct i2c_client *client);
int mpu6050_read_sensors(struct i2c_client *client, struct imu_data *data);

#endif /* _MPU6050_H_ */
