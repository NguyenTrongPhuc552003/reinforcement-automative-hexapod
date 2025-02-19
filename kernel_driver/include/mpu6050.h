#ifndef MPU6050_H
#define MPU6050_H

#include <linux/types.h>
#include "i2c_comm.h"

// MPU6050 I2C address
#define MPU6050_I2C_ADDR             0x68

// MPU6050 registers
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_FIFO_EN         0x23
#define MPU6050_REG_INT_ENABLE      0x38
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_ACCEL_XOUT_L    0x3C
#define MPU6050_REG_ACCEL_YOUT_H    0x3D
#define MPU6050_REG_ACCEL_YOUT_L    0x3E
#define MPU6050_REG_ACCEL_ZOUT_H    0x3F
#define MPU6050_REG_ACCEL_ZOUT_L    0x40
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_TEMP_OUT_L      0x42
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_GYRO_XOUT_L     0x44
#define MPU6050_REG_GYRO_YOUT_H     0x45
#define MPU6050_REG_GYRO_YOUT_L     0x46
#define MPU6050_REG_GYRO_ZOUT_H     0x47
#define MPU6050_REG_GYRO_ZOUT_L     0x48
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_PWR_MGMT_2      0x6C
#define MPU6050_REG_WHO_AM_I        0x75

// Configuration values
#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x08
#define MPU6050_GYRO_FS_1000        0x10
#define MPU6050_GYRO_FS_2000        0x18

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x08
#define MPU6050_ACCEL_FS_8          0x10
#define MPU6050_ACCEL_FS_16         0x18

// Structure to hold sensor data
struct mpu6050_data {
    s16 accel_x;
    s16 accel_y;
    s16 accel_z;
    s16 gyro_x;
    s16 gyro_y;
    s16 gyro_z;
    s16 temp;
};

// Function declarations
int mpu6050_init(void);
void mpu6050_cleanup(void);
int mpu6050_read_reg(u8 reg, u8 *value);
int mpu6050_write_reg(u8 reg, u8 value);
int mpu6050_read_all(s16 *accel_x, s16 *accel_y, s16 *accel_z,
                     s16 *gyro_x, s16 *gyro_y, s16 *gyro_z,
                     s16 *temp);

#endif /* MPU6050_H */
