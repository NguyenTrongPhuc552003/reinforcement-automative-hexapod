#include <linux/module.h>
#include <linux/delay.h>
#include "../include/mpu6050.h"
#include "../include/i2c_comm.h"

// Regular function implementations for register access
int mpu6050_write_reg(u8 reg, u8 value)
{
    return i2c_write_data(MPU6050_I2C_ADDR, reg, &value, 1);
}

int mpu6050_read_reg(u8 reg, u8 *value)
{
    return i2c_read_data(MPU6050_I2C_ADDR, reg, value, 1);
}

int mpu6050_read_all(s16 *accel_x, s16 *accel_y, s16 *accel_z,
                     s16 *gyro_x, s16 *gyro_y, s16 *gyro_z,
                     s16 *temp)
{
    u8 data[14];
    int ret;

    // Read all sensor data in one transaction
    ret = i2c_read_data(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H, data, 14);
    if (ret < 0)
        return ret;

    // Convert the data
    *accel_x = (s16)((data[0] << 8) | data[1]);
    *accel_y = (s16)((data[2] << 8) | data[3]);
    *accel_z = (s16)((data[4] << 8) | data[5]);
    *temp = (s16)((data[6] << 8) | data[7]);
    *gyro_x = (s16)((data[8] << 8) | data[9]);
    *gyro_y = (s16)((data[10] << 8) | data[11]);
    *gyro_z = (s16)((data[12] << 8) | data[13]);

    return 0;
}

int mpu6050_init(void)
{
    u8 who_am_i;
    int ret;

    // Check device ID
    ret = mpu6050_read_reg(MPU6050_REG_WHO_AM_I, &who_am_i);
    if (ret < 0) {
        pr_err("Failed to read WHO_AM_I register\n");
        return ret;
    }

    if (who_am_i != 0x68) {
        pr_err("Unexpected WHO_AM_I value: 0x%02X\n", who_am_i);
        return -ENODEV;
    }

    // Wake up the device
    ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret < 0)
        return ret;

    // Set sample rate to 1kHz
    ret = mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, 0x07);
    if (ret < 0)
        return ret;

    // Set gyro full-scale range to ±250°/s
    ret = mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_250);
    if (ret < 0)
        return ret;

    // Set accelerometer full-scale range to ±2g
    ret = mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_2);
    if (ret < 0)
        return ret;

    // Disable FIFO and I2C master mode
    ret = mpu6050_write_reg(MPU6050_REG_CONFIG, 0x00);
    if (ret < 0)
        return ret;

    pr_info("MPU6050 initialized successfully\n");
    return 0;
}

void mpu6050_exit(void)
{
    // Put the device in sleep mode
    mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x40);
}

int mpu6050_read_sensors(struct mpu6050_data *data)
{
    int ret;

    ret = mpu6050_read_all(&data->accel_x, &data->accel_y, &data->accel_z,
                           &data->gyro_x, &data->gyro_y, &data->gyro_z,
                           &data->temp);
    if (ret < 0)
        return ret;

    return 0;
}

int mpu6050_set_gyro_range(u8 range)
{
    if (range != MPU6050_GYRO_FS_250 && range != MPU6050_GYRO_FS_500 &&
        range != MPU6050_GYRO_FS_1000 && range != MPU6050_GYRO_FS_2000)
        return -EINVAL;

    return mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, range);
}

int mpu6050_set_accel_range(u8 range)
{
    if (range != MPU6050_ACCEL_FS_2 && range != MPU6050_ACCEL_FS_4 &&
        range != MPU6050_ACCEL_FS_8 && range != MPU6050_ACCEL_FS_16)
        return -EINVAL;

    return mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, range);
}

int mpu6050_self_test(void)
{
    struct mpu6050_data data;
    int ret;

    // Read initial values
    ret = mpu6050_read_sensors(&data);
    if (ret < 0)
        return ret;

    // Verify readings are within reasonable ranges
    if (abs(data.accel_x) > 32767 || abs(data.accel_y) > 32767 || 
        abs(data.accel_z) > 32767 || abs(data.gyro_x) > 32767 || 
        abs(data.gyro_y) > 32767 || abs(data.gyro_z) > 32767) {
        return -ERANGE;
    }

    return 0;
}
