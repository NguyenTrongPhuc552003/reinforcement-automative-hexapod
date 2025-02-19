#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "mpu6050.h"

static struct i2c_client *mpu6050_client;

// Read a single byte from MPU6050
static s32 mpu6050_read_byte(u8 reg)
{
    return i2c_smbus_read_byte_data(mpu6050_client, reg);
}

// Write a single byte to MPU6050
static s32 mpu6050_write_byte(u8 reg, u8 data)
{
    return i2c_smbus_write_byte_data(mpu6050_client, reg, data);
}

// Read multiple bytes from MPU6050
static s32 mpu6050_read_block(u8 reg, u8 *buf, u8 len)
{
    return i2c_smbus_read_i2c_block_data(mpu6050_client, reg, len, buf);
}

int mpu6050_init(void)
{
    s32 ret;
    struct i2c_adapter *adapter;
    struct i2c_board_info board_info = {
        I2C_BOARD_INFO("mpu6050", MPU6050_I2C_ADDR)
    };

    // Get I2C adapter
    adapter = i2c_get_adapter(0); // BeagleBone Black I2C0
    if (!adapter) {
        pr_err("MPU6050: Failed to get I2C adapter\n");
        return -ENODEV;
    }

    // Create I2C client
    mpu6050_client = i2c_new_device(adapter, &board_info);
    if (!mpu6050_client) {
        pr_err("MPU6050: Failed to create I2C client\n");
        i2c_put_adapter(adapter);
        return -ENODEV;
    }

    // Check device ID
    ret = mpu6050_read_byte(MPU6050_WHO_AM_I);
    if (ret < 0 || ret != MPU6050_DEVICE_ID) {
        pr_err("MPU6050: Device ID mismatch (expected: 0x%02x, got: 0x%02x)\n",
               MPU6050_DEVICE_ID, ret);
        goto error;
    }

    // Reset device
    ret = mpu6050_write_byte(MPU6050_PWR_MGMT_1, MPU6050_RESET);
    if (ret < 0) {
        pr_err("MPU6050: Reset failed\n");
        goto error;
    }
    msleep(100); // Wait for reset to complete

    // Wake up device and set clock source
    ret = mpu6050_write_byte(MPU6050_PWR_MGMT_1, MPU6050_PLL_XGYRO);
    if (ret < 0) {
        pr_err("MPU6050: Wake up failed\n");
        goto error;
    }

    // Configure device
    ret = mpu6050_write_byte(MPU6050_CONFIG, 0x03);          // DLPF at 44Hz
    if (ret < 0) goto error;

    ret = mpu6050_write_byte(MPU6050_GYRO_CONFIG, 
                            MPU6050_GYRO_FS_500);    // ±500 deg/s
    if (ret < 0) goto error;

    ret = mpu6050_write_byte(MPU6050_ACCEL_CONFIG, 
                            MPU6050_ACCEL_FS_4);     // ±4g
    if (ret < 0) goto error;

    pr_info("MPU6050: Initialization complete\n");
    return 0;

error:
    i2c_unregister_device(mpu6050_client);
    i2c_put_adapter(adapter);
    return ret;
}

int mpu6050_read_all(s16 *accel_x, s16 *accel_y, s16 *accel_z,
                     s16 *gyro_x, s16 *gyro_y, s16 *gyro_z,
                     s16 *temp)
{
    u8 data[14];
    s32 ret;

    ret = mpu6050_read_block(MPU6050_ACCEL_XOUT_H, data, 14);
    if (ret < 0) {
        pr_err("MPU6050: Failed to read sensor data\n");
        return ret;
    }

    // Convert raw data to signed 16-bit values
    *accel_x = (s16)((data[0] << 8) | data[1]);
    *accel_y = (s16)((data[2] << 8) | data[3]);
    *accel_z = (s16)((data[4] << 8) | data[5]);
    *temp    = (s16)((data[6] << 8) | data[7]);
    *gyro_x  = (s16)((data[8] << 8) | data[9]);
    *gyro_y  = (s16)((data[10] << 8) | data[11]);
    *gyro_z  = (s16)((data[12] << 8) | data[13]);

    return 0;
}

void mpu6050_cleanup(void)
{
    if (mpu6050_client) {
        // Put device in sleep mode
        mpu6050_write_byte(MPU6050_PWR_MGMT_1, MPU6050_SLEEP);
        i2c_unregister_device(mpu6050_client);
        mpu6050_client = NULL;
        pr_info("MPU6050: Cleanup complete\n");
    }
}

MODULE_LICENSE("GPL");
