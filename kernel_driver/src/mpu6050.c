#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/err.h>
#include "mpu6050.h"

static struct i2c_client *mpu6050_client;

/* Read all sensor data */
int mpu6050_read_all(s16 *accel_x, s16 *accel_y, s16 *accel_z,
                     s16 *gyro_x, s16 *gyro_y, s16 *gyro_z,
                     s16 *temp)
{
    u8 data[14];
    int ret;

    if (!mpu6050_client)
        return -ENODEV;

    ret = i2c_smbus_read_i2c_block_data(mpu6050_client, MPU6050_ACCEL_XOUT_H, 
                                       sizeof(data), data);
    if (ret < 0) {
        dev_err(&mpu6050_client->dev, "Failed to read sensor data\n");
        return ret;
    }

    *accel_x = (s16)((data[0] << 8) | data[1]);
    *accel_y = (s16)((data[2] << 8) | data[3]);
    *accel_z = (s16)((data[4] << 8) | data[5]);
    *temp    = (s16)((data[6] << 8) | data[7]);
    *gyro_x  = (s16)((data[8] << 8) | data[9]);
    *gyro_y  = (s16)((data[10] << 8) | data[11]);
    *gyro_z  = (s16)((data[12] << 8) | data[13]);

    return 0;
}
EXPORT_SYMBOL_GPL(mpu6050_read_all);

/* Initialize MPU6050 hardware */
static int mpu6050_hw_init(struct i2c_client *client)
{
    int ret;

    /* Reset device */
    ret = i2c_smbus_write_byte_data(client, MPU6050_PWR_MGMT_1, MPU6050_RESET);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to reset device\n");
        return ret;
    }
    msleep(100);

    /* Wake up and set clock source */
    ret = i2c_smbus_write_byte_data(client, MPU6050_PWR_MGMT_1, MPU6050_PLL_XGYRO);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to set clock source\n");
        return ret;
    }
    msleep(10);

    /* Configure sensors */
    ret = i2c_smbus_write_byte_data(client, MPU6050_CONFIG, 0x03);  // DLPF at 44Hz
    if (ret < 0) {
        dev_err(&client->dev, "Failed to configure DLPF\n");
        return ret;
    }

    ret = i2c_smbus_write_byte_data(client, MPU6050_GYRO_CONFIG, MPU6050_GYRO_FS_500);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to configure gyroscope\n");
        return ret;
    }

    ret = i2c_smbus_write_byte_data(client, MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_4);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to configure accelerometer\n");
        return ret;
    }

    return 0;
}

/* Initialize MPU6050 driver */
int mpu6050_init(void)
{
    int ret;
    struct i2c_adapter *adapter;
    struct i2c_board_info board_info = {
        I2C_BOARD_INFO("mpu6050", MPU6050_DEVICE_ID)
    };

    /* Get I2C adapter */
    adapter = i2c_get_adapter(3);  // Use I2C bus 3
    if (!adapter) {
        pr_err("Failed to get I2C adapter 3\n");
        return -ENODEV;
    }

    /* Create I2C client */
    mpu6050_client = i2c_new_device(adapter, &board_info);
    i2c_put_adapter(adapter);

    if (!mpu6050_client) {
        pr_err("Failed to create MPU6050 I2C client\n");
        return -ENODEV;
    }

    /* Verify device ID */
    ret = i2c_smbus_read_byte_data(mpu6050_client, MPU6050_WHO_AM_I);
    if (ret < 0) {
        pr_err("Failed to read WHO_AM_I register\n");
        goto err_remove;
    }

    if ((ret & 0xFF) != MPU6050_DEVICE_ID) {
        pr_err("Invalid device ID: %02x\n", ret & 0xFF);
        ret = -ENODEV;
        goto err_remove;
    }

    /* Initialize hardware */
    ret = mpu6050_hw_init(mpu6050_client);
    if (ret < 0) {
        pr_err("Failed to initialize MPU6050 hardware\n");
        goto err_remove;
    }

    pr_info("MPU6050 initialized successfully\n");
    return 0;

err_remove:
    i2c_unregister_device(mpu6050_client);
    mpu6050_client = NULL;
    return ret;
}
EXPORT_SYMBOL_GPL(mpu6050_init);

/* Cleanup MPU6050 driver */
void mpu6050_cleanup(void)
{
    if (mpu6050_client) {
        i2c_unregister_device(mpu6050_client);
        mpu6050_client = NULL;
    }
    pr_info("MPU6050 cleaned up\n");
}
EXPORT_SYMBOL_GPL(mpu6050_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phuc Nguyen");
MODULE_DESCRIPTION("MPU6050 I2C driver");
MODULE_VERSION("1.0");
