#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include "mpu6050.h"

/* Global MPU6050 device structure */
static struct mpu6050_dev mpu6050_device;

/**
 * Write a register to the MPU6050
 */
static int mpu6050_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
    int ret;

    ret = i2c_smbus_write_byte_data(client, reg, value);
    if (ret < 0)
        dev_err(&client->dev, "Failed to write to reg 0x%02x: %d\n", reg, ret);

    return ret;
}

/**
 * Read a register from the MPU6050
 */
static int mpu6050_read_reg(struct i2c_client *client, u8 reg)
{
    int ret;

    ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0)
        dev_err(&client->dev, "Failed to read reg 0x%02x: %d\n", reg, ret);

    return ret;
}

/**
 * Read a block of data from MPU6050
 */
static int mpu6050_read_block(struct i2c_client *client, u8 reg, u8 len, u8 *buf)
{
    int ret;

    ret = i2c_smbus_read_i2c_block_data(client, reg, len, buf);
    if (ret < 0)
        dev_err(&client->dev, "Failed to read block from reg 0x%02x: %d\n", reg, ret);
    else if (ret != len)
        return -EIO;

    return ret;
}

/**
 * Initialize the MPU6050 hardware
 */
static int mpu6050_hw_init(struct i2c_client *client)
{
    int ret;

    /* Reset the device */
    ret = mpu6050_write_reg(client, MPU6050_PWR_MGMT_1, MPU6050_RESET);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to reset device: %d\n", ret);
        return ret;
    }

    /* Use schedule_timeout_interruptible() instead of msleep() */
    schedule_timeout_interruptible(msecs_to_jiffies(100));

    /* Wake up device and select best clock source */
    ret = mpu6050_write_reg(client, MPU6050_PWR_MGMT_1, MPU6050_CLOCK_PLL);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to wake up device: %d\n", ret);
        return ret;
    }

    schedule_timeout_interruptible(msecs_to_jiffies(10));

    /* Configure gyroscope range */
    ret = mpu6050_write_reg(client, MPU6050_GYRO_CONFIG,
                            (MPU6050_GYRO_FS_500 << 3));
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to configure gyroscope: %d\n", ret);
        return ret;
    }

    /* Configure accelerometer range */
    ret = mpu6050_write_reg(client, MPU6050_ACCEL_CONFIG,
                            (MPU6050_ACCEL_FS_2G << 3));
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to configure accelerometer: %d\n", ret);
        return ret;
    }

    /* Set sample rate divider (for 100Hz) */
    ret = mpu6050_write_reg(client, MPU6050_SMPLRT_DIV, 9); /* 1000/(1+9) = 100Hz */
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to set sample rate: %d\n", ret);
        return ret;
    }

    /* Configure digital low pass filter */
    ret = mpu6050_write_reg(client, MPU6050_CONFIG, MPU6050_DLPF_10HZ);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to configure DLPF: %d\n", ret);
        return ret;
    }

    /* Disable interrupts */
    ret = mpu6050_write_reg(client, MPU6050_INT_ENABLE, 0x00);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to disable interrupts: %d\n", ret);
        return ret;
    }

    /* Verify device ID */
    ret = mpu6050_read_reg(client, MPU6050_WHO_AM_I);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to read device ID: %d\n", ret);
        return ret;
    }

    if (ret != MPU6050_DEVICE_ID)
    {
        dev_err(&client->dev, "Unexpected device ID: got 0x%02x, expected 0x%02x\n",
                ret, MPU6050_DEVICE_ID);
        return -ENODEV;
    }

    dev_info(&client->dev, "MPU6050 initialized\n");
    return 0;
}

/**
 * Initialize the MPU6050 module
 */
int mpu6050_init(void)
{
    struct i2c_adapter *adapter;
    struct i2c_client *client;
    int ret;

    pr_info("MPU6050: initializing\n");

    /* Clear device structure */
    memset(&mpu6050_device, 0, sizeof(mpu6050_device));

    /* Get I2C adapter */
    adapter = i2c_get_adapter(MPU6050_I2C_BUS);
    if (!adapter)
    {
        pr_err("MPU6050: failed to get I2C adapter\n");
        return -ENODEV;
    }

    /* Create I2C client */
    client = i2c_new_dummy(adapter, MPU6050_I2C_ADDR);
    if (!client)
    {
        pr_err("MPU6050: failed to create I2C client\n");
        i2c_put_adapter(adapter);
        return -ENOMEM;
    }

    /* Initialize hardware */
    ret = mpu6050_hw_init(client);
    if (ret)
    {
        pr_err("MPU6050: hardware initialization failed: %d\n", ret);
        i2c_unregister_device(client);
        i2c_put_adapter(adapter);
        return ret;
    }

    /* Store client in device structure */
    mpu6050_device.client = client;
    mpu6050_device.initialized = true;

    i2c_put_adapter(adapter);
    pr_info("MPU6050: initialization complete\n");
    return 0;
}

/**
 * Cleanup the MPU6050 module
 */
void mpu6050_cleanup(void)
{
    pr_info("MPU6050: cleaning up\n");

    if (mpu6050_device.initialized && mpu6050_device.client)
    {
        /* Put device in sleep mode to save power */
        mpu6050_write_reg(mpu6050_device.client, MPU6050_PWR_MGMT_1, MPU6050_SLEEP_MODE);

        /* Unregister I2C client */
        i2c_unregister_device(mpu6050_device.client);
        mpu6050_device.client = NULL;
        mpu6050_device.initialized = false;
    }
}

/**
 * Read sensor data from MPU6050
 */
int mpu6050_read_sensors(struct hexapod_imu_data *data)
{
    u8 buffer[14];
    int ret;

    if (!mpu6050_device.initialized || !mpu6050_device.client || !data)
        return -EINVAL;

    /* Read all sensors in a single transaction (registers 0x3B to 0x48) */
    ret = mpu6050_read_block(mpu6050_device.client, MPU6050_ACCEL_XOUT_H, 14, buffer);
    if (ret < 0)
        return ret;

    /* Combine high and low bytes into 16-bit signed values */
    data->accel_x = (s16)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (s16)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (s16)((buffer[4] << 8) | buffer[5]);

    /* Temperature data (not used in hexapod) */
    /* s16 temp = (s16)((buffer[6] << 8) | buffer[7]); */

    data->gyro_x = (s16)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (s16)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (s16)((buffer[12] << 8) | buffer[13]);

    return 0;
}

EXPORT_SYMBOL_GPL(mpu6050_init);
EXPORT_SYMBOL_GPL(mpu6050_cleanup);
EXPORT_SYMBOL_GPL(mpu6050_read_sensors);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MPU6050 IMU Driver");
MODULE_AUTHOR("StrongFood");
