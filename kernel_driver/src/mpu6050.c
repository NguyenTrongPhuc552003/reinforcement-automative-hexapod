#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include "mpu6050.h"

// Add error counters for diagnostics
static struct
{
    atomic_t read_errors;
    atomic_t write_errors;
    atomic_t timeout_errors;
} mpu6050_error_stats;

/**
 * Write to an MPU6050 register
 */
static int mpu6050_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
    int ret, retries = 3;

    if (!client)
        return -EINVAL;

    /* Try up to 3 times */
    while (retries--)
    {
        ret = i2c_smbus_write_byte_data(client, reg, val);
        if (ret == 0)
            return 0;

#ifdef DEBUG_I2C
        dev_err(&client->dev, "Write retry %d for reg 0x%02x\n",
                2 - retries, reg);
#endif
        /* Use schedule_timeout_interruptible() instead of msleep() */
        schedule_timeout_interruptible(msecs_to_jiffies(10));
    }

    // If all retries failed, increment error counter
    if (ret < 0)
    {
        atomic_inc(&mpu6050_error_stats.write_errors);
    }

    return ret;
}

/**
 * Read from an MPU6050 register
 */
static int mpu6050_read_reg(struct i2c_client *client, u8 reg)
{
    int ret, retries = 3;

    if (!client)
        return -EINVAL;

    /* Try up to 3 times */
    while (retries--)
    {
        ret = i2c_smbus_read_byte_data(client, reg);
        if (ret >= 0)
            return ret;

#ifdef DEBUG_I2C
        dev_err(&client->dev, "Read retry %d for reg 0x%02x\n",
                2 - retries, reg);
#endif
        /* Use schedule_timeout_interruptible() instead of msleep() */
        schedule_timeout_interruptible(msecs_to_jiffies(10));
    }

    return ret;
}

/**
 * Read multiple registers from MPU6050
 */
static int mpu6050_read_block(struct i2c_client *client, u8 reg,
                              u8 length, u8 *values)
{
    int ret;

    if (!client || !values)
        return -EINVAL;

    ret = i2c_smbus_read_i2c_block_data(client, reg, length, values);
    if (ret < 0)
    {
        dev_err(&client->dev, "Block read failed: %d\n", ret);
        return ret;
    }

    if (ret != length)
    {
        dev_err(&client->dev, "Block read: expected %d bytes, got %d\n",
                length, ret);
        return -EIO;
    }

    return 0;
}

/**
 * Initialize the MPU6050
 */
int mpu6050_init(struct i2c_client *client)
{
    u8 id;
    int ret;

    if (!client)
        return -EINVAL;

    // Initialize error counters
    atomic_set(&mpu6050_error_stats.read_errors, 0);
    atomic_set(&mpu6050_error_stats.write_errors, 0);
    atomic_set(&mpu6050_error_stats.timeout_errors, 0);

    /* Verify device ID */
    ret = mpu6050_read_reg(client, MPU6050_WHO_AM_I);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to read device ID: %d\n", ret);
        return ret;
    }

    id = ret & 0xFF;
    dev_info(&client->dev, "Device ID: 0x%02x\n", id);

    /* Accept various device IDs - MPU6050, MPU9250, and variants */
    if (id != MPU6050_DEVICE_ID)
    {
        dev_err(&client->dev, "Unsupported device ID: 0x%02x\n", id);
        return -ENODEV;
    }

    /* Reset device */
    ret = mpu6050_write_reg(client, MPU6050_PWR_MGMT_1, MPU6050_RESET);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to reset device: %d\n", ret);
        return ret;
    }
    // msleep(100); /* Wait for reset to complete */
    /* Use schedule_timeout_interruptible() instead of msleep() */
    schedule_timeout_interruptible(msecs_to_jiffies(100));

    /* Wake up device and select best clock source */
    ret = mpu6050_write_reg(client, MPU6050_PWR_MGMT_1, MPU6050_CLOCK_PLL);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to wake up device: %d\n", ret);
        return ret;
    }
    // msleep(10);
    /* Use schedule_timeout_interruptible() instead of msleep() */
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

    // Simplified success message
    dev_info(&client->dev, "MPU6050 initialized\n");
    return 0;
}

/**
 * Remove the MPU6050
 */
void mpu6050_remove(struct i2c_client *client)
{
    if (!client)
        return;

    /* Put device in sleep mode to save power */
    mpu6050_write_reg(client, MPU6050_PWR_MGMT_1, MPU6050_SLEEP_MODE);
}

/**
 * Read sensor data from MPU6050
 */
int mpu6050_read_sensors(struct i2c_client *client, struct hexapod_imu_data *data)
{
    u8 buffer[14];
    int ret;

    if (!client || !data)
        return -EINVAL;

    /* Read all sensors in a single transaction (registers 0x3B to 0x48) */
    ret = mpu6050_read_block(client, MPU6050_ACCEL_XOUT_H, 14, buffer);
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
EXPORT_SYMBOL_GPL(mpu6050_remove);
EXPORT_SYMBOL_GPL(mpu6050_read_sensors);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MPU6050 IMU Driver");
MODULE_AUTHOR("StrongFood");
