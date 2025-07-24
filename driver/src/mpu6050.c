/*
 * Hexapod Project - A Reinforcement Learning-based Autonomous Hexapod
 * Copyright (C) 2025  Nguyen Trong Phuc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/**
 * @file mpu6050.c
 * @brief Kernel-space driver for the MPU6050 IMU
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include "mpu6050.h"

/* Register map for MPU6050 */
#define MPU6050_REG_SMPLRT_DIV 0x19   /* Sample Rate Divider */
#define MPU6050_REG_CONFIG 0x1A       /* Configuration */
#define MPU6050_REG_GYRO_CONFIG 0x1B  /* Gyroscope Configuration */
#define MPU6050_REG_ACCEL_CONFIG 0x1C /* Accelerometer Configuration */
#define MPU6050_REG_FIFO_EN 0x23      /* FIFO Enable */
#define MPU6050_REG_INT_ENABLE 0x38   /* Interrupt Enable */
#define MPU6050_REG_ACCEL_XOUT_H 0x3B /* Accel X-axis High Byte */
#define MPU6050_REG_ACCEL_XOUT_L 0x3C /* Accel X-axis Low Byte */
#define MPU6050_REG_ACCEL_YOUT_H 0x3D /* Accel Y-axis High Byte */
#define MPU6050_REG_ACCEL_YOUT_L 0x3E /* Accel Y-axis Low Byte */
#define MPU6050_REG_ACCEL_ZOUT_H 0x3F /* Accel Z-axis High Byte */
#define MPU6050_REG_ACCEL_ZOUT_L 0x40 /* Accel Z-axis Low Byte */
#define MPU6050_REG_TEMP_OUT_H 0x41   /* Temperature High Byte */
#define MPU6050_REG_TEMP_OUT_L 0x42   /* Temperature Low Byte */
#define MPU6050_REG_GYRO_XOUT_H 0x43  /* Gyro X-axis High Byte */
#define MPU6050_REG_GYRO_XOUT_L 0x44  /* Gyro X-axis Low Byte */
#define MPU6050_REG_GYRO_YOUT_H 0x45  /* Gyro Y-axis High Byte */
#define MPU6050_REG_GYRO_YOUT_L 0x46  /* Gyro Y-axis Low Byte */
#define MPU6050_REG_GYRO_ZOUT_H 0x47  /* Gyro Z-axis High Byte */
#define MPU6050_REG_GYRO_ZOUT_L 0x48  /* Gyro Z-axis Low Byte */
#define MPU6050_REG_PWR_MGMT_1 0x6B   /* Power Management 1 */
#define MPU6050_REG_PWR_MGMT_2 0x6C   /* Power Management 2 */
#define MPU6050_REG_WHO_AM_I 0x75     /* Who Am I */

/* MPU6050 specific constants */
#define MPU6050_DEVICE_ID 0x68 /* Expected value from WHO_AM_I register */
#define MPU6050_RESET_BIT 0x80 /* Reset bit in PWR_MGMT_1 */
#define MPU6050_SLEEP_BIT 0x40 /* Sleep bit in PWR_MGMT_1 */
#define MPU6050_CLOCK_PLL 0x01 /* Use PLL with X axis gyro reference */

#define MPU6050_INIT_DELAY_MS 100  /* Delay after reset (ms) */
#define MPU6050_WAKEUP_DELAY_MS 10 /* Delay after wake-up (ms) */

/* Default configuration values */
static const struct mpu6050_config mpu6050_default_config = {
    .gyro_range = MPU6050_GYRO_FS_500,  /* ±500 degrees/second */
    .accel_range = MPU6050_ACCEL_FS_2G, /* ±2g */
    .dlpf_setting = MPU6050_DLPF_10HZ,  /* 10Hz low-pass filter for more responsive data */
    .sample_rate_div = 9,               /* 1000/(1+4) = 200Hz */
};

/* MPU6050 device structure */
struct mpu6050_dev
{
    struct i2c_client *client;    /* I2C client for the device */
    struct mpu6050_config config; /* Current configuration */
    struct mutex lock;            /* Lock for thread safety */
    int initialized;              /* Initialization status */
};

/* Global device structure */
static struct mpu6050_dev mpu6050_device;

/**
 * Write a register with retry logic
 *
 * @param client I2C client
 * @param reg Register address
 * @param value Value to write
 * @return 0 on success, negative error code on failure
 */
static int mpu6050_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
    int ret;

    ret = i2c_smbus_write_byte_data(client, reg, value);
    if (ret < 0)
        dev_err(&client->dev, "Failed to write reg 0x%02x: %d\n", reg, ret);

    return ret;
}

/**
 * Read a register with retry logic
 *
 * @param client I2C client
 * @param reg Register address
 * @return Register value on success, negative error code on failure
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
 * Read a block of data with retry logic
 *
 * @param client I2C client
 * @param reg Starting register address
 * @param len Number of bytes to read
 * @param buf Buffer to store read data
 * @return Number of bytes read on success, negative error code on failure
 */
static int mpu6050_read_block(struct i2c_client *client, u8 reg, u8 len, u8 *buf)
{
    int ret;

    ret = i2c_smbus_read_i2c_block_data(client, reg, len, buf);
    if (ret < 0)
        dev_err(&client->dev, "Failed to read block from reg 0x%02x: %d\n", reg, ret);

    return ret;
}

/**
 * Initialize MPU6050 hardware with the given configuration
 *
 * @param client I2C client
 * @param config Configuration parameters
 * @return 0 on success, negative error code on failure
 */
static int mpu6050_hw_init(struct i2c_client *client, const struct mpu6050_config *config)
{
    int ret;

    /* Reset the device */
    ret = mpu6050_write_reg(client, MPU6050_REG_PWR_MGMT_1, MPU6050_RESET_BIT);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to reset device: %d\n", ret);
        return ret;
    }

    /* Wait for reset to complete */
    schedule_timeout_interruptible(msecs_to_jiffies(MPU6050_INIT_DELAY_MS));

    /* Wake up device and select best clock source */
    ret = mpu6050_write_reg(client, MPU6050_REG_PWR_MGMT_1, MPU6050_CLOCK_PLL);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to wake up device: %d\n", ret);
        return ret;
    }

    schedule_timeout_interruptible(msecs_to_jiffies(MPU6050_WAKEUP_DELAY_MS));

    /* Configure gyroscope range */
    ret = mpu6050_write_reg(client, MPU6050_REG_GYRO_CONFIG, (config->gyro_range << 3));
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to configure gyroscope: %d\n", ret);
        return ret;
    }

    /* Configure accelerometer range */
    ret = mpu6050_write_reg(client, MPU6050_REG_ACCEL_CONFIG, (config->accel_range << 3));
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to configure accelerometer: %d\n", ret);
        return ret;
    }

    /* Set sample rate divider */
    ret = mpu6050_write_reg(client, MPU6050_REG_SMPLRT_DIV, config->sample_rate_div);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to set sample rate: %d\n", ret);
        return ret;
    }

    /* Configure digital low pass filter */
    ret = mpu6050_write_reg(client, MPU6050_REG_CONFIG, config->dlpf_setting);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to configure DLPF: %d\n", ret);
        return ret;
    }

    /* Disable interrupts */
    ret = mpu6050_write_reg(client, MPU6050_REG_INT_ENABLE, 0x00);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to disable interrupts: %d\n", ret);
        return ret;
    }

    /* Verify device ID */
    ret = mpu6050_read_reg(client, MPU6050_REG_WHO_AM_I);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to read WHO_AM_I register: %d\n", ret);
        return ret;
    }

    if (ret != MPU6050_DEVICE_ID)
    {
        dev_err(&client->dev, "Unexpected WHO_AM_I value: got 0x%02x, expected 0x%02x\n",
                ret, MPU6050_DEVICE_ID);
        return -ENODEV;
    }

    dev_info(&client->dev, "MPU6050 initialized\n");
    return 0;
}

/**
 * Initialize the MPU6050 module with default configuration
 *
 * @return 0 on success, negative error code on failure
 */
int mpu6050_init(void)
{
    return mpu6050_init_with_config(&mpu6050_default_config);
}
EXPORT_SYMBOL_GPL(mpu6050_init);

/**
 * Initialize the MPU6050 with specific configuration
 *
 * @param config Pointer to configuration structure
 * @return 0 on success, negative error code on failure
 */
int mpu6050_init_with_config(const struct mpu6050_config *config)
{
    struct i2c_adapter *adapter;
    struct i2c_client *client;
    int ret = -EINVAL; // Initialize to an error value

    /* If already initialized, return success */
    if (mpu6050_device.initialized)
    {
        pr_info("MPU6050: Already initialized\n");
        return 0;
    }

    pr_info("MPU6050: Initializing\n");

    /* Initialize device structure */
    memset(&mpu6050_device, 0, sizeof(mpu6050_device));
    mutex_init(&mpu6050_device.lock);

    /* Set configuration (use default if NULL provided) */
    if (config)
    {
        memcpy(&mpu6050_device.config, config, sizeof(struct mpu6050_config));
    }
    else
    {
        memcpy(&mpu6050_device.config, &mpu6050_default_config, sizeof(struct mpu6050_config));
    }

    /* Get I2C adapter */
    adapter = i2c_get_adapter(MPU6050_I2C_BUS);
    if (!adapter)
    {
        pr_err("MPU6050: Failed to get I2C adapter %d\n", MPU6050_I2C_BUS);
        return -ENODEV;
    }

    /* Create I2C client */
    client = i2c_new_dummy_device(adapter, MPU6050_I2C_ADDR);
    if (IS_ERR(client))
    {
        pr_err("MPU6050: Failed to create I2C client\n");
        i2c_put_adapter(adapter);
        return PTR_ERR(client);
    }

    /* Initialize hardware */
    ret = mpu6050_hw_init(client, &mpu6050_device.config);
    if (ret)
    {
        pr_err("MPU6050: Hardware initialization failed: %d\n", ret);
        i2c_unregister_device(client);
        i2c_put_adapter(adapter);
        return ret;
    }

    /* Store client in device structure */
    mpu6050_device.client = client;
    mpu6050_device.initialized = 1;

    i2c_put_adapter(adapter);
    pr_info("MPU6050: Initialization complete\n");
    return 0;
}
EXPORT_SYMBOL_GPL(mpu6050_init_with_config);

/**
 * Read sensor data from MPU6050
 *
 * @param data Pointer to data structure to fill with sensor readings
 * @return 0 on success, negative error code on failure
 */
int mpu6050_read_sensors(struct mpu6050_imu_data *data)
{
    u8 buffer[14];
    int ret;
    u8 pwr_mgmt;

    if (!mpu6050_device.initialized || !mpu6050_device.client || !data)
    {
        return -EINVAL;
    }

    mutex_lock(&mpu6050_device.lock);

    /* Check if device is in sleep mode and wake it up if needed */
    pwr_mgmt = i2c_smbus_read_byte_data(mpu6050_device.client, MPU6050_REG_PWR_MGMT_1);
    if (pwr_mgmt & MPU6050_SLEEP_BIT)
    {
        /* Device is sleeping, wake it up */
        ret = mpu6050_write_reg(mpu6050_device.client, MPU6050_REG_PWR_MGMT_1, MPU6050_CLOCK_PLL);
        if (ret < 0)
        {
            mutex_unlock(&mpu6050_device.lock);
            return ret;
        }

        /* Give the device time to wake up */
        schedule_timeout_interruptible(msecs_to_jiffies(MPU6050_WAKEUP_DELAY_MS));
    }

    /* Read all sensors in a single transaction (registers 0x3B to 0x48) */
    ret = mpu6050_read_block(mpu6050_device.client, MPU6050_REG_ACCEL_XOUT_H, 14, buffer);

    mutex_unlock(&mpu6050_device.lock);

    if (ret < 0)
    {
        return ret;
    }

    /* Combine high and low bytes into 16-bit signed values */
    data->accel_x = (s16)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (s16)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (s16)((buffer[4] << 8) | buffer[5]);

    /* Skip temperature data at indices 6-7 */

    data->gyro_x = (s16)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (s16)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (s16)((buffer[12] << 8) | buffer[13]);

    return 0;
}
EXPORT_SYMBOL_GPL(mpu6050_read_sensors);

/**
 * Get device initialization status
 *
 * @return 1 if device is initialized, 0 if not
 */
int mpu6050_is_initialized(void)
{
    return mpu6050_device.initialized ? 1 : 0;
}
EXPORT_SYMBOL_GPL(mpu6050_is_initialized);

/**
 * Cleanup the MPU6050 module
 */
void mpu6050_cleanup(void)
{
    pr_info("MPU6050: Cleaning up\n");

    mutex_lock(&mpu6050_device.lock);

    if (mpu6050_device.initialized && mpu6050_device.client)
    {
        /* Put device in sleep mode to save power */
        mpu6050_write_reg(mpu6050_device.client, MPU6050_REG_PWR_MGMT_1, MPU6050_SLEEP_BIT);

        /* Unregister I2C client */
        i2c_unregister_device(mpu6050_device.client);
        mpu6050_device.client = NULL;
        mpu6050_device.initialized = 0;
    }

    mutex_unlock(&mpu6050_device.lock);
}
EXPORT_SYMBOL_GPL(mpu6050_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MPU6050 IMU Driver");
MODULE_AUTHOR("StrongFood");
MODULE_VERSION("1.0");
