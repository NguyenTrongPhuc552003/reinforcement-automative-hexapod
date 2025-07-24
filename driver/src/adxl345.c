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
 * @file adxl345.c
 * @brief Kernel-space driver implementation for ADXL345 3-axis accelerometer
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include "adxl345.h"

/* Global device structure */
static struct adxl345_dev g_adxl345_dev = {
    .initialized = 0,
    .i2c_fd = -1};

/* I2C adapter and client */
static struct i2c_adapter *i2c_adap = NULL;
static struct i2c_client *i2c_client = NULL;

/**
 * @brief Write a byte to an ADXL345 register
 */
int adxl345_write_register(u8 reg, u8 value)
{
    s32 ret;

    if (!g_adxl345_dev.initialized || !i2c_client)
    {
        pr_err("ADXL345: Device not initialized\n");
        return -ENODEV;
    }

    ret = i2c_smbus_write_byte_data(i2c_client, reg, value);
    if (ret < 0)
    {
        pr_err("ADXL345: Failed to write to register 0x%02x: %d\n", reg, ret);
        return ret;
    }

    return 0;
}

/**
 * @brief Read a byte from an ADXL345 register
 */
int adxl345_read_register(u8 reg, u8 *value)
{
    s32 ret;

    if (!g_adxl345_dev.initialized || !i2c_client)
    {
        pr_err("ADXL345: Device not initialized\n");
        return -ENODEV;
    }

    ret = i2c_smbus_read_byte_data(i2c_client, reg);
    if (ret < 0)
    {
        pr_err("ADXL345: Failed to read register 0x%02x: %d\n", reg, ret);
        return ret;
    }

    *value = (u8)ret;
    return 0;
}

/**
 * @brief Check if ADXL345 device is connected
 */
int adxl345_check_connection(void)
{
    u8 device_id;
    int ret;

    /* Read the DEVICE_ID register */
    ret = adxl345_read_register(ADXL345_REG_DEVID, &device_id);
    if (ret < 0)
        return 0; /* Connection failed */

    /* Check if the device ID matches */
    return (device_id == ADXL345_DEVICE_ID) ? 1 : 0;
}

/**
 * @brief Initialize ADXL345 device
 */
int adxl345_init(int i2c_bus)
{
    int ret;
    u8 device_id;
    struct i2c_board_info board_info = {
        I2C_BOARD_INFO("adxl345", ADXL345_I2C_ADDR2)};

    /* Check if already initialized */
    if (g_adxl345_dev.initialized)
    {
        pr_info("ADXL345: Device already initialized\n");
        return 0;
    }

    /* Get the I2C adapter */
    i2c_adap = i2c_get_adapter(i2c_bus);
    if (!i2c_adap)
    {
        pr_err("ADXL345: Failed to get I2C adapter %d\n", i2c_bus);
        return -ENODEV;
    }

    /* Create a new I2C client device */
    i2c_client = i2c_new_client_device(i2c_adap, &board_info);
    if (IS_ERR(i2c_client))
    {
        pr_err("ADXL345: Failed to create I2C client\n");
        i2c_put_adapter(i2c_adap);
        return PTR_ERR(i2c_client);
    }

    /* Store device information */
    g_adxl345_dev.i2c_addr = ADXL345_I2C_ADDR2;

    /* Verify device ID */
    ret = adxl345_read_register(ADXL345_REG_DEVID, &device_id);
    if (ret < 0)
    {
        /* Try alternative address */
        i2c_unregister_device(i2c_client);

        board_info.addr = ADXL345_I2C_ADDR1;
        i2c_client = i2c_new_client_device(i2c_adap, &board_info);
        if (IS_ERR(i2c_client))
        {
            pr_err("ADXL345: Failed to create I2C client with alt address\n");
            i2c_put_adapter(i2c_adap);
            return PTR_ERR(i2c_client);
        }

        g_adxl345_dev.i2c_addr = ADXL345_I2C_ADDR1;

        /* Try reading again with the new address */
        ret = adxl345_read_register(ADXL345_REG_DEVID, &device_id);
        if (ret < 0)
        {
            pr_err("ADXL345: Failed to read device ID\n");
            adxl345_cleanup();
            return ret;
        }
    }

    /* Verify device ID value */
    if (device_id != ADXL345_DEVICE_ID)
    {
        pr_err("ADXL345: Invalid device ID 0x%02x (expected 0xE5)\n", device_id);
        adxl345_cleanup();
        return -ENODEV;
    }

    pr_info("ADXL345: Found device with ID 0x%02x at address 0x%02x on bus %d\n",
            device_id, g_adxl345_dev.i2c_addr, i2c_bus);

    /* Configure the device */

    /* Set data format: +/-2g range, full-resolution mode */
    ret = adxl345_write_register(ADXL345_REG_DATA_FORMAT,
                                 ADXL345_DATA_FORMAT_FULL_RES);
    if (ret < 0)
    {
        pr_err("ADXL345: Failed to set data format\n");
        adxl345_cleanup();
        return ret;
    }

    /* Set data rate: 100 Hz (0x0A) */
    ret = adxl345_write_register(ADXL345_REG_BW_RATE, 0x0A);
    if (ret < 0)
    {
        pr_err("ADXL345: Failed to set data rate\n");
        adxl345_cleanup();
        return ret;
    }

    /* Enable measurement mode */
    ret = adxl345_set_power_mode(true);
    if (ret < 0)
    {
        pr_err("ADXL345: Failed to enable measurement mode\n");
        adxl345_cleanup();
        return ret;
    }

    g_adxl345_dev.initialized = 1;

    pr_info("ADXL345: Initialization complete\n");
    return 0;
}

/**
 * @brief Close and clean up ADXL345 device
 */
void adxl345_cleanup(void)
{
    if (!g_adxl345_dev.initialized)
        return;

    /* Disable measurement mode */
    adxl345_set_power_mode(false);

    /* Clean up I2C resources */
    if (i2c_client)
    {
        i2c_unregister_device(i2c_client);
        i2c_client = NULL;
    }

    if (i2c_adap)
    {
        i2c_put_adapter(i2c_adap);
        i2c_adap = NULL;
    }

    g_adxl345_dev.initialized = 0;
    pr_info("ADXL345: Cleanup complete\n");
}

/**
 * @brief Set power mode for ADXL345
 */
int adxl345_set_power_mode(bool measure)
{
    u8 value;
    int ret;

    /* Read current value */
    ret = adxl345_read_register(ADXL345_REG_POWER_CTL, &value);
    if (ret < 0)
        return ret;

    /* Update measurement bit */
    if (measure)
        value |= ADXL345_POWER_CTL_MEASURE;
    else
        value &= ~ADXL345_POWER_CTL_MEASURE;

    /* Write new value */
    ret = adxl345_write_register(ADXL345_REG_POWER_CTL, value);
    return ret;
}

/**
 * @brief Set measurement range for ADXL345
 */
int adxl345_set_range(u8 range, u8 full_res)
{
    u8 value;
    int ret;

    /* Validate range */
    if (range > ADXL345_DATA_FORMAT_16G)
        return -EINVAL;

    /* Read current value */
    ret = adxl345_read_register(ADXL345_REG_DATA_FORMAT, &value);
    if (ret < 0)
        return ret;

    /* Clear range bits and set new range */
    value &= ~ADXL345_DATA_FORMAT_RANGE_MASK;
    value |= range;

    /* Set or clear full resolution bit */
    if (full_res)
        value |= ADXL345_DATA_FORMAT_FULL_RES;
    else
        value &= ~ADXL345_DATA_FORMAT_FULL_RES;

    /* Write new value */
    ret = adxl345_write_register(ADXL345_REG_DATA_FORMAT, value);
    return ret;
}

/**
 * @brief Read acceleration data from ADXL345
 */
int adxl345_read_accel(struct adxl345_accel_data *data)
{
    int ret;
    u8 buf[6];
    s32 x, y, z;

    if (!g_adxl345_dev.initialized || !data)
        return -EINVAL;

    /* Read all 6 bytes in one transaction */
    ret = i2c_smbus_read_i2c_block_data(i2c_client, ADXL345_REG_DATAX0, 6, buf);
    if (ret != 6)
    {
        pr_err("ADXL345: Failed to read acceleration data: %d\n", ret);
        return (ret < 0) ? ret : -EIO;
    }

    /* Convert data (little-endian) */
    x = (s16)((buf[1] << 8) | buf[0]);
    y = (s16)((buf[3] << 8) | buf[2]);
    z = (s16)((buf[5] << 8) | buf[4]);

    data->x = x;
    data->y = y;
    data->z = z;

    return 0;
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ADXL345 Accelerometer Driver");
MODULE_AUTHOR("StrongFood");
MODULE_VERSION("1.0");
