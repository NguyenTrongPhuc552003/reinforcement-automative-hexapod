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
 * @file adxl345.h
 * @brief ADXL345 3-axis accelerometer driver
 *
 * Driver for the ADXL345 accelerometer, providing I2C communication
 * and sensor data readings.
 */

#ifndef _ADXL345_H_
#define _ADXL345_H_

#include <linux/types.h>

/* ADXL345 I2C addresses */
#define ADXL345_I2C_ADDR1 0x1D /* ALT ADDRESS pin connected to VCC */
#define ADXL345_I2C_ADDR2 0x53 /* ALT ADDRESS pin connected to GND (default) */
#define ADXL345_I2C_BUS 2      /* Default I2C bus for ADXL345 on BeagleBone */

/* ADXL345 register addresses */
#define ADXL345_REG_DEVID 0x00          /* Device ID */
#define ADXL345_REG_THRESH_TAP 0x1D     /* Tap threshold */
#define ADXL345_REG_OFSX 0x1E           /* X-axis offset */
#define ADXL345_REG_OFSY 0x1F           /* Y-axis offset */
#define ADXL345_REG_OFSZ 0x20           /* Z-axis offset */
#define ADXL345_REG_DUR 0x21            /* Tap duration */
#define ADXL345_REG_LATENT 0x22         /* Tap latency */
#define ADXL345_REG_WINDOW 0x23         /* Tap window */
#define ADXL345_REG_THRESH_ACT 0x24     /* Activity threshold */
#define ADXL345_REG_THRESH_INACT 0x25   /* Inactivity threshold */
#define ADXL345_REG_TIME_INACT 0x26     /* Inactivity time */
#define ADXL345_REG_ACT_INACT_CTL 0x27  /* Activity/inactivity control */
#define ADXL345_REG_THRESH_FF 0x28      /* Free-fall threshold */
#define ADXL345_REG_TIME_FF 0x29        /* Free-fall time */
#define ADXL345_REG_TAP_AXES 0x2A       /* Tap axes control */
#define ADXL345_REG_ACT_TAP_STATUS 0x2B /* Activity/tap status */
#define ADXL345_REG_BW_RATE 0x2C        /* Data rate and power mode control */
#define ADXL345_REG_POWER_CTL 0x2D      /* Power-saving features control */
#define ADXL345_REG_INT_ENABLE 0x2E     /* Interrupt enable control */
#define ADXL345_REG_INT_MAP 0x2F        /* Interrupt mapping control */
#define ADXL345_REG_INT_SOURCE 0x30     /* Interrupt source */
#define ADXL345_REG_DATA_FORMAT 0x31    /* Data format control */
#define ADXL345_REG_DATAX0 0x32         /* X-axis data 0 */
#define ADXL345_REG_DATAX1 0x33         /* X-axis data 1 */
#define ADXL345_REG_DATAY0 0x34         /* Y-axis data 0 */
#define ADXL345_REG_DATAY1 0x35         /* Y-axis data 1 */
#define ADXL345_REG_DATAZ0 0x36         /* Z-axis data 0 */
#define ADXL345_REG_DATAZ1 0x37         /* Z-axis data 1 */
#define ADXL345_REG_FIFO_CTL 0x38       /* FIFO control */
#define ADXL345_REG_FIFO_STATUS 0x39    /* FIFO status */

/* ADXL345 constants */
#define ADXL345_DEVICE_ID 0xE5

/* ADXL345 POWER_CTL register bits */
#define ADXL345_POWER_CTL_MEASURE (1 << 3)
#define ADXL345_POWER_CTL_SLEEP (1 << 2)
#define ADXL345_POWER_CTL_AUTO_SLEEP (1 << 4)
#define ADXL345_POWER_CTL_LINK (1 << 5)

/* ADXL345 DATA_FORMAT register bits */
#define ADXL345_DATA_FORMAT_RANGE_MASK 0x03
#define ADXL345_DATA_FORMAT_2G 0x00
#define ADXL345_DATA_FORMAT_4G 0x01
#define ADXL345_DATA_FORMAT_8G 0x02
#define ADXL345_DATA_FORMAT_16G 0x03
#define ADXL345_DATA_FORMAT_FULL_RES (1 << 3)

/**
 * @brief ADXL345 device structure
 */
struct adxl345_dev
{
    int i2c_fd;      /* I2C device file descriptor */
    u8 i2c_addr;     /* I2C device address */
    int initialized; /* Initialization flag */
};

/**
 * @brief ADXL345 acceleration data structure
 */
struct adxl345_accel_data
{
    s16 x; /* X-axis acceleration */
    s16 y; /* Y-axis acceleration */
    s16 z; /* Z-axis acceleration */
};

/**
 * @brief Initialize ADXL345 device
 *
 * @param i2c_bus I2C bus number
 * @return 0 on success, negative error code on failure
 */
int adxl345_init(int i2c_bus);

/**
 * @brief Close and clean up ADXL345 device
 */
void adxl345_cleanup(void);

/**
 * @brief Read acceleration data from ADXL345
 *
 * @param data Pointer to acceleration data structure
 * @return 0 on success, negative error code on failure
 */
int adxl345_read_accel(struct adxl345_accel_data *data);

/**
 * @brief Set measurement range for ADXL345
 *
 * @param range Range value (0=2G, 1=4G, 2=8G, 3=16G)
 * @param full_res Full resolution mode (0 or 1)
 * @return 0 on success, negative error code on failure
 */
int adxl345_set_range(u8 range, u8 full_res);

/**
 * @brief Set power mode for ADXL345
 *
 * @param measure Enable measurement mode
 * @return 0 on success, negative error code on failure
 */
int adxl345_set_power_mode(bool measure);

/**
 * @brief Read a register from the ADXL345
 *
 * @param reg Register address
 * @param value Pointer to store the read value
 * @return 0 on success, negative error code on failure
 */
int adxl345_read_register(u8 reg, u8 *value);

/**
 * @brief Write a value to a register on the ADXL345
 *
 * @param reg Register address
 * @param value Value to write
 * @return 0 on success, negative error code on failure
 */
int adxl345_write_register(u8 reg, u8 value);

/**
 * @brief Check if ADXL345 device is connected
 *
 * @return 1 if device is found, 0 otherwise
 */
int adxl345_check_connection(void);

#endif /* _ADXL345_H_ */
