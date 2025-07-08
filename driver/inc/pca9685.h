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
 * @file pca9685.h
 * @brief PCA9685 PWM controller driver
 *
 * Driver for the PCA9685 PWM controller, providing I2C communication
 * and PWM signal generation.
 */

#ifndef _PCA9685_H_
#define _PCA9685_H_

#include <linux/types.h>

/* I2C Bus and Addresses */
#define PCA9685_I2C_BUS 3       /* BeagleBone I2C3 */
#define PCA9685_I2C_ADDR_1 0x40 /* Primary address */
#define PCA9685_I2C_ADDR_2 0x41 /* Secondary address */

/* Controller configuration */
#define PCA9685_MAX_CHANNELS 32 /* 16 channels per controller, 2 controllers */
#define PCA9685_CHANNELS_PER_DEVICE 16

/* PCA9685 registers */
#define PCA9685_REG_MODE1 0x00
#define PCA9685_REG_MODE2 0x01
#define PCA9685_REG_LED0_ON_L 0x06
#define PCA9685_REG_ALL_LED_ON_L 0xFA
#define PCA9685_REG_ALL_LED_ON_H 0xFB
#define PCA9685_REG_ALL_LED_OFF_L 0xFC
#define PCA9685_REG_ALL_LED_OFF_H 0xFD
#define PCA9685_REG_PRESCALE 0xFE

/* Mode register bits */
#define PCA9685_BIT_RESTART 0x80
#define PCA9685_BIT_EXTCLK 0x40
#define PCA9685_BIT_AI 0x20 /* Auto-increment */
#define PCA9685_BIT_SLEEP 0x10
#define PCA9685_BIT_ALLCALL 0x01

#define PCA9685_BIT_INVRT 0x10
#define PCA9685_BIT_OCH 0x08
#define PCA9685_BIT_OUTDRV 0x04
#define PCA9685_BIT_OUTNE1 0x02
#define PCA9685_BIT_OUTNE0 0x01

/* Clock and PWM parameters */
#define PCA9685_CLOCK_FREQ 25000000UL /* 25 MHz */
#define PCA9685_PWM_FREQ_DEFAULT 50   /* 50 Hz default */
#define PCA9685_PWM_RES 4096          /* 12-bit resolution */

/* Servo pulse width limits (microseconds) */
#define PCA9685_PWM_MIN_US 1000 /* 1ms = -90 degrees */
#define PCA9685_PWM_MID_US 1500 /* 1.5ms = 0 degrees */
#define PCA9685_PWM_MAX_US 2000 /* 2ms = +90 degrees */

/* Timing limits */
#define PCA9685_PWM_FREQ_MIN 24   /* Minimum PWM frequency */
#define PCA9685_PWM_FREQ_MAX 1526 /* Maximum PWM frequency */

/**
 * Initialize the PCA9685 PWM controller system
 *
 * @return 0 on success, negative error code on failure
 */
int pca9685_init(void);

/**
 * Release resources used by PCA9685
 */
void pca9685_cleanup(void);

/**
 * Set PWM frequency for all controllers
 *
 * @param freq_hz Frequency in Hz (24-1526)
 * @return 0 on success, negative error code on failure
 */
int pca9685_set_pwm_freq(u16 freq_hz);

/**
 * Set PWM values for a specific channel
 *
 * @param channel Channel number (0-31)
 * @param on On-time count (0-4095)
 * @param off Off-time count (0-4095)
 * @return 0 on success, negative error code on failure
 */
int pca9685_set_pwm(u8 channel, u16 on, u16 off);

/**
 * Set PWM pulse width in microseconds for a channel
 *
 * @param channel Channel number (0-31)
 * @param us Pulse width in microseconds
 * @return 0 on success, negative error code on failure
 */
int pca9685_set_pwm_us(u8 channel, u16 us);

/**
 * Enable all PWM outputs on all controllers
 *
 * @return 0 on success, negative error code on failure
 */
int pca9685_enable_outputs(void);

#endif /* _PCA9685_H_ */
