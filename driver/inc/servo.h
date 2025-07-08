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
 * @file servo.h
 * @brief Servo control driver
 *
 * Driver for controlling servos in the hexapod, providing
 * functions for setting angles and managing servo state.
 */

#ifndef _SERVO_H_
#define _SERVO_H_

#include <linux/types.h>

/* Servo configuration constants */
#define SERVO_NUM_LEGS 6
#define SERVO_NUM_JOINTS_PER_LEG 3
#define SERVO_TOTAL_SERVOS (SERVO_NUM_LEGS * SERVO_NUM_JOINTS_PER_LEG)

/* Servo angle limits (degrees) */
#define SERVO_MIN_ANGLE -90
#define SERVO_MAX_ANGLE 90

/* Servo timing constants (microseconds) */
#define SERVO_CENTER_US 1500 /* Center position (0 degrees) */
#define SERVO_RANGE_US 500   /* +/- range for full deflection */
#define SERVO_FREQ_HZ 50     /* Standard servo frequency */

/**
 * @brief Initialize servo control system
 *
 * Sets up servo configuration, initializes PCA9685 to proper frequency,
 * and centers all servos to their default positions.
 *
 * @return 0 on success, negative error code on failure
 */
int servo_init(void);

/**
 * @brief Clean up servo control system
 *
 * Centers all servos before shutdown and releases resources.
 */
void servo_cleanup(void);

/**
 * @brief Set servo angle for a specific leg and joint
 *
 * @param leg Leg number (0-5)
 * @param joint Joint number (0=hip, 1=knee, 2=ankle)
 * @param angle Angle in degrees (-90 to +90)
 * @return 0 on success, negative error code on failure
 */
int servo_set_angle(u8 leg, u8 joint, s16 angle);

/**
 * @brief Set calibration offsets for a specific leg
 *
 * These offsets are added to commanded angles before converting
 * to servo pulses, allowing for hardware alignment adjustment.
 *
 * @param leg Leg number (0-5)
 * @param hip_offset Hip joint offset in degrees
 * @param knee_offset Knee joint offset in degrees
 * @param ankle_offset Ankle joint offset in degrees
 * @return 0 on success, negative error code on failure
 */
int servo_set_calibration(u8 leg, s16 hip_offset, s16 knee_offset, s16 ankle_offset);

/**
 * @brief Center all servos to their zero positions
 *
 * Moves all servos to their neutral positions with calibration
 * offsets applied.
 *
 * @return 0 on success, negative error code on failure
 */
int servo_center_all(void);

#endif /* _SERVO_H_ */
