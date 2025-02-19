#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include "servo.h"
#include "pca9685.h"

// PWM configuration for servos
#define PWM_FREQ         50          // 50Hz
#define PWM_PERIOD       20000      // 20ms in microseconds
#define PWM_MIN_DUTY     1000       // 1ms in microseconds
#define PWM_MAX_DUTY     2000       // 2ms in microseconds
#define PWM_RESOLUTION   4096       // 12-bit resolution

// PCA9685 addresses
#define PCA9685_I2C_ADDR1    0x40
#define PCA9685_I2C_ADDR2    0x41

// Servo configuration structure
struct servo_config {
    u8 i2c_addr;        // PCA9685 I2C address
    u8 channel;         // PCA9685 channel
    s16 min_angle;      // Minimum angle
    s16 max_angle;      // Maximum angle
    s16 offset;         // Angle offset for calibration
};

// Leg configuration (3 servos per leg)
struct leg_config {
    struct servo_config coxa;    // Hip joint
    struct servo_config femur;   // Thigh joint
    struct servo_config tibia;   // Knee joint
};

// Hexapod configuration (6 legs)
static struct leg_config legs[NUM_LEGS] = {
    // Right front leg (ID: 0)
    {
        .coxa  = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 0,  .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 },
        .femur = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 1,  .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 },
        .tibia = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 2,  .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 }
    },
    // Right middle leg (ID: 1)
    {
        .coxa  = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 3,  .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 },
        .femur = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 4,  .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 },
        .tibia = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 5,  .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 }
    },
    // Right back leg (ID: 2)
    {
        .coxa  = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 6,  .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 },
        .femur = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 7,  .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 },
        .tibia = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 8,  .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 }
    },
    // Left front leg (ID: 3)
    {
        .coxa  = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 9,  .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 },
        .femur = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 10, .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 },
        .tibia = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 11, .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 }
    },
    // Left middle leg (ID: 4)
    {
        .coxa  = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 12, .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 },
        .femur = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 13, .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 },
        .tibia = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 14, .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 }
    },
    // Left back leg (ID: 5)
    {
        .coxa  = { .i2c_addr = PCA9685_I2C_ADDR1, .channel = 15, .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 },
        .femur = { .i2c_addr = PCA9685_I2C_ADDR2, .channel = 0,  .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 },
        .tibia = { .i2c_addr = PCA9685_I2C_ADDR2, .channel = 1,  .min_angle = ANGLE_MIN, .max_angle = ANGLE_MAX, .offset = 0 }
    }
};

// Convert angle to PWM duty cycle
static u16 angle_to_pwm(s16 angle, const struct servo_config *config)
{
    s32 duty_us;
    u16 pwm_value;

    // Apply offset and constrain angle
    angle += config->offset;
    if (angle < config->min_angle)
        angle = config->min_angle;
    if (angle > config->max_angle)
        angle = config->max_angle;

    // Map angle to duty cycle (microseconds)
    duty_us = PWM_MIN_DUTY + ((angle - ANGLE_MIN) * (PWM_MAX_DUTY - PWM_MIN_DUTY)) / (ANGLE_MAX - ANGLE_MIN);

    // Convert to PWM value (12-bit)
    pwm_value = (duty_us * PWM_RESOLUTION) / PWM_PERIOD;
    if (pwm_value >= PWM_RESOLUTION)
        pwm_value = PWM_RESOLUTION - 1;

    return pwm_value;
}

// Get servo configuration for a specific leg and joint
static struct servo_config *get_servo_config(u8 leg_id, u8 joint_id)
{
    if (leg_id >= NUM_LEGS) {
        pr_err("Servo: Invalid leg ID %u\n", leg_id);
        return NULL;
    }

    switch (joint_id) {
        case JOINT_COXA:
            return &legs[leg_id].coxa;
        case JOINT_FEMUR:
            return &legs[leg_id].femur;
        case JOINT_TIBIA:
            return &legs[leg_id].tibia;
        default:
            pr_err("Servo: Invalid joint ID %u\n", joint_id);
            return NULL;
    }
}

int servo_init_all(void)
{
    int ret, i;

    // Initialize first PCA9685
    ret = pca9685_device_init(PCA9685_I2C_ADDR1);
    if (ret < 0) {
        pr_err("Servo: Failed to initialize first PCA9685\n");
        return ret;
    }

    // Initialize second PCA9685
    ret = pca9685_device_init(PCA9685_I2C_ADDR2);
    if (ret < 0) {
        pr_err("Servo: Failed to initialize second PCA9685\n");
        return ret;
    }

    // Move all servos to neutral position (0 degrees)
    for (i = 0; i < NUM_LEGS; i++) {
        ret = servo_move_leg(i, 0, 0, 0);
        if (ret < 0) {
            pr_err("Servo: Failed to set neutral position for leg %d\n", i);
            return ret;
        }
    }

    pr_info("Servo: All servos initialized successfully\n");
    return 0;
}

int servo_set_angle(u8 leg_id, u8 joint_id, s16 angle)
{
    struct servo_config *config;
    u16 pwm_value;
    int ret;

    // Get servo configuration
    config = get_servo_config(leg_id, joint_id);
    if (!config)
        return -EINVAL;

    // Convert angle to PWM value
    pwm_value = angle_to_pwm(angle, config);

    // Set PWM value
    ret = pca9685_set_pwm(config->i2c_addr, config->channel, 0, pwm_value);
    if (ret < 0) {
        pr_err("Servo: Failed to set PWM for leg %u joint %u (angle: %d)\n", 
               leg_id, joint_id, angle);
        return ret;
    }

    pr_debug("Servo: Set leg %u joint %u to angle %d (PWM: %u)\n", 
             leg_id, joint_id, angle, pwm_value);
    return 0;
}

int servo_move_leg(u8 leg_id, s16 coxa_angle, s16 femur_angle, s16 tibia_angle)
{
    int ret;

    if (leg_id >= NUM_LEGS) {
        pr_err("Servo: Invalid leg ID %u\n", leg_id);
        return -EINVAL;
    }

    // Set angles for all joints
    ret = servo_set_angle(leg_id, JOINT_COXA, coxa_angle);
    if (ret < 0)
        return ret;

    ret = servo_set_angle(leg_id, JOINT_FEMUR, femur_angle);
    if (ret < 0)
        return ret;

    ret = servo_set_angle(leg_id, JOINT_TIBIA, tibia_angle);
    if (ret < 0)
        return ret;

    pr_debug("Servo: Moved leg %u to angles: coxa=%d, femur=%d, tibia=%d\n", 
             leg_id, coxa_angle, femur_angle, tibia_angle);
    return 0;
}

void servo_cleanup(void)
{
    // Put all servos in neutral position
    int i;
    for (i = 0; i < NUM_LEGS; i++) {
        if (servo_move_leg(i, 0, 0, 0) < 0)
            pr_err("Servo: Failed to set neutral position for leg %d during cleanup\n", i);
    }
    msleep(500); // Wait for servos to reach position

    // Cleanup PCA9685 devices
    pca9685_cleanup();
    pr_info("Servo: Cleanup complete\n");
}

MODULE_LICENSE("GPL");
