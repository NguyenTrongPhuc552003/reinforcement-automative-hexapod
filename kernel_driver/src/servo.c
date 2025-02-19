#include <linux/module.h>
#include "../include/servo.h"
#include "../include/pca9685.h"

#define SERVO_FREQ          50    // 50Hz for standard servos
#define MIN_PULSE_WIDTH     600   // 0.6ms
#define MAX_PULSE_WIDTH     2400  // 2.4ms
#define DEFAULT_PULSE_WIDTH 1500  // 1.5ms (center position)
#define SERVO_I2C_ADDR      0x40

// Servo mapping configuration for hexapod
static const struct servo_config default_servo_configs[NUM_LEGS][SERVOS_PER_LEG] = {
    // Right side servos (0-8)
    [LEG_FRONT_RIGHT] = {
        [JOINT_COXA]  = {.channel = 0,  .min_angle = 0, .max_angle = 180, .default_angle = COXA_DEFAULT,  .inverted = false},
        [JOINT_FEMUR] = {.channel = 1,  .min_angle = 0, .max_angle = 180, .default_angle = FEMUR_DEFAULT, .inverted = false},
        [JOINT_TIBIA] = {.channel = 2,  .min_angle = 0, .max_angle = 180, .default_angle = TIBIA_DEFAULT, .inverted = false}
    },
    [LEG_MIDDLE_RIGHT] = {
        [JOINT_COXA]  = {.channel = 3,  .min_angle = 0, .max_angle = 180, .default_angle = COXA_DEFAULT,  .inverted = false},
        [JOINT_FEMUR] = {.channel = 4,  .min_angle = 0, .max_angle = 180, .default_angle = FEMUR_DEFAULT, .inverted = false},
        [JOINT_TIBIA] = {.channel = 5,  .min_angle = 0, .max_angle = 180, .default_angle = TIBIA_DEFAULT, .inverted = false}
    },
    [LEG_BACK_RIGHT] = {
        [JOINT_COXA]  = {.channel = 6,  .min_angle = 0, .max_angle = 180, .default_angle = COXA_DEFAULT,  .inverted = false},
        [JOINT_FEMUR] = {.channel = 7,  .min_angle = 0, .max_angle = 180, .default_angle = FEMUR_DEFAULT, .inverted = false},
        [JOINT_TIBIA] = {.channel = 8,  .min_angle = 0, .max_angle = 180, .default_angle = TIBIA_DEFAULT, .inverted = false}
    },
    // Left side servos (9-17)
    [LEG_FRONT_LEFT] = {
        [JOINT_COXA]  = {.channel = 9,  .min_angle = 0, .max_angle = 180, .default_angle = COXA_DEFAULT,  .inverted = true},
        [JOINT_FEMUR] = {.channel = 10, .min_angle = 0, .max_angle = 180, .default_angle = FEMUR_DEFAULT, .inverted = true},
        [JOINT_TIBIA] = {.channel = 11, .min_angle = 0, .max_angle = 180, .default_angle = TIBIA_DEFAULT, .inverted = true}
    },
    [LEG_MIDDLE_LEFT] = {
        [JOINT_COXA]  = {.channel = 12, .min_angle = 0, .max_angle = 180, .default_angle = COXA_DEFAULT,  .inverted = true},
        [JOINT_FEMUR] = {.channel = 13, .min_angle = 0, .max_angle = 180, .default_angle = FEMUR_DEFAULT, .inverted = true},
        [JOINT_TIBIA] = {.channel = 14, .min_angle = 0, .max_angle = 180, .default_angle = TIBIA_DEFAULT, .inverted = true}
    },
    [LEG_BACK_LEFT] = {
        [JOINT_COXA]  = {.channel = 15, .min_angle = 0, .max_angle = 180, .default_angle = COXA_DEFAULT,  .inverted = true},
        [JOINT_FEMUR] = {.channel = 16, .min_angle = 0, .max_angle = 180, .default_angle = FEMUR_DEFAULT, .inverted = true},
        [JOINT_TIBIA] = {.channel = 17, .min_angle = 0, .max_angle = 180, .default_angle = TIBIA_DEFAULT, .inverted = true}
    }
};

static struct servo_data servos[NUM_LEGS][SERVOS_PER_LEG];

static inline u16 angle_to_pulse(u8 angle)
{
    return MIN_PULSE_WIDTH + ((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * angle / 180);
}

int servo_init(struct servo_data *servo)
{
    if (!servo)
        return -EINVAL;

    return pca9685_device_init(SERVO_I2C_ADDR);
}

int servo_init_all(void)
{
    int leg, joint;
    int ret;

    // Initialize PCA9685
    ret = pca9685_device_init(SERVO_I2C_ADDR);
    if (ret < 0)
        return ret;

    // Set PWM frequency
    ret = pca9685_set_pwm_freq(SERVO_I2C_ADDR, SERVO_FREQ);
    if (ret < 0)
        return ret;

    // Initialize all servos
    for (leg = 0; leg < NUM_LEGS; leg++) {
        for (joint = 0; joint < SERVOS_PER_LEG; joint++) {
            servos[leg][joint].config = default_servo_configs[leg][joint];
            servos[leg][joint].is_initialized = true;
            servos[leg][joint].current_angle = default_servo_configs[leg][joint].default_angle;
            
            // Set initial position
            ret = servo_set_angle(leg, joint, default_servo_configs[leg][joint].default_angle);
            if (ret < 0)
                return ret;
        }
    }

    return 0;
}

int servo_set_angle(unsigned int leg_id, unsigned int joint_id, unsigned int angle)
{
    u16 pulse;
    u8 adjusted_angle;
    struct servo_data *servo;

    if (leg_id >= NUM_LEGS || joint_id >= SERVOS_PER_LEG)
        return -EINVAL;

    servo = &servos[leg_id][joint_id];
    
    if (!servo->is_initialized)
        return -EINVAL;

    // Clamp angle to valid range
    if (angle < servo->config.min_angle)
        angle = servo->config.min_angle;
    else if (angle > servo->config.max_angle)
        angle = servo->config.max_angle;

    // Invert angle if necessary
    if (servo->config.inverted)
        adjusted_angle = servo->config.max_angle - (angle - servo->config.min_angle);
    else
        adjusted_angle = angle;

    // Convert angle to pulse width
    pulse = angle_to_pulse(adjusted_angle);

    servo->current_angle = angle;
    return pca9685_set_pwm(SERVO_I2C_ADDR, servo->config.channel, 0, pulse);
}

int servo_move_leg(unsigned int leg_id, u8 coxa_angle, u8 femur_angle, u8 tibia_angle)
{
    int ret;

    if (leg_id >= NUM_LEGS)
        return -EINVAL;

    ret = servo_set_angle(leg_id, JOINT_COXA, coxa_angle);
    if (ret < 0)
        return ret;

    ret = servo_set_angle(leg_id, JOINT_FEMUR, femur_angle);
    if (ret < 0)
        return ret;

    return servo_set_angle(leg_id, JOINT_TIBIA, tibia_angle);
}

int servo_move_leg_relative(unsigned int leg_id, s8 coxa_delta, s8 femur_delta, s8 tibia_delta)
{
    struct servo_data *coxa;
    struct servo_data *femur;
    struct servo_data *tibia;

    if (leg_id >= NUM_LEGS)
        return -EINVAL;

    coxa = &servos[leg_id][JOINT_COXA];
    femur = &servos[leg_id][JOINT_FEMUR];
    tibia = &servos[leg_id][JOINT_TIBIA];

    return servo_move_leg(leg_id,
        coxa->current_angle + coxa_delta,
        femur->current_angle + femur_delta,
        tibia->current_angle + tibia_delta);
}

int servo_reset_all_to_default(void)
{
    int leg, joint;
    int ret;

    for (leg = 0; leg < NUM_LEGS; leg++) {
        for (joint = 0; joint < SERVOS_PER_LEG; joint++) {
            ret = servo_set_angle(leg, joint, default_servo_configs[leg][joint].default_angle);
            if (ret < 0)
                return ret;
        }
    }

    return 0;
}

int servo_get_current_angle(unsigned int leg_id, unsigned int joint_id)
{
    if (leg_id >= NUM_LEGS || joint_id >= SERVOS_PER_LEG)
        return -EINVAL;

    return servos[leg_id][joint_id].current_angle;
}

void servo_cleanup(void)
{
    // Reset all servos to default position before cleanup
    servo_reset_all_to_default();
    
    // Put PCA9685 to sleep
    pca9685_device_sleep(SERVO_I2C_ADDR);
}

EXPORT_SYMBOL_GPL(servo_init);
EXPORT_SYMBOL_GPL(servo_init_all);
EXPORT_SYMBOL_GPL(servo_set_angle);
EXPORT_SYMBOL_GPL(servo_move_leg);
EXPORT_SYMBOL_GPL(servo_move_leg_relative);
EXPORT_SYMBOL_GPL(servo_reset_all_to_default);
EXPORT_SYMBOL_GPL(servo_get_current_angle);
EXPORT_SYMBOL_GPL(servo_cleanup);

MODULE_LICENSE("GPL");