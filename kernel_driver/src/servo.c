#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include "servo.h"
#include "pca9685.h"

/* Naming inconsistency: Using NUM_JOINTS_PER_LEG vs JOINTS_PER_LEG */
// Servo channel mapping
static const u8 servo_map[NUM_LEGS][NUM_JOINTS_PER_LEG] = {
    {0,  1,  2},    /* Leg 0 */
    {3,  4,  5},    /* Leg 1 */
    {6,  7,  8},    /* Leg 2 */
    {9,  10, 11},  /* Leg 3 */
    {12, 13, 14}, /* Leg 4 */
    {15, 0,  1}  /* Leg 5 */
};

/* Servo calibration data */
static s16 servo_offset[NUM_LEGS][NUM_JOINTS_PER_LEG] = {0};
static bool servo_reverse[NUM_LEGS][NUM_JOINTS_PER_LEG] = {false};

/* Initialize servo subsystem */
int servo_init(void)
{
    int ret;

    ret = pca9685_init();
    if (ret < 0)
    {
        pr_err("Failed to initialize PCA9685\n");
        return ret;
    }

    /* Set servo frequency */
    ret = pca9685_set_pwm_freq(PCA9685_FREQ);
    if (ret < 0)
    {
        pr_err("Failed to set PWM frequency\n");
        pca9685_cleanup();
        return ret;
    }

    /* Center all servos */
    ret = servo_center_all();
    if (ret < 0)
    {
        pr_err("Failed to center servos\n");
        pca9685_cleanup();
        return ret;
    }

    pr_info("Servo subsystem initialized\n");
    return 0;
}

/* Clean up servo subsystem */
void servo_cleanup(void)
{
    servo_center_all();
    pca9685_cleanup();
}

/* Update function naming for consistency */
int servo_set_angle(uint8_t leg, uint8_t joint, int16_t angle)
{
    u8 channel;

    if (leg >= NUM_LEGS || joint >= NUM_JOINTS_PER_LEG)
        return -EINVAL;

    /* Apply limits */
    if (angle < SERVO_MIN_ANGLE)
        angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;

    /* Apply calibration */
    if (servo_reverse[leg][joint])
        angle = -angle;
    angle += servo_offset[leg][joint];

    /* Get physical channel */
    channel = servo_map[leg][joint];

    /* Convert angle to PWM */
    return pca9685_set_pwm_ms(channel % SERVO_CHANNELS_PER_CONTROLLER,
                              PCA9685_MID_PULSE + (angle * (PCA9685_MAX_PULSE - PCA9685_MID_PULSE) / 90));
}

/* Get servo angle */
/* int servo_get_angle(uint8_t leg, uint8_t joint, int16_t *angle)
{
    return 0;
} */

/* Center all servos */
int servo_center_all(void)
{
    int leg, joint, ret;

    for (leg = 0; leg < NUM_LEGS; leg++)
    {
        for (joint = 0; joint < NUM_JOINTS_PER_LEG; joint++)
        {
            ret = servo_set_angle(leg, joint, 0);
            if (ret < 0)
                return ret;
            msleep(10);
        }
    }
    return 0;
}

EXPORT_SYMBOL_GPL(servo_init);
EXPORT_SYMBOL_GPL(servo_cleanup);
EXPORT_SYMBOL_GPL(servo_set_angle);
// EXPORT_SYMBOL_GPL(servo_get_angle);
EXPORT_SYMBOL_GPL(servo_center_all);
