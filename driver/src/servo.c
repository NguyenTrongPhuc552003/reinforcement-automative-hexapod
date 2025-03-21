#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include "servo.h"
#include "pca9685.h"

/* Servo mapping - which channel for each servo */
static const u8 servo_map[NUM_LEGS][NUM_JOINTS_PER_LEG] = {
    {0, 1, 2}, /* Leg 0 - First PCA9685 */
    {3, 4, 5}, /* Leg 1 - First PCA9685 */
    {6, 7, 8}, /* Leg 2 - First PCA9685 */
    {0, 1, 2}, /* Leg 3 - Second PCA9685 */
    {3, 4, 5}, /* Leg 4 - Second PCA9685 */
    {6, 7, 8}  /* Leg 5 - Second PCA9685 */
};

/* Which PCA9685 controller to use for each leg (0 = first, 1 = second) */
static const u8 leg_controller[NUM_LEGS] = {
    0, 0, 0, /* First 3 legs use first controller */
    1, 1, 1  /* Last 3 legs use second controller */
};

/* Calibration offsets */
static s16 servo_offsets[NUM_LEGS][NUM_JOINTS_PER_LEG] = {{0}};

/* Convert angle to pulse width */
static u16 angle_to_pulse(s16 angle)
{
    const s16 center = 1500; /* 1500 µs is center (0°) */
    const s16 range = 500;   /* ±500 µs for ±90° */
    s16 pulse;

    /* Limit angle range */
    if (angle < SERVO_MIN_ANGLE)
        angle = SERVO_MIN_ANGLE;
    else if (angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;

    /* Calculate pulse width in µs */
    pulse = center + (angle * range) / SERVO_MAX_ANGLE;
    return pulse;
}

/* Initialize servo subsystem */
int servo_init(void)
{
    int ret;

    /* Initialize all servo offsets to 0 */
    memset(servo_offsets, 0, sizeof(servo_offsets));

    /* Set default PWM frequency for servos (50Hz) */
    ret = pca9685_set_pwm_freq(50);
    if (ret)
        return ret;

    /* Center all servos on startup */
    ret = servo_center_all();
    if (ret)
        return ret;

    pr_info("Servo subsystem initialized\n");
    return 0;
}

/* Clean up servo subsystem */
void servo_cleanup(void)
{
    /* Center all servos before shutting down */
    servo_center_all();
    pr_info("Servo subsystem cleaned up\n");
}

/* Set servo angle */
int servo_set_angle(u8 leg, u8 joint, s16 angle)
{
    u8 channel;
    u16 pulse_width;
    u8 controller_offset = 0;

    /* Validate parameters */
    if (leg >= NUM_LEGS || joint >= NUM_JOINTS_PER_LEG)
        return -EINVAL;

    /* Get the appropriate channel */
    channel = servo_map[leg][joint];

    /* If using two controllers and this is a leg that should use second controller */
    if (default_config.use_secondary_controller && leg_controller[leg] == 1)
    {
        controller_offset = 16;
        channel += controller_offset;
    }
    else if (!default_config.use_secondary_controller && leg_controller[leg] == 1)
    {
        /* In test mode with only one controller, remap legs that would normally use
         * the second controller to the first controller
         */
        channel = servo_map[leg][joint]; // Use the channel as is with first controller
    }

    /* Apply calibration offset */
    angle += servo_offsets[leg][joint];

    /* Convert angle to pulse width */
    pulse_width = angle_to_pulse(angle);

    /* Set the pulse width */
    return pca9685_set_pwm_us(channel, pulse_width);
}

/* Set calibration for a leg */
int servo_set_calibration(u8 leg, s16 hip_offset, s16 knee_offset, s16 ankle_offset)
{
    if (leg >= NUM_LEGS)
        return -EINVAL;

    /* Store offsets */
    servo_offsets[leg][0] = hip_offset;
    servo_offsets[leg][1] = knee_offset;
    servo_offsets[leg][2] = ankle_offset;

    return 0;
}

/* Center all servos */
int servo_center_all(void)
{
    int leg, joint, ret;

    for (leg = 0; leg < NUM_LEGS; leg++)
    {
        for (joint = 0; joint < NUM_JOINTS_PER_LEG; joint++)
        {
            ret = servo_set_angle(leg, joint, 0);
            if (ret)
                return ret;

            /* Use schedule() instead of msleep for better system responsiveness */
            schedule_timeout_interruptible(msecs_to_jiffies(5));
        }
    }

    return 0;
}

EXPORT_SYMBOL_GPL(servo_init);
EXPORT_SYMBOL_GPL(servo_cleanup);
EXPORT_SYMBOL_GPL(servo_set_angle);
EXPORT_SYMBOL_GPL(servo_set_calibration);
EXPORT_SYMBOL_GPL(servo_center_all);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Hexapod Servo Control");
MODULE_AUTHOR("StrongFood");
