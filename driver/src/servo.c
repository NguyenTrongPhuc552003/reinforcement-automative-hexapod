/**
 * @file servo.c
 * @brief Kernel-space driver for controlling servos
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include "servo.h"
#include "pca9685.h"

/* Module parameters */
static int servo_debug = 0;
module_param(servo_debug, int, 0644);
MODULE_PARM_DESC(servo_debug, "Enable debug output (0=disabled, 1=enabled)");

/* Servo driver state */
struct servo_driver
{
    int initialized;                                       /* Initialization status flag */
    struct mutex lock;                                     /* Thread synchronization lock */
    s16 offsets[SERVO_NUM_LEGS][SERVO_NUM_JOINTS_PER_LEG]; /* Calibration offsets */
};

/* Global driver state */
static struct servo_driver servo_drv = {
    .initialized = 0,
};

/* Servo mapping - which channel for each servo */
static const u8 servo_map[SERVO_NUM_LEGS][SERVO_NUM_JOINTS_PER_LEG] = {
    {0, 1, 2},  /* Leg 0 - First PCA9685 */
    {4, 5, 6},  /* Leg 1 - First PCA9685 */
    {8, 9, 10}, /* Leg 2 - First PCA9685 */
    {0, 1, 2},  /* Leg 3 - Second PCA9685 */
    {4, 5, 6},  /* Leg 4 - Second PCA9685 */
    {8, 9, 10}  /* Leg 5 - Second PCA9685 */
};

/* Which PCA9685 controller to use for each leg (0=primary, 1=secondary) */
static const u8 leg_controller[SERVO_NUM_LEGS] = {
    0, 0, 0, /* First 3 legs use first controller */
    1, 1, 1  /* Last 3 legs use second controller */
};

/**
 * Convert angle in degrees to pulse width in microseconds
 *
 * @param angle Angle in degrees (-90 to +90)
 * @return Pulse width in microseconds
 */
static inline u16 angle_to_pulse(s16 angle)
{
    /* Ensure angle is within valid range */
    if (angle < SERVO_MIN_ANGLE)
        angle = SERVO_MIN_ANGLE;
    else if (angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;

    /* Convert angle to pulse width: 1500µs ± (angle * 500µs / 90°) */
    return SERVO_CENTER_US + ((angle * SERVO_RANGE_US) / SERVO_MAX_ANGLE);
}

/**
 * Validate channel parameters
 *
 * @param leg Leg number
 * @param joint Joint number
 * @return 0 if valid, -EINVAL if invalid
 */
static inline int validate_channel(u8 leg, u8 joint)
{
    if (leg >= SERVO_NUM_LEGS || joint >= SERVO_NUM_JOINTS_PER_LEG)
        return -EINVAL;
    return 0;
}

/**
 * Initialize servo control system
 *
 * @return 0 on success, negative error code on failure
 */
int servo_init(void)
{
    int ret;

    pr_info("Servo: Initializing servo subsystem\n");

    /* Check if already initialized */
    if (servo_drv.initialized)
    {
        pr_warn("Servo: Already initialized\n");
        return 0;
    }

    /* Initialize lock */
    mutex_init(&servo_drv.lock);

    /* Clear all calibration offsets */
    memset(servo_drv.offsets, 0, sizeof(servo_drv.offsets));

    /* Set PWM frequency for servos (50Hz standard) */
    ret = pca9685_set_pwm_freq(SERVO_FREQ_HZ);
    if (ret < 0)
    {
        pr_err("Servo: Failed to set PWM frequency: %d\n", ret);
        return ret;
    }

    /* Mark as initialized BEFORE centering servos */
    servo_drv.initialized = 1;

    /* Center all servos for safe starting position */
    ret = servo_center_all();
    if (ret < 0)
    {
        pr_err("Servo: Failed to center servos: %d\n", ret);
        servo_drv.initialized = 0; /* Reset flag on failure */
        return ret;
    }

    pr_info("Servo: Subsystem initialized successfully\n");
    return 0;
}

/**
 * Clean up servo control system
 */
void servo_cleanup(void)
{
    pr_info("Servo: Cleaning up subsystem\n");

    /* Only proceed if initialized */
    if (!servo_drv.initialized)
        return;

    /* Center all servos before shutting down for safety */
    if (servo_center_all() < 0)
        pr_warn("Servo: Failed to center servos during cleanup\n");

    servo_drv.initialized = 0;
    pr_info("Servo: Subsystem cleaned up\n");
}

/**
 * Set servo angle for a specific leg and joint
 *
 * @param leg Leg number (0-5)
 * @param joint Joint number (0=hip, 1=knee, 2=ankle)
 * @param angle Angle in degrees (-90 to +90)
 * @return 0 on success, negative error code on failure
 */
int servo_set_angle(u8 leg, u8 joint, s16 angle)
{
    int ret;
    u8 channel;
    u16 pulse_width;
    static int first_command = 1;

    /* Parameter validation */
    ret = validate_channel(leg, joint);
    if (ret < 0)
        return ret;

    if (!servo_drv.initialized)
    {
        pr_err("Servo: Subsystem not initialized\n");
        return -ENODEV;
    }

    /* Lock for thread safety */
    mutex_lock(&servo_drv.lock);

    /* On first servo command, ensure PWM outputs are enabled */
    if (first_command)
    {
        ret = pca9685_enable_outputs();
        if (ret < 0)
        {
            pr_err("Servo: Failed to enable PWM outputs: %d\n", ret);
            mutex_unlock(&servo_drv.lock);
            return ret;
        }
        first_command = 0;
    }

    /* Get the appropriate channel number */
    channel = servo_map[leg][joint];

    /* Apply controller offset for second PCA9685 */
    if (leg_controller[leg] == 1)
    {
        channel += PCA9685_CHANNELS_PER_DEVICE;
    }

    /* Apply calibration offset */
    angle += servo_drv.offsets[leg][joint];

    /* Convert angle to pulse width */
    pulse_width = angle_to_pulse(angle);

    if (servo_debug)
        pr_info("Servo: Setting leg %u, joint %u (channel %u) to %d° (pulse %uµs)\n",
                leg, joint, channel, angle, pulse_width);

    /* Set the pulse width on PCA9685 */
    ret = pca9685_set_pwm_us(channel, pulse_width);
    if (ret < 0)
    {
        pr_err("Servo: Failed to set PWM for leg %u, joint %u (channel %u): %d\n",
               leg, joint, channel, ret);
    }

    mutex_unlock(&servo_drv.lock);
    return ret;
}

/**
 * Set calibration offsets for a specific leg
 *
 * @param leg Leg number (0-5)
 * @param hip_offset Hip joint offset in degrees
 * @param knee_offset Knee joint offset in degrees
 * @param ankle_offset Ankle joint offset in degrees
 * @return 0 on success, negative error code on failure
 */
int servo_set_calibration(u8 leg, s16 hip_offset, s16 knee_offset, s16 ankle_offset)
{
    if (leg >= SERVO_NUM_LEGS)
        return -EINVAL;

    if (!servo_drv.initialized)
    {
        pr_err("Servo: Subsystem not initialized\n");
        return -ENODEV;
    }

    mutex_lock(&servo_drv.lock);

    /* Store offsets for future use */
    servo_drv.offsets[leg][0] = hip_offset;
    servo_drv.offsets[leg][1] = knee_offset;
    servo_drv.offsets[leg][2] = ankle_offset;

    if (servo_debug)
    {
        pr_info("Servo: Calibration set for leg %u: hip=%d°, knee=%d°, ankle=%d°\n",
                leg, hip_offset, knee_offset, ankle_offset);
    }

    mutex_unlock(&servo_drv.lock);
    return 0;
}

/**
 * Center all servos to their zero positions
 *
 * @return 0 on success, negative error code on failure
 */
int servo_center_all(void)
{
    int leg, joint, ret;
    int errors = 0;

    if (!servo_drv.initialized)
    {
        pr_err("Servo: Subsystem not initialized\n");
        return -ENODEV;
    }

    /* Ensure PWM outputs are enabled before centering */
    ret = pca9685_enable_outputs();
    if (ret < 0)
    {
        pr_err("Servo: Failed to enable PWM outputs: %d\n", ret);
        return ret;
    }

    pr_info("Servo: Centering all servos\n");

    /* Center each servo individually with brief delays between them
     * to avoid current spikes from moving all servos simultaneously */
    for (leg = 0; leg < SERVO_NUM_LEGS; leg++)
    {
        for (joint = 0; joint < SERVO_NUM_JOINTS_PER_LEG; joint++)
        {
            ret = servo_set_angle(leg, joint, 0);
            if (ret < 0)
            {
                pr_err("Servo: Failed to center leg %u, joint %u: %d\n",
                       leg, joint, ret);
                errors++;
            }

            /* Small delay between commands for smoother operation */
            schedule_timeout_interruptible(msecs_to_jiffies(5));
        }
    }

    if (errors)
        return -EIO;

    return 0;
}

/* Export symbols for other kernel modules */
EXPORT_SYMBOL_GPL(servo_init);
EXPORT_SYMBOL_GPL(servo_cleanup);
EXPORT_SYMBOL_GPL(servo_set_angle);
EXPORT_SYMBOL_GPL(servo_set_calibration);
EXPORT_SYMBOL_GPL(servo_center_all);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Hexapod Servo Control Driver");
MODULE_AUTHOR("StrongFood");
MODULE_VERSION("1.0");
