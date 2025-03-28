#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include "hexapod.h"
#include "servo.h"
#include "mpu6050.h"
#include "pca9685.h"

/* Module parameters */
static int debug = 0;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Enable debug messages (0=disable, 1=enable)");

/* Global device data */
struct hexapod_data hexapod_dev;
static struct miscdevice hexapod_miscdev;

/* Debug logging macro */
#define hexapod_dbg(fmt, ...)                        \
    do                                               \
    {                                                \
        if (debug)                                   \
            pr_info("Hexapod: " fmt, ##__VA_ARGS__); \
    } while (0)

/**
 * @brief Validate a leg number is within range
 *
 * @param leg Leg number to check
 * @return 0 if valid, -EINVAL if invalid
 */
static inline int validate_leg_number(u8 leg)
{
    return (leg < NUM_LEGS) ? 0 : -EINVAL;
}

/**
 * @brief Validate joint angles are within permitted ranges
 *
 * @param pos Pointer to position structure
 * @return 0 if valid, -EINVAL if invalid
 */
static inline int validate_joint_angles(const struct hexapod_leg_position *pos)
{
    return (pos &&
            pos->hip >= SERVO_MIN_ANGLE && pos->hip <= SERVO_MAX_ANGLE &&
            pos->knee >= SERVO_MIN_ANGLE && pos->knee <= SERVO_MAX_ANGLE &&
            pos->ankle >= SERVO_MIN_ANGLE && pos->ankle <= SERVO_MAX_ANGLE)
               ? 0
               : -EINVAL;
}

/**
 * Set leg position for a specific leg
 */
int hexapod_set_leg_position(struct hexapod_data *dev, u8 leg, struct hexapod_leg_position *pos)
{
    int ret;

    if (!dev || !dev->initialized)
        return -EINVAL;

    ret = validate_leg_number(leg);
    if (ret != 0)
        return ret;

    ret = validate_joint_angles(pos);
    if (ret != 0)
        return ret;

    mutex_lock(&dev->lock);

    /* Store the position */
    memcpy(&dev->positions[leg], pos, sizeof(struct hexapod_leg_position));

    /* Update servos - send commands to hardware */
    ret = servo_set_angle(leg, 0, pos->hip);
    if (ret < 0)
    {
        pr_err("Hexapod: Failed to set hip angle for leg %d: %d\n", leg, ret);
        goto unlock;
    }

    ret = servo_set_angle(leg, 1, pos->knee);
    if (ret < 0)
    {
        pr_err("Hexapod: Failed to set knee angle for leg %d: %d\n", leg, ret);
        goto unlock;
    }

    ret = servo_set_angle(leg, 2, pos->ankle);
    if (ret < 0)
    {
        pr_err("Hexapod: Failed to set ankle angle for leg %d: %d\n", leg, ret);
        goto unlock;
    }

    hexapod_dbg("Set leg %d to position: hip=%d, knee=%d, ankle=%d\n",
                leg, pos->hip, pos->knee, pos->ankle);

unlock:
    mutex_unlock(&dev->lock);
    return ret;
}
EXPORT_SYMBOL_GPL(hexapod_set_leg_position);

/**
 * Get IMU data from sensor
 */
int hexapod_get_imu_data(struct hexapod_data *dev, struct hexapod_imu_data *data)
{
    int ret;

    if (!dev || !dev->initialized || !data)
        return -EINVAL;

    mutex_lock(&dev->lock);
    ret = mpu6050_read_sensors((struct mpu6050_imu_data *)data);
    if (ret < 0)
    {
        pr_err("Hexapod: Failed to read IMU data: %d\n", ret);
    }
    else
    {
        hexapod_dbg("IMU data read successfully\n");
    }
    mutex_unlock(&dev->lock);

    return ret;
}
EXPORT_SYMBOL_GPL(hexapod_get_imu_data);

/**
 * Set calibration values for a leg
 */
int hexapod_set_calibration(struct hexapod_data *dev, u8 leg, struct hexapod_calibration *cal)
{
    int ret = 0;

    if (!dev || !dev->initialized || !cal)
        return -EINVAL;

    ret = validate_leg_number(leg);
    if (ret != 0)
        return ret;

    mutex_lock(&dev->lock);

    /* Store calibration */
    dev->calibration[leg] = *cal;

    /* Apply to servo subsystem */
    ret = servo_set_calibration(leg, cal->hip_offset,
                                cal->knee_offset, cal->ankle_offset);
    if (ret < 0)
    {
        pr_err("Hexapod: Failed to set calibration for leg %d: %d\n", leg, ret);
    }
    else
    {
        hexapod_dbg("Calibration set for leg %d: hip=%d, knee=%d, ankle=%d\n",
                    leg, cal->hip_offset, cal->knee_offset, cal->ankle_offset);
    }

    mutex_unlock(&dev->lock);
    return ret;
}
EXPORT_SYMBOL_GPL(hexapod_set_calibration);

/**
 * Center all servos to neutral position
 */
int hexapod_center_all(struct hexapod_data *dev)
{
    int i, ret;
    struct hexapod_leg_position center = {0, 0, 0};

    if (!dev || !dev->initialized)
        return -EINVAL;

    mutex_lock(&dev->lock);

    /* Update stored positions to centered */
    for (i = 0; i < NUM_LEGS; i++)
    {
        dev->positions[i] = center;
    }

    /* Use servo subsystem to center all servos in one operation */
    ret = servo_center_all();
    if (ret < 0)
    {
        pr_err("Hexapod: Failed to center all servos: %d\n", ret);
    }
    else
    {
        hexapod_dbg("All servos centered successfully\n");
    }

    mutex_unlock(&dev->lock);
    return ret;
}
EXPORT_SYMBOL_GPL(hexapod_center_all);

/**
 * Process IOCTL commands
 */
static long hexapod_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    void __user *argp = (void __user *)arg;

    /* Validate IOCTL command */
    if (_IOC_TYPE(cmd) != HEXAPOD_IOC_MAGIC)
        return -ENOTTY;
    if (_IOC_NR(cmd) > HEXAPOD_IOCTL_MAX)
        return -ENOTTY;

    /* Process based on command code */
    switch (cmd)
    {
    case HEXAPOD_IOCTL_SET_LEG:
    {
        struct hexapod_leg_cmd leg_cmd;

        if (copy_from_user(&leg_cmd, argp, sizeof(leg_cmd)))
            return -EFAULT;

        ret = hexapod_set_leg_position(&hexapod_dev, leg_cmd.leg_num, &leg_cmd.position);
        break;
    }

    case HEXAPOD_IOCTL_GET_IMU:
    {
        struct hexapod_imu_data imu_data;

        ret = hexapod_get_imu_data(&hexapod_dev, &imu_data);
        if (ret == 0)
        {
            if (copy_to_user(argp, &imu_data, sizeof(imu_data)))
                ret = -EFAULT;
        }
        break;
    }

    case HEXAPOD_IOCTL_CALIBRATE:
    {
        struct hexapod_calibration calib;

        if (copy_from_user(&calib, argp, sizeof(calib)))
            return -EFAULT;

        ret = hexapod_set_calibration(&hexapod_dev, calib.leg_num, &calib);
        break;
    }

    case HEXAPOD_IOCTL_CENTER_ALL:
        ret = hexapod_center_all(&hexapod_dev);
        break;

    default:
        ret = -ENOTTY;
    }

    return ret;
}

/**
 * Device open operation
 */
static int hexapod_open(struct inode *inode, struct file *file)
{
    if (!hexapod_dev.initialized)
    {
        pr_err("Hexapod: Device not initialized\n");
        return -ENODEV;
    }

    hexapod_dbg("Device opened\n");
    return 0;
}

/**
 * Device release operation
 */
static int hexapod_release(struct inode *inode, struct file *file)
{
    hexapod_dbg("Device closed\n");
    return 0;
}

/* File operations structure */
static const struct file_operations hexapod_fops = {
    .owner = THIS_MODULE,
    .open = hexapod_open,
    .release = hexapod_release,
    .unlocked_ioctl = hexapod_ioctl,
    .llseek = no_llseek,
};

/**
 * Initialize module components in order of dependency
 */
static int __init hexapod_init(void)
{
    int ret;

    pr_info("Hexapod: Initializing driver\n");

    /* Initialize data structure */
    memset(&hexapod_dev, 0, sizeof(hexapod_dev));
    mutex_init(&hexapod_dev.lock);

    /* Register the misc device */
    hexapod_miscdev.minor = MISC_DYNAMIC_MINOR;
    hexapod_miscdev.name = "hexapod";
    hexapod_miscdev.fops = &hexapod_fops;
    hexapod_miscdev.mode = 0666;

    ret = misc_register(&hexapod_miscdev);
    if (ret)
    {
        pr_err("Hexapod: Failed to register misc device: %d\n", ret);
        return ret;
    }

    /* Initialize MPU6050 IMU sensor */
    ret = mpu6050_init();
    if (ret)
    {
        pr_err("Hexapod: Failed to initialize MPU6050: %d\n", ret);
        goto fail_mpu;
    }

    /* Initialize PCA9685 PWM controller */
    ret = pca9685_init();
    if (ret)
    {
        pr_err("Hexapod: Failed to initialize PCA9685: %d\n", ret);
        goto fail_pca;
    }

    /* Initialize servo system - depends on PCA9685 */
    ret = servo_init();
    if (ret)
    {
        pr_err("Hexapod: Failed to initialize servo system: %d\n", ret);
        goto fail_servo;
    }

    hexapod_dev.initialized = 1;
    pr_info("Hexapod: Driver initialized successfully\n");
    return 0;

fail_servo:
    pca9685_cleanup();
fail_pca:
    mpu6050_cleanup();
fail_mpu:
    misc_deregister(&hexapod_miscdev);
    return ret;
}

/**
 * Clean up module resources in reverse order of initialization
 */
static void __exit hexapod_cleanup(void)
{
    pr_info("Hexapod: Shutting down driver\n");

    if (hexapod_dev.initialized)
    {
        /* Center all servos for safety before shutdown */
        hexapod_center_all(&hexapod_dev);

        /* Clean up subsystems in reverse order */
        servo_cleanup();
        pca9685_cleanup();
        mpu6050_cleanup();

        /* Unregister the device */
        misc_deregister(&hexapod_miscdev);
    }
}

/* Register init and exit functions */
module_init(hexapod_init);
module_exit(hexapod_cleanup);

/* Module metadata */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("StrongFood");
MODULE_DESCRIPTION("Hexapod Robot Control Driver");
MODULE_VERSION("1.0");
