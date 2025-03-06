#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include "hexapod.h"
#include "servo.h"
#include "mpu6050.h"
#include "pca9685.h"

/* Global device data */
struct hexapod_data hexapod_dev;
static struct miscdevice hexapod_miscdev;

/* Set leg position */
int hexapod_set_leg_position(struct hexapod_data *dev, u8 leg, struct hexapod_leg_position *pos)
{
    if (!dev || !dev->initialized || leg >= NUM_LEGS || !pos)
        return -EINVAL;

    /* Check angle limits */
    if (pos->hip < MIN_ANGLE || pos->hip > MAX_ANGLE ||
        pos->knee < MIN_ANGLE || pos->knee > MAX_ANGLE ||
        pos->ankle < MIN_ANGLE || pos->ankle > MAX_ANGLE)
        return -EINVAL;

    /* Set angles */
    mutex_lock(&dev->lock);

    /* Store the position */
    memcpy(&dev->positions[leg], pos, sizeof(struct hexapod_leg_position));

    /* Update servos */
    servo_set_angle(leg, 0, pos->hip);
    servo_set_angle(leg, 1, pos->knee);
    servo_set_angle(leg, 2, pos->ankle);

    mutex_unlock(&dev->lock);
    return 0;
}

/* Get IMU data */
int hexapod_get_imu_data(struct hexapod_data *dev, struct hexapod_imu_data *data)
{
    int ret;

    if (!dev || !dev->initialized || !data)
        return -EINVAL;

    mutex_lock(&dev->lock);
    ret = mpu6050_read_sensors(dev->mpu6050, data);
    mutex_unlock(&dev->lock);

    return ret;
}

/* Set calibration values */
int hexapod_set_calibration(struct hexapod_data *dev, u8 leg, struct hexapod_calibration *cal)
{
    if (!dev || !dev->initialized || leg >= NUM_LEGS || !cal)
        return -EINVAL;

    mutex_lock(&dev->lock);

    /* Store calibration */
    dev->calibration[leg] = *cal;

    /* Apply to servo subsystem */
    servo_set_calibration(leg, cal->hip_offset,
                          cal->knee_offset, cal->ankle_offset);

    mutex_unlock(&dev->lock);
    return 0;
}

/* Center all servos */
int hexapod_center_all(struct hexapod_data *dev)
{
    int i;
    struct hexapod_leg_position center = {0, 0, 0};

    if (!dev || !dev->initialized)
        return -EINVAL;

    mutex_lock(&dev->lock);

    for (i = 0; i < NUM_LEGS; i++)
    {
        dev->positions[i] = center;
    }

    /* Use servo subsystem to center all servos */
    servo_center_all();

    mutex_unlock(&dev->lock);
    return 0;
}

/* File operations */
static long hexapod_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    void __user *argp = (void __user *)arg;

    if (_IOC_TYPE(cmd) != HEXAPOD_IOC_MAGIC)
        return -ENOTTY;
    if (_IOC_NR(cmd) > HEXAPOD_IOCTL_MAX)
        return -ENOTTY;

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

static int hexapod_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int hexapod_release(struct inode *inode, struct file *file)
{
    return 0;
}

static const struct file_operations hexapod_fops = {
    .owner = THIS_MODULE,
    .open = hexapod_open,
    .release = hexapod_release,
    .unlocked_ioctl = hexapod_ioctl,
    .llseek = no_llseek,
};

/* Initialize module */
static int __init hexapod_init(void)
{
    int ret;
    struct i2c_adapter *adapter;

    pr_info("Hexapod: initializing driver\n");

    /* Initialize the data structure */
    memset(&hexapod_dev, 0, sizeof(hexapod_dev));
    mutex_init(&hexapod_dev.lock);

    /* Register the misc device */
    hexapod_miscdev.minor = MISC_DYNAMIC_MINOR;
    hexapod_miscdev.name = "hexapod";
    hexapod_miscdev.fops = &hexapod_fops;

    ret = misc_register(&hexapod_miscdev);
    if (ret)
    {
        pr_err("Hexapod: failed to register misc device\n");
        return ret;
    }

    /* Get I2C adapter */
    adapter = i2c_get_adapter(HEXAPOD_I2C_BUS);
    if (!adapter)
    {
        pr_err("Hexapod: failed to get I2C adapter\n");
        ret = -ENODEV;
        goto fail_i2c;
    }

    /* Initialize MPU6050 */
    hexapod_dev.mpu6050 = i2c_new_dummy(adapter, MPU6050_I2C_ADDR);
    if (!hexapod_dev.mpu6050)
    {
        pr_err("Hexapod: failed to create MPU6050 I2C client\n");
        ret = -ENOMEM;
        goto fail_mpu;
    }

    ret = mpu6050_init(hexapod_dev.mpu6050);
    if (ret)
    {
        pr_err("Hexapod: failed to initialize MPU6050: %d\n", ret);
        goto fail_mpu_init;
    }

    /* Initialize PCA9685 */
    ret = pca9685_init();
    if (ret)
    {
        pr_err("Hexapod: failed to initialize PCA9685: %d\n", ret);
        goto fail_pca;
    }

    /* Initialize servo system */
    ret = servo_init();
    if (ret)
    {
        pr_err("Hexapod: failed to initialize servo system: %d\n", ret);
        goto fail_servo;
    }

    i2c_put_adapter(adapter);
    hexapod_dev.initialized = true;
    pr_info("Hexapod: driver initialized successfully\n");
    return 0;

fail_servo:
    pca9685_cleanup();
fail_pca:
    mpu6050_remove(hexapod_dev.mpu6050);
fail_mpu_init:
    i2c_unregister_device(hexapod_dev.mpu6050);
fail_mpu:
    i2c_put_adapter(adapter);
fail_i2c:
    misc_deregister(&hexapod_miscdev);
    return ret;
}

static void __exit hexapod_cleanup(void)
{
    pr_info("Hexapod: shutting down driver\n");

    if (hexapod_dev.initialized)
    {
        /* Center all servos for safety */
        hexapod_center_all(&hexapod_dev);

        /* Clean up subsystems */
        servo_cleanup();
        pca9685_cleanup();

        if (hexapod_dev.mpu6050)
        {
            mpu6050_remove(hexapod_dev.mpu6050);
            i2c_unregister_device(hexapod_dev.mpu6050);
        }

        /* Unregister the device */
        misc_deregister(&hexapod_miscdev);
    }
}

module_init(hexapod_init);
module_exit(hexapod_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("StrongFood");
MODULE_DESCRIPTION("Hexapod Robot Control Driver");
MODULE_VERSION("1.0");
