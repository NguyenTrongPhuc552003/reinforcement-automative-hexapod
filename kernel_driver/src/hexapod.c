#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include "hexapod.h"
#include "mpu6050.h"
#include "servo.h"
#include "pca9685.h"

#define DRIVER_NAME "hexapod"

/* Global variables */
static struct hexapod_dev hexapod_device;

/* IOCTL handler */
static long hexapod_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int ret = 0;

    mutex_lock(&hexapod_device.lock);

    switch (cmd)
    {
    case SET_LEG_POSITION:
    {
        struct leg_command command;
        if (copy_from_user(&command, argp, sizeof(command)))
        {
            ret = -EFAULT;
            goto out;
        }
        ret = hexapod_set_leg_position(command.leg_num, &command.position);
        break;
    }

    case GET_IMU_DATA:
    {
        struct imu_data data;
        ret = hexapod_get_imu_data(&data);
        if (ret == 0)
        {
            if (copy_to_user(argp, &data, sizeof(data)))
                ret = -EFAULT;
        }
        break;
    }

    default:
        ret = -ENOTTY;
    }

out:
    mutex_unlock(&hexapod_device.lock);
    return ret;
}

/* File operations */
static const struct file_operations hexapod_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = hexapod_ioctl,
    .open = nonseekable_open,
    .llseek = no_llseek,
};

/* Misc device structure */
static struct miscdevice hexapod_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DRIVER_NAME,
    .fops = &hexapod_fops,
};

/* Init/Exit functions */
static int __init hexapod_init(void)
{
    int ret;
    struct i2c_adapter *adapter;

    // Initialize mutex
    mutex_init(&hexapod_device.lock);

    // Register misc device
    ret = misc_register(&hexapod_miscdev);
    if (ret)
    {
        pr_err("Failed to register misc device: %d\n", ret);
        return ret;
    }

    // Get I2C adapter
    adapter = i2c_get_adapter(HEXAPOD_I2C_BUS);
    if (!adapter)
    {
        pr_err("Failed to get I2C adapter %d\n", HEXAPOD_I2C_BUS);
        ret = -ENODEV;
        goto fail_i2c;
    }

    // Initialize MPU6050
    ret = mpu6050_probe(adapter);
    if (ret < 0)
    {
        pr_err("Failed to initialize MPU6050: %d\n", ret);
        goto fail_mpu;
    }

    // Initialize servo subsystem
    ret = servo_init(adapter);
    if (ret < 0)
    {
        pr_err("Failed to initialize servo subsystem: %d\n", ret);
        goto fail_servo;
    }

    i2c_put_adapter(adapter);
    pr_info("Hexapod driver initialized successfully\n");
    return 0;

fail_servo:
    mpu6050_shutdown();
fail_mpu:
    i2c_put_adapter(adapter);
fail_i2c:
    misc_deregister(&hexapod_miscdev);
    return ret;
}

static void __exit hexapod_exit(void)
{
    servo_cleanup();
    mpu6050_shutdown();
    misc_deregister(&hexapod_miscdev);
    pr_info("Hexapod driver removed\n");
}

module_init(hexapod_init);
module_exit(hexapod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Hexapod Robot Control Driver");
MODULE_VERSION("1.0");
