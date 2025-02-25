#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include "hexapod.h"
#include "servo.h"
#include "mpu6050.h"

/* Device Info */
#define DEVICE_NAME "hexapod"
#define CLASS_NAME "hexapod"

/* Global Variables */
static int major_number;
static struct class *hexapod_class = NULL;
static struct cdev hexapod_cdev;

/* File Operations */
static int hexapod_open(struct inode *inode, struct file *file)
{
    // Opening the hexapod device
    pr_info("hexapod: Device opened\n");
    return 0;
}

static int hexapod_release(struct inode *inode, struct file *file)
{
    // Closing the hexapod device
    pr_info("hexapod: Device closed\n");
    return 0;
}

static long hexapod_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    switch (cmd)
    {
    case SET_LEG_POSITION:
    {
        struct leg_command cmd;
        if (copy_from_user(&cmd, (void __user *)arg, sizeof(cmd)))
            return -EFAULT;

        ret = leg_set_position(cmd.leg_num,
                               cmd.position.hip,
                               cmd.position.knee,
                               cmd.position.ankle);
        break;
    }

    case GET_IMU_DATA:
    {
        struct imu_data data;
        ret = mpu6050_read_all(&data.accel_x, &data.accel_y, &data.accel_z,
                               &data.gyro_x, &data.gyro_y, &data.gyro_z);
        if (ret < 0)
            return ret;

        if (copy_to_user((void __user *)arg, &data, sizeof(data)))
            return -EFAULT;
        break;
    }

    default:
        return -ENOTTY;
    }

    return ret;
}

/* File Operations Structure */
static const struct file_operations hexapod_fops = {
    .owner = THIS_MODULE,
    .open = hexapod_open,
    .release = hexapod_release,
    .unlocked_ioctl = hexapod_ioctl,
};

/* Module Init */
static int __init hexapod_init(void)
{
    int ret;
    dev_t dev;

    /* Initialize MPU6050 */
    ret = mpu6050_init();
    if (ret < 0)
    {
        pr_err("Failed to initialize MPU6050\n");
        goto cleanup_mpu6050;
    }

    /* Initialize Servo System */
    ret = servo_init();
    if (ret < 0)
    {
        pr_err("Failed to initialize servo system\n");
        goto cleanup_servo;
    }

    /* Allocate character device region */
    ret = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
    if (ret < 0)
    {
        pr_err("Failed to allocate character device region\n");
        goto cleanup_servo;
    }
    major_number = MAJOR(dev);

    /* Create device class */
    hexapod_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(hexapod_class))
    {
        pr_err("Failed to create device class\n");
        ret = PTR_ERR(hexapod_class);
        goto cleanup_chrdev;
    }

    /* Create device */
    if (IS_ERR(device_create(hexapod_class, NULL, dev, NULL, DEVICE_NAME)))
    {
        pr_err("Failed to create device\n");
        ret = PTR_ERR(hexapod_class);
        goto cleanup_class;
    }

    /* Initialize character device */
    cdev_init(&hexapod_cdev, &hexapod_fops);
    hexapod_cdev.owner = THIS_MODULE;
    ret = cdev_add(&hexapod_cdev, dev, 1);
    if (ret < 0)
    {
        pr_err("Failed to add character device\n");
        goto cleanup_device;
    }

    pr_info("Hexapod driver initialized\n");
    return 0;

cleanup_device:
    device_destroy(hexapod_class, dev);
cleanup_class:
    class_destroy(hexapod_class);
cleanup_chrdev:
    unregister_chrdev_region(dev, 1);
cleanup_servo:
    servo_cleanup();
cleanup_mpu6050:
    mpu6050_cleanup();
    return ret;
}

/* Module Exit */
static void __exit hexapod_exit(void)
{
    dev_t dev = MKDEV(major_number, 0);

    cdev_del(&hexapod_cdev);
    device_destroy(hexapod_class, dev);
    class_destroy(hexapod_class);
    unregister_chrdev_region(dev, 1);
    servo_cleanup();
    mpu6050_cleanup();
    pr_info("Hexapod driver removed\n");
}

/* Module dependencies */
MODULE_SOFTDEP("pre: i2c_bcm2835");
MODULE_SOFTDEP("pre: i2c_dev");

module_init(hexapod_init);
module_exit(hexapod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("StrongFood");
MODULE_DESCRIPTION("Hexapod Robot Driver");
MODULE_VERSION("1.0");
