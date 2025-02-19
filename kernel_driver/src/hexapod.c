#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include "hexapod.h"
#include "hexapod_ioctl.h"
#include "servo.h"
#include "mpu6050.h"

// Device variables
static dev_t dev;
static struct cdev cdev;
static struct class *device_class;

// File operations
static int hexapod_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int hexapod_release(struct inode *inode, struct file *file)
{
    return 0;
}

static long hexapod_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    switch (cmd) {
        case HEXAPOD_IOC_SET_SERVO: {
            struct servo_control ctrl;
            if (copy_from_user(&ctrl, (void *)arg, sizeof(ctrl)))
                return -EFAULT;

            // Validate inputs
            if (ctrl.leg_id >= NUM_LEGS || ctrl.joint_id >= NUM_JOINTS_PER_LEG)
                return -EINVAL;

            ret = servo_set_angle(ctrl.leg_id, ctrl.joint_id, ctrl.angle);
            break;
        }

        case HEXAPOD_IOC_GET_MPU6050: {
            struct mpu6050_data data;
            s16 values[7];  // Array to hold aligned values

            // Read values into properly aligned array
            ret = mpu6050_read_all(&values[0], &values[1], &values[2],
                                  &values[3], &values[4], &values[5],
                                  &values[6]);
            if (ret < 0)
                return ret;

            // Copy values to packed structure
            data.accel_x = values[0];
            data.accel_y = values[1];
            data.accel_z = values[2];
            data.gyro_x = values[3];
            data.gyro_y = values[4];
            data.gyro_z = values[5];
            data.temp = values[6];

            if (copy_to_user((void *)arg, &data, sizeof(data)))
                return -EFAULT;
            break;
        }

        case HEXAPOD_IOC_SET_MOVEMENT: {
            struct movement_control ctrl;
            if (copy_from_user(&ctrl, (void *)arg, sizeof(ctrl)))
                return -EFAULT;

            // Validate inputs
            if (ctrl.speed > 1000)
                return -EINVAL;

            // Implement movement patterns here
            switch (ctrl.type) {
                case MOVE_FORWARD:
                    // TODO: Implement forward movement
                    break;
                case MOVE_BACKWARD:
                    // TODO: Implement backward movement
                    break;
                case TURN_LEFT:
                    // TODO: Implement left turn
                    break;
                case TURN_RIGHT:
                    // TODO: Implement right turn
                    break;
                case MOVE_UP:
                    // TODO: Implement body up movement
                    break;
                case MOVE_DOWN:
                    // TODO: Implement body down movement
                    break;
                case MOVE_STOP:
                    // TODO: Implement stop movement
                    break;
                default:
                    return -EINVAL;
            }
            break;
        }

        default:
            return -ENOTTY;
    }

    return ret;
}

static struct file_operations hexapod_fops = {
    .owner = THIS_MODULE,
    .open = hexapod_open,
    .release = hexapod_release,
    .unlocked_ioctl = hexapod_ioctl,
};

// Module init function
static int __init __attribute__((cold)) hexapod_init(void)
{
    int ret;

    // Allocate device number
    ret = alloc_chrdev_region(&dev, 0, 1, HEXAPOD_DEV_NAME);
    if (ret < 0) {
        pr_err("Failed to allocate device number\n");
        return ret;
    }

    // Initialize character device
    cdev_init(&cdev, &hexapod_fops);
    cdev.owner = THIS_MODULE;
    ret = cdev_add(&cdev, dev, 1);
    if (ret < 0) {
        pr_err("Failed to add character device\n");
        goto fail_cdev;
    }

    // Create device class
    device_class = class_create(THIS_MODULE, HEXAPOD_CLASS_NAME);
    if (IS_ERR(device_class)) {
        pr_err("Failed to create device class\n");
        ret = PTR_ERR(device_class);
        goto fail_class;
    }

    // Create device file
    if (IS_ERR(device_create(device_class, NULL, dev, NULL, HEXAPOD_DEV_NAME))) {
        pr_err("Failed to create device file\n");
        ret = PTR_ERR(device_class);
        goto fail_device;
    }

    // Initialize MPU6050
    ret = mpu6050_init();
    if (ret < 0) {
        pr_err("Failed to initialize MPU6050\n");
        goto fail_mpu6050;
    }

    // Initialize servo controller
    ret = servo_init_all();
    if (ret < 0) {
        pr_err("Failed to initialize servo controller\n");
        goto fail_servo;
    }

    pr_info("Hexapod driver initialized\n");
    return 0;

fail_servo:
    mpu6050_cleanup();
fail_mpu6050:
    device_destroy(device_class, dev);
fail_device:
    class_destroy(device_class);
fail_class:
    cdev_del(&cdev);
fail_cdev:
    unregister_chrdev_region(dev, 1);
    return ret;
}

// Module cleanup function
static void __exit __attribute__((cold)) hexapod_cleanup(void)
{
    servo_cleanup();
    mpu6050_cleanup();
    device_destroy(device_class, dev);
    class_destroy(device_class);
    cdev_del(&cdev);
    unregister_chrdev_region(dev, 1);
    pr_info("Hexapod driver removed\n");
}

module_init(hexapod_init);
module_exit(hexapod_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Trong Phuc");
MODULE_DESCRIPTION("Hexapod Robot Driver");
MODULE_VERSION("1.0");
