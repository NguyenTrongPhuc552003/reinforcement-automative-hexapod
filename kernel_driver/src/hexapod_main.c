#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include "hexapod_main.h"
#include "i2c_comm.h"
#include "uart_comm.h"
#include "servo.h"
#include "mpu6050.h"
#include "hexapod_ioctl.h"

#define DEVICE_NAME "hexapod"
#define CLASS_NAME "hexapod"

static int major;
static struct class *hexapod_class = NULL;
static struct cdev hexapod_cdev;

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
    struct servo_control servo;
    struct movement_pattern pattern;
    struct mpu6050_data mpu_data;
    struct uart_msg msg;

    switch (cmd) {
        case IOCTL_SET_SERVO:
            if (copy_from_user(&servo, (struct servo_control *)arg, sizeof(servo)))
                return -EFAULT;
            
            ret = servo_set_angle(servo.leg_id, servo.joint_id, servo.angle);
            break;

        case IOCTL_MOVE_LEG:
            if (copy_from_user(&servo, (struct servo_control *)arg, sizeof(servo)))
                return -EFAULT;
            
            ret = servo_move_leg(servo.leg_id, 
                               servo.joint_id == 0 ? servo.angle : -1,
                               servo.joint_id == 1 ? servo.angle : -1,
                               servo.joint_id == 2 ? servo.angle : -1);
            break;

        case IOCTL_SET_PATTERN:
            if (copy_from_user(&pattern, (struct movement_pattern *)arg, sizeof(pattern)))
                return -EFAULT;
            
            // TODO: Implement pattern movement
            ret = -ENOSYS;  // Not implemented yet
            break;

        case IOCTL_GET_MPU6050:
            ret = mpu6050_read_all(&mpu_data.accel_x, &mpu_data.accel_y, &mpu_data.accel_z,
                                 &mpu_data.gyro_x, &mpu_data.gyro_y, &mpu_data.gyro_z,
                                 &mpu_data.temp);
            if (ret < 0)
                return ret;

            if (copy_to_user((struct mpu6050_data *)arg, &mpu_data, sizeof(mpu_data)))
                return -EFAULT;
            break;

        case UART_SEND:
            if (copy_from_user(&msg, (struct uart_msg *)arg, sizeof(msg)))
                return -EFAULT;
            
            ret = uart_write(msg.data, msg.length);
            break;

        case UART_RECEIVE:
            if (copy_from_user(&msg, (struct uart_msg *)arg, sizeof(msg)))
                return -EFAULT;
            
            ret = uart_read(msg.data, msg.length);
            
            if (ret >= 0) {
                if (copy_to_user(((struct uart_msg *)arg)->data, msg.data, ret))
                    return -EFAULT;
            }
            break;

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

static int __init hexapod_init(void)
{
    int ret;
    dev_t dev;

    // Allocate a major number dynamically
    ret = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        printk(KERN_ERR "Failed to allocate char device region\n");
        return ret;
    }
    major = MAJOR(dev);

    // Create device class
    hexapod_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(hexapod_class)) {
        unregister_chrdev_region(MKDEV(major, 0), 1);
        return PTR_ERR(hexapod_class);
    }

    // Create device file
    if (IS_ERR(device_create(hexapod_class, NULL, MKDEV(major, 0), NULL, DEVICE_NAME))) {
        class_destroy(hexapod_class);
        unregister_chrdev_region(MKDEV(major, 0), 1);
        return PTR_ERR(hexapod_class);
    }

    // Initialize character device
    cdev_init(&hexapod_cdev, &hexapod_fops);
    hexapod_cdev.owner = THIS_MODULE;
    ret = cdev_add(&hexapod_cdev, MKDEV(major, 0), 1);
    if (ret < 0) {
        device_destroy(hexapod_class, MKDEV(major, 0));
        class_destroy(hexapod_class);
        unregister_chrdev_region(MKDEV(major, 0), 1);
        return ret;
    }

    // Initialize components
    ret = i2c_init();
    if (ret < 0)
        goto fail_i2c;

    ret = uart_init();
    if (ret < 0)
        goto fail_uart;

    ret = servo_init_all();
    if (ret < 0)
        goto fail_servo;

    ret = mpu6050_init();
    if (ret < 0)
        goto fail_mpu6050;

    printk(KERN_INFO "Hexapod driver initialized\n");
    return 0;

fail_mpu6050:
    servo_cleanup();
fail_servo:
    uart_cleanup();
fail_uart:
    i2c_cleanup();
fail_i2c:
    cdev_del(&hexapod_cdev);
    device_destroy(hexapod_class, MKDEV(major, 0));
    class_destroy(hexapod_class);
    unregister_chrdev_region(MKDEV(major, 0), 1);
    return ret;
}

static void __exit hexapod_exit(void)
{
    mpu6050_cleanup();
    servo_cleanup();
    uart_cleanup();
    i2c_cleanup();
    cdev_del(&hexapod_cdev);
    device_destroy(hexapod_class, MKDEV(major, 0));
    class_destroy(hexapod_class);
    unregister_chrdev_region(MKDEV(major, 0), 1);
    printk(KERN_INFO "Hexapod driver removed\n");
}

module_init(hexapod_init);
module_exit(hexapod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Hexapod Robot Control Driver");
MODULE_VERSION("1.0");
