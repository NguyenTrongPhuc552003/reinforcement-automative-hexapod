#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include "mpu6050.h"
#include "pca9685.h"
#include "servo.h"

#define HEXAPOD_NAME "hexapod"
#define HEXAPOD_CLASS "hexapod"

/* Per-device structure */
struct hexapod_dev {
    struct cdev cdev;
    struct class *class;
    dev_t devt;
    struct device *device;
    struct i2c_client *mpu6050_client;
    struct mutex lock;
};

static struct hexapod_dev *hexapod_device;

/* File operations */
static int hexapod_open(struct inode *inode, struct file *file)
{
    struct hexapod_dev *dev = container_of(inode->i_cdev, struct hexapod_dev, cdev);
    file->private_data = dev;
    return 0;
}

static int hexapod_release(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t hexapod_read(struct file *file, char __user *buf,
                           size_t count, loff_t *offset)
{
    struct hexapod_dev *dev = file->private_data;
    s16 accel_x, accel_y, accel_z;
    s16 gyro_x, gyro_y, gyro_z;
    s16 temp;
    int ret;
    char data[64];
    int len;

    mutex_lock(&dev->lock);
    ret = mpu6050_read_all(&accel_x, &accel_y, &accel_z,
                          &gyro_x, &gyro_y, &gyro_z,
                          &temp);
    mutex_unlock(&dev->lock);

    if (ret < 0)
        return ret;

    len = snprintf(data, sizeof(data),
                  "ax:%d ay:%d az:%d t:%d gx:%d gy:%d gz:%d\n",
                  accel_x, accel_y, accel_z, temp,
                  gyro_x, gyro_y, gyro_z);

    if (*offset >= len)
        return 0;

    if (count > len - *offset)
        count = len - *offset;

    if (copy_to_user(buf, data + *offset, count))
        return -EFAULT;

    *offset += count;
    return count;
}

static const struct file_operations hexapod_fops = {
    .owner = THIS_MODULE,
    .open = hexapod_open,
    .release = hexapod_release,
    .read = hexapod_read,
};

/* Platform driver probe */
static int hexapod_probe(struct platform_device *pdev)
{
    int ret;
    struct hexapod_dev *dev;

    dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;

    mutex_init(&dev->lock);

    /* Create character device */
    ret = alloc_chrdev_region(&dev->devt, 0, 1, HEXAPOD_NAME);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to allocate device number\n");
        return ret;
    }

    cdev_init(&dev->cdev, &hexapod_fops);
    dev->cdev.owner = THIS_MODULE;
    ret = cdev_add(&dev->cdev, dev->devt, 1);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to add character device\n");
        goto err_cdev;
    }

    /* Create sysfs class */
    dev->class = class_create(THIS_MODULE, HEXAPOD_NAME);
    if (IS_ERR(dev->class)) {
        ret = PTR_ERR(dev->class);
        dev_err(&pdev->dev, "Failed to create device class\n");
        goto err_class;
    }

    /* Create sysfs device */
    dev->device = device_create(dev->class, NULL, dev->devt, NULL,
                              HEXAPOD_NAME);
    if (IS_ERR(dev->device)) {
        ret = PTR_ERR(dev->device);
        dev_err(&pdev->dev, "Failed to create device\n");
        goto err_device;
    }

    platform_set_drvdata(pdev, dev);
    hexapod_device = dev;

    /* Initialize MPU6050 */
    ret = mpu6050_init();
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to initialize MPU6050\n");
        goto err_mpu;
    }

    dev_info(&pdev->dev, "Hexapod driver initialized\n");
    return 0;

err_mpu:
    device_destroy(dev->class, dev->devt);
err_device:
    class_destroy(dev->class);
err_class:
    cdev_del(&dev->cdev);
err_cdev:
    unregister_chrdev_region(dev->devt, 1);
    return ret;
}

/* Platform driver remove */
static int hexapod_remove(struct platform_device *pdev)
{
    struct hexapod_dev *dev = platform_get_drvdata(pdev);

    mpu6050_cleanup();
    device_destroy(dev->class, dev->devt);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devt, 1);
    mutex_destroy(&dev->lock);

    return 0;
}

/* Platform driver structure */
static struct platform_driver hexapod_driver = {
    .probe = hexapod_probe,
    .remove = hexapod_remove,
    .driver = {
        .name = HEXAPOD_NAME,
        .owner = THIS_MODULE,
    },
};

/* Module initialization */
static int __init hexapod_init(void)
{
    int ret;

    ret = platform_driver_register(&hexapod_driver);
    if (ret < 0) {
        pr_err("Failed to register platform driver\n");
        return ret;
    }

    return 0;
}

/* Module cleanup */
static void __exit hexapod_exit(void)
{
    platform_driver_unregister(&hexapod_driver);
}

module_init(hexapod_init);
module_exit(hexapod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Hexapod Robot Driver");
MODULE_VERSION("1.0");
