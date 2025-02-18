#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#define DEVICE_NAME "hexapod"
#define CLASS_NAME "hexapod_class"
#define IOCTL_SET_MODE _IOW('H', 1, int)
#define IOCTL_GET_STATUS _IOR('H', 2, int)

static int majorNumber;
static struct class *hexapodClass = NULL;
static struct device *hexapodDevice = NULL;
static struct cdev hexapod_cdev;
static DEFINE_MUTEX(hexapod_mutex);

static int hexapod_mode = 0;  // 0: Idle, 1: Walking, 2: Turning

// Prototypes
static int hexapod_open(struct inode *inodep, struct file *filep);
static int hexapod_release(struct inode *inodep, struct file *filep);
static ssize_t hexapod_read(struct file *filep, char *buffer, size_t len, loff_t *offset);
static ssize_t hexapod_write(struct file *filep, const char *buffer, size_t len, loff_t *offset);
static long hexapod_ioctl(struct file *filep, unsigned int cmd, unsigned long arg);

// File operations structure
static struct file_operations fops = {
    .open = hexapod_open,
    .read = hexapod_read,
    .write = hexapod_write,
    .release = hexapod_release,
    .unlocked_ioctl = hexapod_ioctl,
};

// Open device
static int hexapod_open(struct inode *inodep, struct file *filep) {
    if (!mutex_trylock(&hexapod_mutex)) {
        printk(KERN_ALERT "Hexapod: Device in use by another process\n");
        return -EBUSY;
    }
    printk(KERN_INFO "Hexapod: Device opened\n");
    return 0;
}

// Read from device
static ssize_t hexapod_read(struct file *filep, char *buffer, size_t len, loff_t *offset) {
    int status = hexapod_mode;
    int ret = copy_to_user(buffer, &status, sizeof(status));
    if (ret == 0) {
        printk(KERN_INFO "Hexapod: Sent mode status to user\n");
        return sizeof(status);
    }
    return -EFAULT;
}

// Write to device (not commonly used in char drivers)
static ssize_t hexapod_write(struct file *filep, const char *buffer, size_t len, loff_t *offset) {
    int new_mode;
    if (copy_from_user(&new_mode, buffer, sizeof(new_mode))) {
        return -EFAULT;
    }
    hexapod_mode = new_mode;
    printk(KERN_INFO "Hexapod: Mode changed to %d\n", hexapod_mode);
    return sizeof(new_mode);
}

// ioctl for control
static long hexapod_ioctl(struct file *filep, unsigned int cmd, unsigned long arg) {
    int mode;
    switch (cmd) {
        case IOCTL_SET_MODE:
            if (copy_from_user(&mode, (int __user *)arg, sizeof(mode)))
                return -EFAULT;
            hexapod_mode = mode;
            printk(KERN_INFO "Hexapod: Mode set to %d\n", hexapod_mode);
            break;

        case IOCTL_GET_STATUS:
            if (copy_to_user((int __user *)arg, &hexapod_mode, sizeof(hexapod_mode)))
                return -EFAULT;
            printk(KERN_INFO "Hexapod: Status requested, current mode %d\n", hexapod_mode);
            break;

        default:
            return -EINVAL;
    }
    return 0;
}

// Release device
static int hexapod_release(struct inode *inodep, struct file *filep) {
    mutex_unlock(&hexapod_mutex);
    printk(KERN_INFO "Hexapod: Device closed\n");
    return 0;
}

// Initialize module
static int __init hexapod_init(void) {
    printk(KERN_INFO "Hexapod: Initializing...\n");

    // Allocate major number
    majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
    if (majorNumber < 0) {
        printk(KERN_ALERT "Hexapod: Failed to register major number\n");
        return majorNumber;
    }
    printk(KERN_INFO "Hexapod: Registered with major number %d\n", majorNumber);

    // Create class
    hexapodClass = class_create(CLASS_NAME);
    if (IS_ERR(hexapodClass)) {
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Hexapod: Failed to create class\n");
        return PTR_ERR(hexapodClass);
    }

    // Create device
    hexapodDevice = device_create(hexapodClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
    if (IS_ERR(hexapodDevice)) {
        class_destroy(hexapodClass);
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Hexapod: Failed to create device\n");
        return PTR_ERR(hexapodDevice);
    }

    // Initialize character device
    cdev_init(&hexapod_cdev, &fops);
    cdev_add(&hexapod_cdev, MKDEV(majorNumber, 0), 1);

    mutex_init(&hexapod_mutex);
    printk(KERN_INFO "Hexapod: Initialized successfully\n");
    return 0;
}

// Cleanup module
static void __exit hexapod_exit(void) {
    mutex_destroy(&hexapod_mutex);
    device_destroy(hexapodClass, MKDEV(majorNumber, 0));
    class_unregister(hexapodClass);
    class_destroy(hexapodClass);
    unregister_chrdev(majorNumber, DEVICE_NAME);
    printk(KERN_INFO "Hexapod: Module removed\n");
}

module_init(hexapod_init);
module_exit(hexapod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("StrongFood");
MODULE_DESCRIPTION("Hexapod Kernel Module for Motion Control");
MODULE_VERSION("1.0");

