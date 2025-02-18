#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>

#include "../include/hexapod_main.h"
#include "../include/gpio_control.h"
#include "../include/pwm_control.h"
#include "../include/uart_comm.h"
#include "../include/i2c_comm.h"
#include "../include/servo.h"
#include "../include/pca9685.h"
#include "../include/hexapod_ioctl.h"

#define DEVICE_NAME "hexapod"
#define CLASS_NAME "hexapod_class"

// Module state
static int major_number;
static struct class *hexapod_class = NULL;
static struct device *hexapod_device = NULL;
static struct cdev hexapod_cdev;
static DEFINE_MUTEX(hexapod_mutex);

// Operating mode
// static int hexapod_mode = 0;  // 0: Idle, 1: Walking, 2: Turning

// Declare GPIO arrays with a different name
static int gpio_pins[9];  // Array to store GPIO pin numbers
static const int hexapod_gpio_pins[9] = {
    44, 45, 46,  // Example GPIO pins, adjust these to your actual pins
    47, 48, 49,
    60, 61, 62
};

// Function declarations
static int init_chrdev(void);
static void cleanup_chrdev(void);

// File operations
static int hexapod_open(struct inode *inodep, struct file *filep) {
    if (!mutex_trylock(&hexapod_mutex)) {
        pr_err("Hexapod: Device already in use\n");
        return -EBUSY;
    }
    return 0;
}

static int hexapod_release(struct inode *inodep, struct file *filep) {
    mutex_unlock(&hexapod_mutex);
    return 0;
}

static long hexapod_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    struct uart_msg msg;
    int ret = 0;

    switch (cmd) {
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

// File operations structure
static struct file_operations fops = {
    .open = hexapod_open,
    .release = hexapod_release,
    .unlocked_ioctl = hexapod_ioctl,
    .owner = THIS_MODULE,
};

// Initialize character device
static int init_chrdev(void) {
    int ret, i;

    // Register major number
    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) {
        pr_err("Hexapod: Failed to register major number\n");
        return major_number;
    }

    // Create device class
    hexapod_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(hexapod_class)) {
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(hexapod_class);
    }

    // Create device
    hexapod_device = device_create(hexapod_class, NULL, MKDEV(major_number, 0),
                                 NULL, DEVICE_NAME);
    if (IS_ERR(hexapod_device)) {
        class_destroy(hexapod_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(hexapod_device);
    }

    // Initialize character device
    cdev_init(&hexapod_cdev, &fops);
    ret = cdev_add(&hexapod_cdev, MKDEV(major_number, 0), 1);
    if (ret < 0) {
        device_destroy(hexapod_class, MKDEV(major_number, 0));
        class_destroy(hexapod_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return ret;
    }

    /* // Initialize GPIOs
    for (i = 0; i < 9; i++) {
        gpio_pins[i] = hexapod_gpio_pins[i];
        ret = gpio_request(gpio_pins[i], "sysfs");
        if (ret) {
            pr_err("Failed to request GPIO %d\n", gpio_pins[i]);
            goto gpio_cleanup;
        }
    }

    for (i = 0; i < 9; i++) {
        ret = gpio_direction_output(gpio_pins[i], 0);
        if (ret) {
            pr_err("Failed to set GPIO %d direction\n", gpio_pins[i]);
            goto gpio_cleanup;
        }
    } */

    pr_info("Hexapod: Device initialized successfully\n");
    return 0;

gpio_cleanup:
    cdev_del(&hexapod_cdev);
    device_destroy(hexapod_class, MKDEV(major_number, 0));
    class_destroy(hexapod_class);
    unregister_chrdev(major_number, DEVICE_NAME);
    while (i > 0) {
        i--;
        gpio_free(gpio_pins[i]);
    }
    return ret;
}

// Cleanup character device
static void cleanup_chrdev(void) {
    cdev_del(&hexapod_cdev);
    device_destroy(hexapod_class, MKDEV(major_number, 0));
    class_destroy(hexapod_class);
    unregister_chrdev(major_number, DEVICE_NAME);
}

#define PCA9685_ADDR1 0x40
#define PCA9685_ADDR2 0x41

static int __init __cold hexapod_init(void) {
    int ret;
    int i;
    u8 pca9685_addr1 = PCA9685_ADDR1;
    u8 pca9685_addr2 = PCA9685_ADDR2;

    // Initialize character device
    ret = init_chrdev();
    if (ret < 0)
        return ret;

    // Initialize first PCA9685
    for (i = 0; i < 9; i++) {
        ret = servo_init_channel(pca9685_addr1, i, 0, 180);
        if (ret < 0) {
            pr_err("Failed to initialize servo %d on PCA9685_1\n", i);
            goto cleanup;
        }
    }

    // Initialize second PCA9685
    for (i = 0; i < 9; i++) {
        ret = servo_init_channel(pca9685_addr2, i, 0, 180);
        if (ret < 0) {
            pr_err("Failed to initialize servo %d on PCA9685_2\n", i);
            goto cleanup;
        }
    }

    pr_info("Hexapod: Device initialized successfully\n");
    return 0;

cleanup:
    servo_cleanup();
    cleanup_chrdev();
    return ret;
}

// Module cleanup
static void __exit __cold hexapod_exit(void) {
    // Cleanup all subsystems
    gpio_cleanup();
    servo_cleanup();  // This will also cleanup PCA9685
    uart_cleanup();
    i2c_cleanup();

    // Cleanup character device
    cleanup_chrdev();

    pr_info("Hexapod: Device removed successfully\n");
}

module_init(hexapod_init);
module_exit(hexapod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("StrongFood");
MODULE_DESCRIPTION("Hexapod Robot Control Driver");
MODULE_VERSION("1.0");

