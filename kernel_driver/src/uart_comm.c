#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include "../include/uart_comm.h"

static struct uart_dev uart = {0};

int uart_init(const char *device, int baud) {
    struct tty_struct *tty;
    struct file *file;
    int ret;

    // Open the TTY device directly
    file = filp_open(device, O_RDWR | O_NOCTTY, 0);
    if (IS_ERR(file)) {
        pr_err("Failed to open TTY device\n");
        return PTR_ERR(file);
    }

    if (!file->private_data) {
        filp_close(file, NULL);
        return -ENODEV;
    }

    tty = ((struct tty_file_private *)file->private_data)->tty;
    if (!tty) {
        filp_close(file, NULL);
        return -ENODEV;
    }

    uart.tty = tty;
    filp_close(file, NULL);
    return 0;
}

int uart_write(const char *data, size_t len) {
    struct tty_struct *tty = uart.tty;
    int ret;

    if (!tty || !tty->ops || !tty->ops->write)
        return -ENODEV;

    ret = tty->ops->write(tty, data, len);
    return ret < 0 ? ret : 0;
}

int uart_read(char *data, size_t len) {
    // For now, just return 0 as reading will be implemented later
    return 0;
}

void uart_cleanup(void) {
    if (uart.tty) {
        // No need to release tty_struct when using polling driver
        uart.tty = NULL;
    }

    uart.driver = NULL;  // Driver is managed by the system
}

EXPORT_SYMBOL_GPL(uart_init);
EXPORT_SYMBOL_GPL(uart_write);
EXPORT_SYMBOL_GPL(uart_read);
EXPORT_SYMBOL_GPL(uart_cleanup);

MODULE_LICENSE("GPL");
