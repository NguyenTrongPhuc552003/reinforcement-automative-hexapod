#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include "../include/uart_comm.h"

static struct uart_dev uart = {0};

int uart_init(void)
{
    return uart_init_with_config(DEFAULT_UART_DEVICE, DEFAULT_UART_BAUD);
}

int uart_init_with_config(const char *device, int baud)
{
    struct tty_struct *tty;
    struct file *file;
    struct ktermios new_termios;

    // Open the TTY device directly
    file = filp_open(device, O_RDWR | O_NOCTTY, 0);
    if (IS_ERR(file)) {
        pr_err("Failed to open TTY device %s\n", device);
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

    // Get current termios settings
    memcpy(&new_termios, &tty->termios, sizeof(struct ktermios));

    // Configure TTY for raw mode
    new_termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    new_termios.c_oflag &= ~OPOST;
    new_termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    new_termios.c_cflag &= ~(CSIZE | PARENB);
    new_termios.c_cflag |= CS8;

    // Set baud rate
    tty_termios_encode_baud_rate(&new_termios, baud, baud);

    // Apply new settings
    if (tty->ops->set_termios)
        tty->ops->set_termios(tty, &new_termios);

    uart.tty = tty;
    filp_close(file, NULL);

    pr_info("UART initialized on %s at %d baud\n", device, baud);
    return 0;
}

int uart_write(const char *data, size_t len)
{
    struct tty_struct *tty = uart.tty;
    int ret;

    if (!tty || !tty->ops || !tty->ops->write)
        return -ENODEV;

    ret = tty->ops->write(tty, data, len);
    return ret < 0 ? ret : 0;
}

int uart_read(char *data, size_t len)
{
    struct tty_struct *tty = uart.tty;

    if (!tty || !tty->ops)
        return -ENODEV;

    // For now, we'll return 0 to indicate no data
    // In a full implementation, we would:
    // 1. Check tty->port->count for data availability
    // 2. Use appropriate locking mechanisms
    // 3. Copy data from the tty's internal buffer
    return 0;
}

void uart_cleanup(void)
{
    if (uart.tty) {
        // The TTY layer will handle cleanup
        uart.tty = NULL;
    }
}

EXPORT_SYMBOL_GPL(uart_init);
EXPORT_SYMBOL_GPL(uart_init_with_config);
EXPORT_SYMBOL_GPL(uart_write);
EXPORT_SYMBOL_GPL(uart_read);
EXPORT_SYMBOL_GPL(uart_cleanup);

MODULE_LICENSE("GPL");
