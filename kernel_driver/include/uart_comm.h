#ifndef UART_COMM_H
#define UART_COMM_H

#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/types.h>

struct uart_dev {
    struct tty_struct *tty;
    struct tty_driver *driver;
    int port;
};

// Default UART configuration
#define DEFAULT_UART_DEVICE "/dev/ttyS0"
#define DEFAULT_UART_BAUD 115200

// Function declarations
int uart_init(void);  // Use default configuration
int uart_init_with_config(const char *device, int baud);  // Custom configuration
int uart_write(const u8 *data, size_t len);
int uart_read(u8 *data, size_t len);
void uart_cleanup(void);

#endif /* UART_COMM_H */
