#ifndef UART_COMM_H
#define UART_COMM_H

#include <linux/tty.h>
#include <linux/serial.h>

struct uart_dev {
    struct tty_struct *tty;
    struct tty_driver *driver;
    int port;
};

int uart_init(const char *device, int baud);
int uart_write(const char *data, size_t len);
int uart_read(char *data, size_t len);
void uart_cleanup(void);

#endif /* UART_COMM_H */
