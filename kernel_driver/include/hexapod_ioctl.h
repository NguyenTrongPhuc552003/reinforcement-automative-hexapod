#ifndef HEXAPOD_IOCTL_H
#define HEXAPOD_IOCTL_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define HEXAPOD_IOC_MAGIC 'H'

// UART message structure
struct uart_msg {
    char *data;
    size_t length;
};

// IOCTL commands
#define UART_SEND    _IOW(HEXAPOD_IOC_MAGIC, 6, struct uart_msg)
#define UART_RECEIVE _IOW(HEXAPOD_IOC_MAGIC, 7, struct uart_msg)

#endif // HEXAPOD_IOCTL_H 