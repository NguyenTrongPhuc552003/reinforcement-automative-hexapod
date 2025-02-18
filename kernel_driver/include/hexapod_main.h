#ifndef HEXAPOD_MAIN_H
#define HEXAPOD_MAIN_H

#include <linux/types.h>

// IOCTL commands
#define HEXAPOD_IOC_MAGIC 'H'
#define IOCTL_SET_MODE _IOW(HEXAPOD_IOC_MAGIC, 1, int)
#define IOCTL_SET_SERVO _IOW(HEXAPOD_IOC_MAGIC, 2, struct servo_control)
#define IOCTL_SET_GPIO _IOW(HEXAPOD_IOC_MAGIC, 3, struct gpio_control)
#define IOCTL_SEND_UART _IOW(HEXAPOD_IOC_MAGIC, 4, struct uart_message)
#define IOCTL_I2C_TRANSFER _IOWR(HEXAPOD_IOC_MAGIC, 5, struct i2c_transfer)

// Control structures
struct servo_control {
    unsigned int channel;    // Changed from servo_id to channel
    unsigned int angle;      // 0-180 degrees
};

struct gpio_control {
    unsigned int pin;
    int value;
};

struct uart_message {
    unsigned int port;
    char data[256];
    size_t length;
};

struct i2c_transfer {
    u8 addr;
    u8 reg;
    u8 data[32];
    size_t length;
    bool is_read;
};

#endif /* HEXAPOD_MAIN_H */
