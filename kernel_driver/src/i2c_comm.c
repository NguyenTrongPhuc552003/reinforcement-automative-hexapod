#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "../include/i2c_comm.h"

static struct i2c_client *i2c_client = NULL;

int i2c_init_device(struct i2c_adapter *adapter, u8 addr) {
    struct i2c_board_info board_info = {
        I2C_BOARD_INFO("hexapod", addr)
    };

    if (!adapter)
        return -EINVAL;

    i2c_client = i2c_new_device(adapter, &board_info);
    if (!i2c_client) {
        pr_err("Failed to create I2C client\n");
        return -ENODEV;
    }

    return 0;
}

int i2c_write_data(u8 addr, u8 reg, u8 *data, size_t len) {
    struct i2c_msg msg;
    u8 *buf;
    int ret;

    if (!i2c_client || !data)
        return -EINVAL;

    buf = kmalloc(len + 1, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    buf[0] = reg;
    memcpy(&buf[1], data, len);

    msg.addr = addr;
    msg.flags = 0;  // Write
    msg.len = len + 1;
    msg.buf = buf;

    ret = i2c_transfer(i2c_client->adapter, &msg, 1);
    kfree(buf);

    return ret < 0 ? ret : 0;
}

int i2c_read_data(u8 addr, u8 reg, u8 *data, size_t len) {
    struct i2c_msg msgs[2];
    int ret;

    if (!i2c_client || !data)
        return -EINVAL;

    // First message: Write register address
    msgs[0].addr = addr;
    msgs[0].flags = 0;  // Write
    msgs[0].len = 1;
    msgs[0].buf = &reg;

    // Second message: Read data
    msgs[1].addr = addr;
    msgs[1].flags = I2C_M_RD;  // Read
    msgs[1].len = len;
    msgs[1].buf = data;

    ret = i2c_transfer(i2c_client->adapter, msgs, 2);
    return ret < 0 ? ret : 0;
}

void i2c_cleanup(void) {
    if (i2c_client) {
        i2c_unregister_device(i2c_client);
        i2c_client = NULL;
    }
}
