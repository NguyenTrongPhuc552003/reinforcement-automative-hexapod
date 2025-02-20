#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "pca9685.h"

#define MAX_DEVICES 2

static struct i2c_client *pca9685_clients[MAX_DEVICES];
static int num_devices = 0;

/* Optimized I2C Read */
static s32 pca9685_read_bytes(struct i2c_client *client, u8 reg, u8 *buf, int len) {
    struct i2c_msg msgs[2];
    int ret;
    
    msgs[0].addr  = client->addr;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;
    
    msgs[1].addr  = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = len;
    msgs[1].buf   = buf;
    
    ret = i2c_transfer(client->adapter, msgs, 2);
    return (ret == 2) ? 0 : ret;
}

/* Optimized I2C Write */
static s32 pca9685_write_bytes(struct i2c_client *client, u8 reg, const u8 *buf, int len) {
    struct i2c_msg msg;
    u8 *write_buf;
    int ret;

    write_buf = kmalloc(len + 1, GFP_KERNEL);
    if (!write_buf)
        return -ENOMEM;

    write_buf[0] = reg;
    memcpy(&write_buf[1], buf, len);

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = len + 1;
    msg.buf = write_buf;

    ret = i2c_transfer(client->adapter, &msg, 1);
    kfree(write_buf);

    return (ret == 1) ? 0 : ret;
}

/* Single byte operations */
static s32 pca9685_read_byte(struct i2c_client *client, u8 reg) {
    u8 val;
    int ret = pca9685_read_bytes(client, reg, &val, 1);
    return ret ? ret : val;
}

static s32 pca9685_write_byte(struct i2c_client *client, u8 reg, u8 val) {
    return pca9685_write_bytes(client, reg, &val, 1);
}

/* Set PWM values */
int pca9685_set_pwm(struct i2c_client *client, u8 channel, u16 on_time, u16 off_time) {
    u8 data[4];
    int ret;
    
    if (channel >= PCA9685_NUM_CHANNELS)
        return -EINVAL;

    data[0] = on_time & 0xFF;
    data[1] = (on_time >> 8) & 0xFF;
    data[2] = off_time & 0xFF;
    data[3] = (off_time >> 8) & 0xFF;

    ret = pca9685_write_bytes(client, PCA9685_LED0_ON_L + 4 * channel, data, 4);
    return ret;
}

/* Initialize PCA9685 */
int pca9685_init(struct i2c_client *client) {
    int ret;

    // Reset
    ret = pca9685_write_byte(client, PCA9685_MODE1, PCA9685_SLEEP);
    if (ret < 0)
        return ret;
    msleep(1);

    // Set mode
    ret = pca9685_write_byte(client, PCA9685_MODE1, 
                            PCA9685_AI | PCA9685_ALLCALL);
    if (ret < 0)
        return ret;
    msleep(1);

    // Set PWM frequency to 50Hz
    ret = pca9685_write_byte(client, PCA9685_PRESCALE, PCA9685_PRESCALE_50HZ);
    if (ret < 0)
        return ret;

    // Enable oscillator
    ret = pca9685_write_byte(client, PCA9685_MODE1, 
                            PCA9685_AI | PCA9685_ALLCALL | PCA9685_RESTART);
    if (ret < 0)
        return ret;
    msleep(1);

    return 0;
}

/* Register new PCA9685 device */
int pca9685_register(struct i2c_adapter *adapter, u8 addr) {
    struct i2c_client *client;
    int ret;

    if (num_devices >= MAX_DEVICES)
        return -ENOMEM;

    client = i2c_new_dummy(adapter, addr);
    if (!client)
        return -ENODEV;

    ret = pca9685_init(client);
    if (ret < 0) {
        i2c_unregister_device(client);
        return ret;
    }

    pca9685_clients[num_devices++] = client;
    return 0;
}

/* Cleanup PCA9685 devices */
void pca9685_cleanup(void) {
    int i;
    for (i = 0; i < num_devices; i++) {
        if (pca9685_clients[i]) {
            i2c_unregister_device(pca9685_clients[i]);
            pca9685_clients[i] = NULL;
        }
    }
    num_devices = 0;
}

EXPORT_SYMBOL_GPL(pca9685_register);
EXPORT_SYMBOL_GPL(pca9685_cleanup);
EXPORT_SYMBOL_GPL(pca9685_set_pwm);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phuc Nguyen");
MODULE_DESCRIPTION("PCA9685 PWM driver for BeagleBone");