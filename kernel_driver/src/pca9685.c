#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "pca9685.h"
#include "servo.h"

static struct i2c_client *pca9685_clients[MAX_DEVICES];
static int num_devices = 0;

// Read a single byte from PCA9685
static s32 pca9685_read_byte(struct i2c_client *client, u8 reg)
{
    s32 ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0)
        pr_err("PCA9685: Failed to read from register 0x%02x\n", reg);
    return ret;
}

// Write a single byte to PCA9685
static s32 pca9685_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
    s32 ret = i2c_smbus_write_byte_data(client, reg, data);
    if (ret < 0)
        pr_err("PCA9685: Failed to write 0x%02x to register 0x%02x\n", data, reg);
    return ret;
}

// Write 4 bytes to PCA9685 for PWM control
static s32 pca9685_write_pwm(struct i2c_client *client, u8 reg, u16 on, u16 off)
{
    u8 data[4];
    s32 ret;

    data[0] = on & 0xFF;
    data[1] = on >> 8;
    data[2] = off & 0xFF;
    data[3] = off >> 8;

    ret = i2c_smbus_write_i2c_block_data(client, reg, 4, data);
    if (ret < 0)
        pr_err("PCA9685: Failed to write PWM data to register 0x%02x\n", reg);
    return ret;
}

// Helper function to find client by I2C address
static struct i2c_client *find_client(u8 i2c_addr)
{
    int i;
    for (i = 0; i < num_devices; i++) {
        if (pca9685_clients[i] && pca9685_clients[i]->addr == i2c_addr) {
            return pca9685_clients[i];
        }
    }
    return NULL;
}

int pca9685_device_init(u8 i2c_addr)
{
    struct i2c_adapter *adapter;
    struct i2c_board_info board_info = {
        I2C_BOARD_INFO("pca9685", i2c_addr)
    };
    struct i2c_client *client;
    s32 ret;

    // Get I2C adapter for i2c-0
    adapter = i2c_get_adapter(0);
    if (!adapter) {
        pr_err("PCA9685: Failed to get I2C adapter\n");
        return -ENODEV;
    }

    // Create I2C client
    client = i2c_new_device(adapter, &board_info);
    if (!client) {
        pr_err("PCA9685: Failed to create I2C client for address 0x%02x\n", i2c_addr);
        i2c_put_adapter(adapter);
        return -ENODEV;
    }

    // Store client in array
    if (num_devices >= MAX_DEVICES) {
        pr_err("PCA9685: Too many devices\n");
        i2c_unregister_device(client);
        i2c_put_adapter(adapter);
        return -ENOSPC;
    }
    pca9685_clients[num_devices++] = client;

    // Software reset
    ret = pca9685_write_byte(client, PCA9685_MODE1, PCA9685_RESTART);
    if (ret < 0) {
        pr_err("PCA9685: Failed to reset device at 0x%02x\n", i2c_addr);
        goto error;
    }
    msleep(10);

    // Set to sleep mode
    ret = pca9685_write_byte(client, PCA9685_MODE1, PCA9685_SLEEP);
    if (ret < 0) {
        pr_err("PCA9685: Failed to set sleep mode at 0x%02x\n", i2c_addr);
        goto error;
    }

    // Set PWM frequency
    ret = pca9685_set_pwm_freq(i2c_addr, PWM_FREQ);
    if (ret < 0) {
        pr_err("PCA9685: Failed to set PWM frequency at 0x%02x\n", i2c_addr);
        goto error;
    }

    // Set normal mode with auto-increment enabled
    ret = pca9685_write_byte(client, PCA9685_MODE1, PCA9685_ALLCALL);
    if (ret < 0) {
        pr_err("PCA9685: Failed to set normal mode at 0x%02x\n", i2c_addr);
        goto error;
    }
    msleep(5);

    // Set output mode
    ret = pca9685_write_byte(client, PCA9685_MODE2, PCA9685_OUTDRV);
    if (ret < 0) {
        pr_err("PCA9685: Failed to set output mode at 0x%02x\n", i2c_addr);
        goto error;
    }

    pr_info("PCA9685: Device at 0x%02x initialized successfully\n", i2c_addr);
    return 0;

error:
    i2c_unregister_device(client);
    pca9685_clients[--num_devices] = NULL;
    return ret;
}

int pca9685_set_pwm_freq(u8 i2c_addr, u8 freq)
{
    struct i2c_client *client;
    u32 prescale;
    s32 oldmode, newmode, ret;

    client = find_client(i2c_addr);
    if (!client) {
        pr_err("PCA9685: Device not found at address 0x%02x\n", i2c_addr);
        return -ENODEV;
    }

    // Calculate prescale value for desired frequency
    prescale = (INTERNAL_CLOCK_FREQ / (PWM_RESOLUTION * freq)) - 1;
    if (prescale < 3)
        prescale = 3;
    if (prescale > 255)
        prescale = 255;

    // Read current mode
    oldmode = pca9685_read_byte(client, PCA9685_MODE1);
    if (oldmode < 0)
        return oldmode;

    // Set sleep mode to change prescale
    newmode = (oldmode & ~PCA9685_RESTART) | PCA9685_SLEEP;
    ret = pca9685_write_byte(client, PCA9685_MODE1, newmode);
    if (ret < 0)
        return ret;

    // Set prescale value
    ret = pca9685_write_byte(client, PCA9685_PRESCALE, prescale);
    if (ret < 0)
        return ret;

    // Restore mode
    ret = pca9685_write_byte(client, PCA9685_MODE1, oldmode);
    if (ret < 0)
        return ret;

    msleep(5);

    // Set restart bit
    ret = pca9685_write_byte(client, PCA9685_MODE1, oldmode | PCA9685_RESTART);
    if (ret < 0)
        return ret;

    pr_info("PCA9685: Set frequency to %dHz (prescale: %d)\n", freq, prescale);
    return 0;
}

int pca9685_set_pwm(u8 i2c_addr, u8 channel, u16 on, u16 off)
{
    struct i2c_client *client;
    u8 reg;

    if (channel >= PWM_CHANNELS) {
        pr_err("PCA9685: Invalid channel %d\n", channel);
        return -EINVAL;
    }

    client = find_client(i2c_addr);
    if (!client) {
        pr_err("PCA9685: Device not found at address 0x%02x\n", i2c_addr);
        return -ENODEV;
    }

    reg = PCA9685_LED0_ON_L + (channel * 4);
    return pca9685_write_pwm(client, reg, on, off);
}

void pca9685_cleanup(void)
{
    int i;
    struct i2c_client *client;

    for (i = 0; i < num_devices; i++) {
        client = pca9685_clients[i];
        if (client) {
            pca9685_write_byte(client, PCA9685_MODE1, PCA9685_SLEEP);
            i2c_unregister_device(client);
            pca9685_clients[i] = NULL;
        }
    }
    num_devices = 0;
    pr_info("PCA9685: Cleanup complete\n");
}

MODULE_LICENSE("GPL");