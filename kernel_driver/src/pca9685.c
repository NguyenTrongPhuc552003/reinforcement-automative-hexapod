#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "../include/pca9685.h"
#include "../include/i2c_comm.h"

#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_SUBADR1 0x02
#define PCA9685_SUBADR2 0x03
#define PCA9685_SUBADR3 0x04
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_LED0_ON_H 0x07
#define PCA9685_LED0_OFF_L 0x08
#define PCA9685_LED0_OFF_H 0x09

#define PCA9685_RESTART 0x80
#define PCA9685_SLEEP 0x10
#define PCA9685_ALLCALL 0x01
#define PCA9685_INVRT 0x10
#define PCA9685_OUTDRV 0x04

struct pca9685_device {
    u8 i2c_addr;
    bool is_initialized;
    u8 prescale_value;
};

static struct pca9685_device pca9685_devices[MAX_PCA9685_DEVICES];
static int num_devices = 0;

// Helper functions for single byte operations
static inline int pca9685_read_reg(u8 i2c_addr, u8 reg, u8 *value)
{
    return i2c_read_data(i2c_addr, reg, value, 1);
}

static inline int pca9685_write_reg(u8 i2c_addr, u8 reg, u8 value)
{
    return i2c_write_data(i2c_addr, reg, &value, 1);
}

void pca9685_reset(void)
{
    // Reset implementation
}

int pca9685_device_init(u8 i2c_addr)
{
    int ret;

    // Reset the device
    pca9685_reset();

    // Set sleep mode
    ret = pca9685_write_reg(i2c_addr, PCA9685_MODE1, 0x10);
    if (ret < 0)
        return ret;

    // Set mode1 (normal mode, auto-increment enabled)
    ret = pca9685_write_reg(i2c_addr, PCA9685_MODE1, PCA9685_ALLCALL);
    if (ret < 0)
        return ret;

    // Wait for oscillator
    udelay(500);

    // Add device to our list if not already present
    if (num_devices < MAX_PCA9685_DEVICES) {
        int i;
        bool found = false;

        for (i = 0; i < num_devices; i++) {
            if (pca9685_devices[i].i2c_addr == i2c_addr) {
                found = true;
                break;
            }
        }

        if (!found) {
            pca9685_devices[num_devices].i2c_addr = i2c_addr;
            pca9685_devices[num_devices].is_initialized = true;
            num_devices++;
        }
    }

    return 0;
}

int pca9685_set_pwm_freq(u8 i2c_addr, unsigned int freq_hz)
{
    int ret;
    u8 prescale_val;
    u8 old_mode;
    u8 new_mode;

    // Calculate prescale value
    prescale_val = (u8)(25000000.0f / (4096.0f * freq_hz) - 0.5f);

    // Read current mode
    ret = pca9685_read_reg(i2c_addr, PCA9685_MODE1, &old_mode);
    if (ret < 0)
        return ret;

    // Set sleep mode
    new_mode = (old_mode & ~0x7F) | 0x10;
    ret = pca9685_write_reg(i2c_addr, PCA9685_MODE1, new_mode);
    if (ret < 0)
        return ret;

    // Set prescale value
    ret = pca9685_write_reg(i2c_addr, PCA9685_PRESCALE, prescale_val);
    if (ret < 0)
        return ret;

    // Restore old mode
    ret = pca9685_write_reg(i2c_addr, PCA9685_MODE1, old_mode);
    if (ret < 0)
        return ret;

    // Wait for oscillator
    udelay(500);

    // Update mode with restart bit
    ret = pca9685_write_reg(i2c_addr, PCA9685_MODE1, old_mode | 0x80);
    if (ret < 0)
        return ret;

    return 0;
}

int pca9685_set_pwm(u8 i2c_addr, u8 channel, u16 on_time, u16 off_time)
{
    int ret;
    u8 led_start = PCA9685_LED0_ON_L + (channel * 4);

    ret = pca9685_write_reg(i2c_addr, led_start, on_time & 0xFF);
    if (ret < 0)
        return ret;

    ret = pca9685_write_reg(i2c_addr, led_start + 1, on_time >> 8);
    if (ret < 0)
        return ret;

    ret = pca9685_write_reg(i2c_addr, led_start + 2, off_time & 0xFF);
    if (ret < 0)
        return ret;

    ret = pca9685_write_reg(i2c_addr, led_start + 3, off_time >> 8);
    return ret;
}

int pca9685_device_sleep(u8 i2c_addr)
{
    int ret;
    u8 mode1;

    // Read current mode1 register
    ret = pca9685_read_reg(i2c_addr, PCA9685_MODE1, &mode1);
    if (ret < 0)
        return ret;

    // Set sleep bit
    mode1 |= PCA9685_SLEEP;
    return pca9685_write_reg(i2c_addr, PCA9685_MODE1, mode1);
}

int pca9685_device_wake(u8 i2c_addr)
{
    int ret;
    u8 mode1;

    // Read current mode1 register
    ret = pca9685_read_reg(i2c_addr, PCA9685_MODE1, &mode1);
    if (ret < 0)
        return ret;

    // Clear sleep bit
    mode1 &= ~PCA9685_SLEEP;
    ret = pca9685_write_reg(i2c_addr, PCA9685_MODE1, mode1);
    if (ret < 0)
        return ret;

    // Wait for oscillator to stabilize
    udelay(500);
    return 0;
}

void pca9685_cleanup(void)
{
    int i;
    for (i = 0; i < num_devices; i++) {
        if (pca9685_devices[i].is_initialized) {
            // Put device in sleep mode
            pca9685_device_sleep(pca9685_devices[i].i2c_addr);
            pca9685_devices[i].is_initialized = false;
        }
    }
    num_devices = 0;
}

// Export symbols at the end of the file
EXPORT_SYMBOL_GPL(pca9685_reset);
EXPORT_SYMBOL_GPL(pca9685_device_init);
EXPORT_SYMBOL_GPL(pca9685_set_pwm);
EXPORT_SYMBOL_GPL(pca9685_set_pwm_freq);
EXPORT_SYMBOL_GPL(pca9685_device_sleep);
EXPORT_SYMBOL_GPL(pca9685_device_wake);
EXPORT_SYMBOL_GPL(pca9685_cleanup);

MODULE_LICENSE("GPL");