#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "pca9685.h"

/* PCA9685 registers */
#define PCA9685_MODE1       0x00
#define PCA9685_MODE2       0x01
#define PCA9685_SUBADR1     0x02
#define PCA9685_SUBADR2     0x03
#define PCA9685_SUBADR3     0x04
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0        0x06
#define PCA9685_ALL_LED_OFF_H 0xFA

#define PCA9685_I2C_ADDR    0x40

/* Internal oscillator frequency */
#define OSC_CLOCK          25000000    /* 25MHz */

/* LED channel register offsets */
#define LED_ON_L           0x0
#define LED_ON_H           0x1
#define LED_OFF_L          0x2
#define LED_OFF_H          0x3

#define MODE1_RESTART       0x80
#define MODE1_SLEEP         0x10
#define MODE1_AI            0x20
#define MODE2_OUTDRV        0x04

#define PWM_RESOLUTION      4096
#define PWM_PERIOD_MS       20
#define PWM_FREQUENCY       50

static struct i2c_client *pca9685_client;

/* Helper Functions */
static int pca9685_write_reg(u8 reg, u8 val)
{
    int ret = i2c_smbus_write_byte_data(pca9685_client, reg, val);
    if (ret < 0)
        pr_err("PCA9685: Failed to write 0x%02x to reg 0x%02x (ret=%d)\n", 
               val, reg, ret);
    return ret;
}

static int pca9685_read_reg(u8 reg)
{
    int ret = i2c_smbus_read_byte_data(pca9685_client, reg);
    if (ret < 0)
        pr_err("PCA9685: Failed to read reg 0x%02x (ret=%d)\n", reg, ret);
    return ret;
}

/* Initialize PCA9685 */
int pca9685_init(void)
{
    int ret;
    struct i2c_adapter *adapter;
    u8 mode1;

    pr_info("PCA9685: Initializing PWM controller\n");

    /* Get I2C adapter for bus 3 */
    adapter = i2c_get_adapter(3);
    if (!adapter) {
        pr_err("PCA9685: Failed to get I2C adapter\n");
        return -ENODEV;
    }

    /* Create I2C client */
    pca9685_client = i2c_new_dummy(adapter, PCA9685_I2C_ADDR);
    i2c_put_adapter(adapter);

    if (!pca9685_client) {
        pr_err("PCA9685: Failed to create I2C client\n");
        return -ENOMEM;
    }

    /* Software reset */
    ret = pca9685_write_reg(PCA9685_MODE1, MODE1_RESTART);
    if (ret < 0) {
        pr_err("PCA9685: Reset failed\n");
        goto err_remove;
    }
    msleep(10);

    /* Read MODE1 register to verify device */
    mode1 = pca9685_read_reg(PCA9685_MODE1);
    if (mode1 < 0) {
        pr_err("PCA9685: Failed to verify device\n");
        ret = -ENODEV;
        goto err_remove;
    }

    /* Set MODE1: Auto-increment enabled, Normal mode */
    ret = pca9685_write_reg(PCA9685_MODE1, MODE1_AI);
    if (ret < 0) {
        pr_err("PCA9685: Failed to set MODE1\n");
        goto err_remove;
    }

    /* Set MODE2: Totem pole structure */
    ret = pca9685_write_reg(PCA9685_MODE2, MODE2_OUTDRV);
    if (ret < 0) {
        pr_err("PCA9685: Failed to set MODE2\n");
        goto err_remove;
    }

    /* Set default PWM frequency */
    ret = pca9685_set_pwm_freq(PWM_FREQUENCY);
    if (ret < 0) {
        pr_err("PCA9685: Failed to set PWM frequency\n");
        goto err_remove;
    }

    pr_info("PCA9685: Initialization complete\n");
    return 0;

err_remove:
    i2c_unregister_device(pca9685_client);
    pca9685_client = NULL;
    return ret;
}

/* Set PWM frequency */
int pca9685_set_pwm_freq(u16 freq)
{
    int ret;
    u8 prescale;
    u8 oldmode;

    if (!pca9685_client)
        return -ENODEV;

    /* Calculate prescale value for 25MHz oscillator */
    prescale = (OSC_CLOCK / (PWM_RESOLUTION * (u32)freq)) - 1;

    /* Read current mode */
    oldmode = pca9685_read_reg(PCA9685_MODE1);
    if (oldmode < 0)
        return oldmode;

    /* Enter sleep mode */
    ret = pca9685_write_reg(PCA9685_MODE1, (oldmode & ~MODE1_RESTART) | MODE1_SLEEP);
    if (ret < 0)
        return ret;

    /* Set prescale */
    ret = pca9685_write_reg(PCA9685_PRESCALE, prescale);
    if (ret < 0)
        return ret;

    /* Restore mode */
    ret = pca9685_write_reg(PCA9685_MODE1, oldmode);
    if (ret < 0)
        return ret;

    /* Wait for oscillator */
    msleep(5);

    /* Restart */
    ret = pca9685_write_reg(PCA9685_MODE1, oldmode | MODE1_RESTART);
    if (ret < 0)
        return ret;

    return 0;
}

/* Set PWM values for a channel */
int pca9685_set_pwm(u8 channel, u16 on_time, u16 off_time)
{
    u8 reg;
    int ret;

    if (!pca9685_client)
        return -ENODEV;

    /* Calculate base register for this channel */
    reg = PCA9685_LED0 + (channel * 4);

    /* Write ON time */
    ret = pca9685_write_reg(reg + LED_ON_L, on_time & 0xFF);
    if (ret < 0)
        return ret;
    ret = pca9685_write_reg(reg + LED_ON_H, (on_time >> 8) & 0x0F);
    if (ret < 0)
        return ret;

    /* Write OFF time */
    ret = pca9685_write_reg(reg + LED_OFF_L, off_time & 0xFF);
    if (ret < 0)
        return ret;
    ret = pca9685_write_reg(reg + LED_OFF_H, (off_time >> 8) & 0x0F);
    if (ret < 0)
        return ret;

    return 0;
}

/* Set PWM in milliseconds */
int pca9685_set_pwm_ms(u8 channel, u16 ms)
{
    u32 off_time;
    
    if (!pca9685_client)
        return -ENODEV;

    /* Clamp pulse width to valid range */
    if (ms < PWM_MIN_TIME)
        ms = PWM_MIN_TIME;
    if (ms > PWM_MAX_TIME)
        ms = PWM_MAX_TIME;

    /* Convert ms to 12-bit value (0-4095)
     * For 50Hz: 20ms period = 4096 steps
     * off_time = (ms * 4096) / 20
     */
    off_time = ((u32)ms * PWM_RESOLUTION) / PWM_PERIOD_MS;
    if (off_time > PWM_RESOLUTION)
        off_time = PWM_RESOLUTION;

    return pca9685_set_pwm(channel, 0, (u16)off_time);
}

/* Cleanup PCA9685 */
void pca9685_cleanup(void)
{
    int i;

    if (pca9685_client) {
        /* Turn off all channels */
        for (i = 0; i < PWM_RESOLUTION; i++)
            pca9685_set_pwm(i, 0, 0);
            
        i2c_unregister_device(pca9685_client);
        pca9685_client = NULL;
    }
}

EXPORT_SYMBOL_GPL(pca9685_init);
EXPORT_SYMBOL_GPL(pca9685_cleanup);
EXPORT_SYMBOL_GPL(pca9685_set_pwm);
EXPORT_SYMBOL_GPL(pca9685_set_pwm_freq);
EXPORT_SYMBOL_GPL(pca9685_set_pwm_ms);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("PCA9685 PWM Controller Driver");
MODULE_VERSION("1.0");