#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/err.h>
#include "pca9685.h"

/* Global variables */
static struct i2c_client *pca9685_client;
// Add support for secondary controller
static struct i2c_client *pca9685_secondary_client;

// Add a function to select the appropriate client based on channel number
static struct i2c_client *pca9685_get_client(u8 channel)
{
    // Use primary controller for channels 0-15
    // Use secondary controller for channels 16-31 (mapped to 0-15 internally)
    if (channel < 16)
        return pca9685_client;
    else
        return pca9685_secondary_client;
}

/**
 * Write to a PCA9685 register with error handling
 */
static int pca9685_write_reg(u8 reg, u8 val)
{
    int ret, retries = 3;
    struct i2c_client *client = pca9685_get_client(reg / 4);

    if (!client)
        return -ENODEV;

    /* Try up to 3 times with delay between retries */
    while (retries--)
    {
        ret = i2c_smbus_write_byte_data(client, reg, val);
        if (ret == 0)
            return 0;

        /* Delay before retry */
        // msleep(5);
        /* Use schedule_timeout_interruptible() instead of msleep() */
        schedule_timeout_interruptible(msecs_to_jiffies(5));
    }

    pr_err("PCA9685: Failed to write to register 0x%02x: %d\n", reg, ret);
    return ret;
}

/**
 * Read from a PCA9685 register with error handling
 */
static int pca9685_read_reg(u8 reg)
{
    int ret, retries = 3;
    struct i2c_client *client = pca9685_get_client(reg / 4);

    if (!client)
        return -ENODEV;

    /* Try up to 3 times with delay between retries */
    while (retries--)
    {
        ret = i2c_smbus_read_byte_data(client, reg);
        if (ret >= 0)
            return ret;

        /* Delay before retry */
        // msleep(5);
        /* Use schedule_timeout_interruptible() instead of msleep() */
        schedule_timeout_interruptible(msecs_to_jiffies(5));
    }

    pr_err("PCA9685: Failed to read register 0x%02x: %d\n", reg, ret);
    return ret;
}

/**
 * Initialize the PCA9685 PWM controller
 */
int pca9685_init(void)
{
    struct i2c_adapter *adapter;
    int ret;
    u8 prescale;

    /* Get I2C adapter */
    adapter = i2c_get_adapter(PCA9685_I2C_BUS);
    if (!adapter)
    {
        pr_err("PCA9685: Failed to get I2C adapter %d\n", PCA9685_I2C_BUS);
        return -ENODEV;
    }

    /* Create I2C client - use i2c_new_dummy_device instead of i2c_new_dummy */
    pca9685_client = i2c_new_dummy_device(adapter, PCA9685_I2C_ADDR);
    i2c_put_adapter(adapter);

    if (!pca9685_client)
    {
        pr_err("PCA9685: Failed to create I2C client\n");
        return -ENOMEM;
    }

    /* Software reset */
    ret = pca9685_write_reg(PCA9685_MODE1, 0x80); /* Set reset bit */
    if (ret < 0)
        goto error;
    // msleep(10); /* Wait for reset to complete */
    /* Use schedule_timeout_interruptible() instead of msleep() */
    schedule_timeout_interruptible(msecs_to_jiffies(10));

    /* Set to sleep mode (required to change prescale) */
    ret = pca9685_write_reg(PCA9685_MODE1, MODE1_SLEEP);
    if (ret < 0)
        goto error;

    /* Calculate prescale for default frequency */
    prescale = (u8)((PCA9685_CLOCK_FREQ / (PCA9685_PWM_RES * PCA9685_PWM_FREQ)) - 1);

    /* Set prescale value */
    ret = pca9685_write_reg(PCA9685_PRESCALE, prescale);
    if (ret < 0)
        goto error;

    /* Set to normal operation with auto-increment */
    ret = pca9685_write_reg(PCA9685_MODE1, MODE1_AI);
    if (ret < 0)
        goto error;
    // msleep(10); /* Wait for oscillator */
    /* Use schedule_timeout_interruptible() instead of msleep() */
    schedule_timeout_interruptible(msecs_to_jiffies(10));

    /* Set output mode to totem pole (default) */
    ret = pca9685_write_reg(PCA9685_MODE2, MODE2_OUTDRV);
    if (ret < 0)
        goto error;

    /* Turn off all outputs */
    ret = pca9685_write_reg(PCA9685_ALL_LED_OFF_L, 0);
    if (ret < 0)
        goto error;
    ret = pca9685_write_reg(PCA9685_ALL_LED_OFF_H, 0x10); /* Full off */
    if (ret < 0)
        goto error;

    pr_info("PCA9685: Initialized at address 0x%02x, prescale %d, freq %d Hz\n",
            PCA9685_I2C_ADDR, prescale, PCA9685_PWM_FREQ);
    return 0;

error:
    if (pca9685_client)
    {
        i2c_unregister_device(pca9685_client);
        pca9685_client = NULL;
    }
    return ret;
}

/**
 * Set PWM frequency
 */
int pca9685_set_pwm_freq(u16 freq_hz)
{
    int ret;
    u8 prescale, oldmode;

    if (freq_hz < 24 || freq_hz > 1526)
        return -EINVAL; /* Frequency out of range */

    /* Calculate prescale value */
    prescale = (u8)((PCA9685_CLOCK_FREQ / (PCA9685_PWM_RES * freq_hz)) - 1);

    /* Read current mode */
    ret = pca9685_read_reg(PCA9685_MODE1);
    if (ret < 0)
        return ret;
    oldmode = ret;

    /* Set sleep mode */
    ret = pca9685_write_reg(PCA9685_MODE1, (oldmode & ~MODE1_RESTART) | MODE1_SLEEP);
    if (ret < 0)
        return ret;

    /* Set prescale */
    ret = pca9685_write_reg(PCA9685_PRESCALE, prescale);
    if (ret < 0)
        return ret;

    /* Restore previous mode */
    ret = pca9685_write_reg(PCA9685_MODE1, oldmode);
    if (ret < 0)
        return ret;
    // msleep(10); /* Wait for oscillator */
    /* Use schedule_timeout_interruptible() instead of msleep() */
    schedule_timeout_interruptible(msecs_to_jiffies(10));

    /* Restart */
    ret = pca9685_write_reg(PCA9685_MODE1, oldmode | MODE1_RESTART);
    if (ret < 0)
        return ret;

    // Simplify the debug message - remove excessive details
    pr_info("PCA9685: Set frequency to %d Hz\n", freq_hz);
    return 0;
}

/**
 * Set PWM values for a specific channel
 */
int pca9685_set_pwm(u8 channel, u16 on, u16 off)
{
    int ret;
    u8 reg;
    u8 data[4];

    if (channel > 31 || on > 4095 || off > 4095)
        return -EINVAL;

    reg = PCA9685_LED0_ON_L + ((channel % 16) * 4);

    /* Prepare data for all 4 registers at once */
    data[0] = on & 0xFF;
    data[1] = (on >> 8) & 0x0F;
    data[2] = off & 0xFF;
    data[3] = (off >> 8) & 0x0F;

    /* Write all 4 registers in a single block write if possible */
    if (i2c_check_functionality(pca9685_get_client(channel)->adapter, I2C_FUNC_SMBUS_WRITE_I2C_BLOCK))
    {
        ret = i2c_smbus_write_i2c_block_data(pca9685_get_client(channel), reg, 4, data);
        if (ret == 0)
            return 0;
    }

    /* Fall back to individual writes if block write fails */
    ret = pca9685_write_reg(reg, data[0]);
    if (ret < 0)
        return ret;

    ret = pca9685_write_reg(reg + 1, data[1]);
    if (ret < 0)
        return ret;

    ret = pca9685_write_reg(reg + 2, data[2]);
    if (ret < 0)
        return ret;

    ret = pca9685_write_reg(reg + 3, data[3]);
    if (ret < 0)
        return ret;

    return 0;
}

/**
 * Set PWM pulse width in microseconds
 */
int pca9685_set_pwm_us(u8 channel, u16 us)
{
    unsigned int pulse;

    if (channel > 31)
        return -EINVAL;

    /* Ensure pulse width is within limits */
    if (us < PWM_MIN_US)
        us = PWM_MIN_US;
    else if (us > PWM_MAX_US)
        us = PWM_MAX_US;

    /* Convert microseconds to PWM value:
     * 4096 counts per period, period = 1000000/PCA9685_PWM_FREQ microseconds
     */
    pulse = (us * PCA9685_PWM_RES * PCA9685_PWM_FREQ) / 1000000UL;
    if (pulse > 4095)
        pulse = 4095;

    return pca9685_set_pwm(channel, 0, pulse);
}

/**
 * Clean up PCA9685 resources
 */
void pca9685_cleanup(void)
{
    if (pca9685_client)
    {
        /* Turn off all outputs */
        pca9685_write_reg(PCA9685_ALL_LED_OFF_L, 0);
        pca9685_write_reg(PCA9685_ALL_LED_OFF_H, 0x10); /* Full off */

        /* Unregister client */
        i2c_unregister_device(pca9685_client);
        pca9685_client = NULL;
    }

    if (pca9685_secondary_client)
    {
        /* Turn off all outputs */
        pca9685_write_reg(PCA9685_ALL_LED_OFF_L, 0);
        pca9685_write_reg(PCA9685_ALL_LED_OFF_H, 0x10); /* Full off */

        /* Unregister client */
        i2c_unregister_device(pca9685_secondary_client);
        pca9685_secondary_client = NULL;
    }
}

/* Export symbols */
EXPORT_SYMBOL_GPL(pca9685_init);
EXPORT_SYMBOL_GPL(pca9685_cleanup);
EXPORT_SYMBOL_GPL(pca9685_set_pwm);
EXPORT_SYMBOL_GPL(pca9685_set_pwm_freq);
EXPORT_SYMBOL_GPL(pca9685_set_pwm_us);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PCA9685 PWM Controller Driver");
MODULE_AUTHOR("StrongFood");