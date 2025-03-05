#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "pca9685.h"

static struct i2c_client *pca9685_client;

/* Helper Functions */
static int pca9685_write_reg(u8 reg, u8 val)
{
    int ret, retries = 3;

    if (!pca9685_client)
        return -ENODEV;

    while (retries--)
    {
        ret = i2c_smbus_write_byte_data(pca9685_client, reg, val);
        if (ret == 0)
        {
            pr_debug("PCA9685: Write success: reg=0x%02x val=0x%02x\n", reg, val);
            return 0;
        }

        pr_warn("PCA9685: Write retry %d: reg=0x%02x val=0x%02x ret=%d\n",
                2 - retries, reg, val, ret);
        msleep(10); // Wait 10ms between retries
    }

    pr_err("PCA9685: Write failed after retries: reg=0x%02x val=0x%02x ret=%d\n",
           reg, val, ret);
    return ret;
}

static int pca9685_read_reg(u8 reg)
{
    int ret, retries = 3;

    if (!pca9685_client)
        return -ENODEV;

    while (retries--)
    {
        ret = i2c_smbus_read_byte_data(pca9685_client, reg);
        if (ret >= 0)
            return ret;

        pr_warn("PCA9685: Read retry %d: reg=0x%02x ret=%d\n",
                2 - retries, reg, ret);
        msleep(10); // Wait 10ms between retries
    }

    pr_err("PCA9685: Read failed after retries: reg=0x%02x ret=%d\n",
           reg, ret);
    return ret;
}

/* Initialize PCA9685 */
int pca9685_init(void)
{
    struct i2c_adapter *adapter;
    int ret;
    int i;
    // u8 oldmode;

    adapter = i2c_get_adapter(PCA9685_I2C_BUS); // BeagleBone I2C3
    if (!adapter)
    {
        pr_err("PCA9685: Failed to get I2C%d adapter\n", PCA9685_I2C_BUS);
        return -ENODEV;
    }

    pca9685_client = i2c_new_dummy(adapter, PCA9685_I2C_ADDR);
    i2c_put_adapter(adapter);

    if (!pca9685_client)
    {
        pr_err("PCA9685: Failed to create I2C client\n");
        return -ENOMEM;
    }

    /* Software reset and initialization sequence */
    ret = pca9685_write_reg(PCA9685_MODE1, 0x00); // Normal mode
    if (ret < 0)
        goto init_error;
    msleep(1);

    /* Set prescaler for 50Hz (20ms) */
    ret = pca9685_write_reg(PCA9685_MODE1, MODE1_SLEEP); // Must sleep before changing prescaler
    if (ret < 0)
        goto init_error;
    msleep(1);

    ret = pca9685_write_reg(PCA9685_PRESCALE, 121); // 25MHz/(4096*50Hz)-1 ≈ 121
    if (ret < 0)
        goto init_error;

    ret = pca9685_write_reg(PCA9685_MODE1, MODE1_AI); // Auto-increment, wake up
    if (ret < 0)
        goto init_error;
    msleep(1);

    /* Configure output mode */
    ret = pca9685_write_reg(PCA9685_MODE2, PCA9685_OUTDRV); // Push-pull outputs
    if (ret < 0)
        goto init_error;

    /* Reset all outputs to center position */
    for (i = 0; i < 16; i++)
    {
        ret = pca9685_set_pwm_ms(i, PWM_CENTER_TIME);
        if (ret < 0)
        {
            pr_warn("PCA9685: Failed to center channel %d\n", i);
        }
        msleep(1);
    }

    pr_info("PCA9685: Initialized successfully at address 0x%02X on I2C bus %d\n",
            PCA9685_I2C_ADDR, PCA9685_I2C_BUS);
    return 0;

init_error:
    pr_err("PCA9685: Initialization failed (ret=%d)\n", ret);
    i2c_unregister_device(pca9685_client);
    pca9685_client = NULL;
    return ret;
}

/* Set PWM frequency */
int pca9685_set_pwm_freq(u16 freq)
{
    u8 prescale;
    u8 oldmode;
    int ret;

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

    if (channel > 15)
        return -EINVAL;

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
    u16 off_value;

    if (channel > 15)
        return -EINVAL;

    if (!pca9685_client)
        return -ENODEV;

    /* Convert ms to 12-bit PWM value */
    off_value = (ms * PWM_RESOLUTION) / 20000;
    if (off_value > 4095)
        off_value = 4095;

    return pca9685_set_pwm(channel, 0, off_value);
}

/* Cleanup PCA9685 */
void pca9685_cleanup(void)
{
    if (pca9685_client)
    {
        /* Don't try to write to device during cleanup */
        i2c_unregister_device(pca9685_client);
        pca9685_client = NULL;
    }
}

/* Export symbols for other parts of the driver */
EXPORT_SYMBOL_GPL(pca9685_init);
EXPORT_SYMBOL_GPL(pca9685_cleanup);
EXPORT_SYMBOL_GPL(pca9685_set_pwm);
EXPORT_SYMBOL_GPL(pca9685_set_pwm_ms);
EXPORT_SYMBOL_GPL(pca9685_set_pwm_freq);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("PCA9685 16-channel PWM Driver");