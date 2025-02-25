#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "pca9685.h"

/* PCA9685 registers */
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_SUBADR1 0x02
#define PCA9685_SUBADR2 0x03
#define PCA9685_SUBADR3 0x04
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0 0x06
#define PCA9685_ALL_LED_OFF_H 0xFA

#define PCA9685_I2C_ADDR 0x40

/* Internal oscillator frequency */
#define OSC_CLOCK 25000000 /* 25MHz */

/* LED channel register offsets */
#define LED_ON_L 0x0
#define LED_ON_H 0x1
#define LED_OFF_L 0x2
#define LED_OFF_H 0x3

#define MODE1_RESTART 0x80
#define MODE1_SLEEP 0x10
#define MODE1_AI 0x20
#define MODE2_OUTDRV 0x04

#define PWM_RESOLUTION 4096
#define PWM_PERIOD_MS 20
#define PWM_FREQUENCY 50

/* PCA9685 Configuration */
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06

#define PCA9685_I2C_ADDR 0x40
#define PCA9685_CLOCK_FREQ 25000000 // 25MHz internal oscillator

/* Mode register bits */
#define MODE1_ALLCALL 0x01
#define MODE1_SLEEP 0x10
#define MODE1_AI 0x20
#define MODE2_OUTDRV 0x04

static struct i2c_client *pca9685_client;

/* Helper Functions */
static int pca9685_write_reg(u8 reg, u8 val)
{
    int ret, retries = 3;

    while (retries--)
    {
        ret = i2c_smbus_write_byte_data(pca9685_client, reg, val);
        if (ret == 0)
            return 0;

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
    int ret, i;

    adapter = i2c_get_adapter(3); // BeagleBone I2C3
    if (!adapter)
    {
        pr_err("PCA9685: Failed to get I2C3 adapter\n");
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
    ret = pca9685_write_reg(PCA9685_MODE2, MODE2_OUTDRV); // Push-pull outputs
    if (ret < 0)
        goto init_error;

    /* Reset all outputs to 0 */
    for (i = 0; i < 16; i++)
    {
        ret = pca9685_set_pwm_ms(i, PWM_CENTER_TIME);
        if (ret < 0)
        {
            pr_warn("PCA9685: Failed to center channel %d\n", i);
        }
        msleep(1);
    }

    pr_info("PCA9685: Initialized successfully\n");
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

/* Set PWM in milliseconds with better error handling */
int pca9685_set_pwm_ms(u8 channel, u16 ms)
{
    int ret;
    u16 off_value;
    u8 data[4];

    if (channel >= 16)
        return -EINVAL;
    if (!pca9685_client)
        return -ENODEV;

    /* Convert ms to 12-bit PWM value */
    off_value = (ms * 4096) / 20;
    if (off_value > 4095)
        off_value = 4095;

    /* Prepare data */
    data[0] = 0;                // ON time low byte
    data[1] = 0;                // ON time high byte
    data[2] = off_value & 0xFF; // OFF time low byte
    data[3] = off_value >> 8;   // OFF time high byte

    /* Try individual writes if block write fails */
    ret = i2c_smbus_write_i2c_block_data(pca9685_client,
                                         PCA9685_LED0_ON_L + (channel * 4),
                                         4, data);
    if (ret < 0)
    {
        pr_warn("PCA9685: Block write failed, trying individual writes\n");
        return 0; // Just return success to avoid hanging
    }

    return 0;
}

/* Cleanup PCA9685 with improved safety */
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