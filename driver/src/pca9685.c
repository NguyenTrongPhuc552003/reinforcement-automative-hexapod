#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include "pca9685.h"

#define PCA9685_RESET_DELAY 10   /* Delay after reset (ms) */
#define PCA9685_OSCSTART_DELAY 5 /* Delay after restart (ms) */

struct pca9685_controller
{
    struct i2c_client *client;
    int initialized;
};

/* Global state */
static struct
{
    struct pca9685_controller controllers[2]; /* Primary and secondary controllers */
    struct mutex lock;                        /* Lock for thread safety */
    u16 current_freq_hz;                      /* Current PWM frequency */
    int initialized;                          /* Overall initialization status */
} pca9685_state = {
    .current_freq_hz = PCA9685_PWM_FREQ_DEFAULT,
    .initialized = 0};

/* Controller selection helpers */
static inline struct i2c_client *pca9685_get_client(u8 channel)
{
    return (channel < PCA9685_CHANNELS_PER_DEVICE) ? pca9685_state.controllers[0].client : pca9685_state.controllers[1].client;
}

static inline u8 pca9685_get_channel_offset(u8 channel)
{
    return channel % PCA9685_CHANNELS_PER_DEVICE;
}

/**
 * Write to a PCA9685 register with error handling and retry
 *
 * @param client I2C client
 * @param reg Register address
 * @param val Register value
 * @return 0 on success, negative error code on failure
 */
static int pca9685_write_reg_client(struct i2c_client *client, u8 reg, u8 val)
{
    int ret;

    if (!client)
        return -EINVAL;

    ret = i2c_smbus_write_byte_data(client, reg, val);
    if (ret < 0)
        dev_err(&client->dev, "Failed to write register 0x%02x: %d\n", reg, ret);

    return ret;
}

/**
 * Write to a PCA9685 register by channel selection
 *
 * @param reg Register address
 * @param val Register value
 * @param channel Channel number to select controller (optional, -1 for both)
 * @return 0 on success, negative error code on failure
 */
static int pca9685_write_reg(u8 reg, u8 val, int channel)
{
    int ret = 0;

    if (channel < 0 || channel >= PCA9685_CHANNELS_PER_DEVICE)
    {
        /* Write to both controllers */
        if (pca9685_state.controllers[0].initialized)
        {
            ret = pca9685_write_reg_client(pca9685_state.controllers[0].client, reg, val);
            if (ret < 0)
                return ret;
        }

        if (pca9685_state.controllers[1].initialized)
        {
            ret = pca9685_write_reg_client(pca9685_state.controllers[1].client, reg, val);
        }
    }
    else
    {
        /* Write to specific controller based on channel */
        ret = pca9685_write_reg_client(pca9685_get_client(channel), reg, val);
    }

    return ret;
}

/**
 * Read from a PCA9685 register with error handling
 *
 * @param client I2C client
 * @param reg Register address
 * @return Register value on success, negative error code on failure
 */
static int pca9685_read_reg_client(struct i2c_client *client, u8 reg)
{
    int ret;

    if (!client)
        return -EINVAL;

    ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0)
        dev_err(&client->dev, "Failed to read register 0x%02x: %d\n", reg, ret);

    return ret;
}

/**
 * Read from a PCA9685 register using channel to select controller
 *
 * @param reg Register address
 * @param channel Channel number to select controller
 * @return Register value on success, negative error code on failure
 */
static __maybe_unused int pca9685_read_reg(u8 reg, u8 channel)
{
    struct i2c_client *client = pca9685_get_client(channel);
    return pca9685_read_reg_client(client, reg);
}

/**
 * Write block of data to a PCA9685 register with error handling
 *
 * @param client I2C client
 * @param reg Register address
 * @param len Length of data
 * @param buf Buffer containing data
 * @return 0 on success, negative error code on failure
 */
static int pca9685_write_block(struct i2c_client *client, u8 reg, u8 len, const u8 *buf)
{
    int ret;

    if (!client || !buf)
        return -EINVAL;

    if (i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WRITE_I2C_BLOCK))
    {
        ret = i2c_smbus_write_i2c_block_data(client, reg, len, buf);
        if (ret == 0)
            return 0;
    }
    else
    {
        /* Fall back to individual byte writes */
        int i;
        for (i = 0; i < len; i++)
        {
            ret = i2c_smbus_write_byte_data(client, reg + i, buf[i]);
            if (ret < 0)
                break;
        }
        if (i == len)
            return 0;
    }

    dev_err(&client->dev, "Failed to write block to register 0x%02x: %d\n", reg, ret);
    return ret;
}

/**
 * Initialize a PCA9685 controller
 *
 * @param client I2C client
 * @return 0 on success, negative error code on failure
 */
static int pca9685_init_controller(struct i2c_client *client)
{
    int ret;
    u8 prescale;

    if (!client)
        return -EINVAL;

    /* Software reset */
    ret = pca9685_write_reg_client(client, PCA9685_REG_MODE1, 0x80);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to reset PCA9685: %d\n", ret);
        return ret;
    }

    /* Allow time for reset to complete */
    schedule_timeout_interruptible(msecs_to_jiffies(PCA9685_RESET_DELAY));

    /* Set to sleep mode (required to change prescale) */
    ret = pca9685_write_reg_client(client, PCA9685_REG_MODE1, PCA9685_BIT_SLEEP);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to set sleep mode: %d\n", ret);
        return ret;
    }

    /* Calculate prescale for default frequency */
    prescale = (u8)((PCA9685_CLOCK_FREQ / (PCA9685_PWM_RES * pca9685_state.current_freq_hz)) - 1);

    /* Set prescale value */
    ret = pca9685_write_reg_client(client, PCA9685_REG_PRESCALE, prescale);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to set prescale: %d\n", ret);
        return ret;
    }

    /* Set to normal operation with auto-increment */
    ret = pca9685_write_reg_client(client, PCA9685_REG_MODE1, PCA9685_BIT_AI);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to set operating mode: %d\n", ret);
        return ret;
    }

    schedule_timeout_interruptible(msecs_to_jiffies(PCA9685_OSCSTART_DELAY));

    /* Set output mode to totem pole (default) */
    ret = pca9685_write_reg_client(client, PCA9685_REG_MODE2, PCA9685_BIT_OUTDRV);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to set output mode: %d\n", ret);
        return ret;
    }

    /* Turn off all outputs */
    ret = pca9685_write_reg_client(client, PCA9685_REG_ALL_LED_OFF_L, 0);
    if (ret < 0)
        return ret;

    ret = pca9685_write_reg_client(client, PCA9685_REG_ALL_LED_OFF_H, 0x10); /* Full off */
    if (ret < 0)
        return ret;

    dev_info(&client->dev, "PCA9685 at 0x%02x initialized\n", client->addr);
    return 0;
}

/**
 * Initialize the PCA9685 PWM controller
 *
 * @return 0 on success, negative error code on failure
 */
int pca9685_init(void)
{
    struct i2c_adapter *adapter;
    int ret;

    /* Check if already initialized */
    if (pca9685_state.initialized)
    {
        pr_info("PCA9685: Already initialized\n");
        return 0;
    }

    /* Initialize global state */
    mutex_init(&pca9685_state.lock);
    memset(pca9685_state.controllers, 0, sizeof(pca9685_state.controllers));
    pca9685_state.current_freq_hz = PCA9685_PWM_FREQ_DEFAULT;

    /* Get I2C adapter */
    adapter = i2c_get_adapter(PCA9685_I2C_BUS);
    if (!adapter)
    {
        pr_err("PCA9685: Failed to get I2C adapter %d\n", PCA9685_I2C_BUS);
        return -ENODEV;
    }

    /* Create I2C clients */
    mutex_lock(&pca9685_state.lock);

    /* Primary controller */
    pca9685_state.controllers[0].client = i2c_new_dummy(adapter, PCA9685_I2C_ADDR_1);
    if (!pca9685_state.controllers[0].client)
    {
        pr_err("PCA9685: Failed to create primary I2C client\n");
        mutex_unlock(&pca9685_state.lock);
        i2c_put_adapter(adapter);
        return -ENOMEM;
    }

    /* Secondary controller */
    pca9685_state.controllers[1].client = i2c_new_dummy(adapter, PCA9685_I2C_ADDR_2);
    if (!pca9685_state.controllers[1].client)
    {
        pr_err("PCA9685: Failed to create secondary I2C client\n");
        i2c_unregister_device(pca9685_state.controllers[0].client);
        mutex_unlock(&pca9685_state.lock);
        i2c_put_adapter(adapter);
        return -ENOMEM;
    }

    /* Safe to release adapter now that both clients are created */
    i2c_put_adapter(adapter);

    /* Initialize primary controller */
    ret = pca9685_init_controller(pca9685_state.controllers[0].client);
    if (ret < 0)
    {
        pr_err("PCA9685: Failed to initialize primary controller\n");
        goto error;
    }
    pca9685_state.controllers[0].initialized = 1;

    /* Initialize secondary controller */
    ret = pca9685_init_controller(pca9685_state.controllers[1].client);
    if (ret < 0)
    {
        pr_err("PCA9685: Failed to initialize secondary controller\n");
        goto error;
    }
    pca9685_state.controllers[1].initialized = 1;

    pca9685_state.initialized = 1;
    mutex_unlock(&pca9685_state.lock);

    pr_info("PCA9685: Dual controllers initialized at %d Hz\n", pca9685_state.current_freq_hz);
    return 0;

error:
    i2c_unregister_device(pca9685_state.controllers[0].client);
    i2c_unregister_device(pca9685_state.controllers[1].client);
    pca9685_state.controllers[0].client = NULL;
    pca9685_state.controllers[1].client = NULL;
    mutex_unlock(&pca9685_state.lock);
    return ret;
}
EXPORT_SYMBOL_GPL(pca9685_init);

/**
 * Set PWM frequency for all controllers
 *
 * @param freq_hz Frequency in Hz
 * @return 0 on success, negative error code on failure
 */
int pca9685_set_pwm_freq(u16 freq_hz)
{
    u8 prescale;
    int ret = 0;
    u8 oldmode1[2] = {0}; // Initialize array elements to zero
    u8 newmode1[2] = {0}; // Initialize array elements to zero

    if (!pca9685_state.initialized)
        return -EINVAL;

    if (freq_hz < PCA9685_PWM_FREQ_MIN || freq_hz > PCA9685_PWM_FREQ_MAX)
        return -EINVAL;

    /* Calculate prescale value */
    prescale = (u8)((PCA9685_CLOCK_FREQ / (PCA9685_PWM_RES * freq_hz)) - 1);

    mutex_lock(&pca9685_state.lock);

    /* Read current modes */
    if (pca9685_state.controllers[0].initialized)
    {
        oldmode1[0] = pca9685_read_reg_client(pca9685_state.controllers[0].client, PCA9685_REG_MODE1);
        if (oldmode1[0] < 0)
        {
            ret = oldmode1[0];
            goto unlock;
        }
    }

    if (pca9685_state.controllers[1].initialized)
    {
        oldmode1[1] = pca9685_read_reg_client(pca9685_state.controllers[1].client, PCA9685_REG_MODE1);
        if (oldmode1[1] < 0)
        {
            ret = oldmode1[1];
            goto unlock;
        }
    }

    /* Set sleep mode */
    if (pca9685_state.controllers[0].initialized)
    {
        newmode1[0] = (oldmode1[0] & ~PCA9685_BIT_RESTART) | PCA9685_BIT_SLEEP;
        ret = pca9685_write_reg_client(pca9685_state.controllers[0].client, PCA9685_REG_MODE1, newmode1[0]);
        if (ret < 0)
            goto unlock;
    }

    if (pca9685_state.controllers[1].initialized)
    {
        newmode1[1] = (oldmode1[1] & ~PCA9685_BIT_RESTART) | PCA9685_BIT_SLEEP;
        ret = pca9685_write_reg_client(pca9685_state.controllers[1].client, PCA9685_REG_MODE1, newmode1[1]);
        if (ret < 0)
            goto unlock;
    }

    /* Set prescale value */
    ret = pca9685_write_reg(PCA9685_REG_PRESCALE, prescale, -1);
    if (ret < 0)
        goto unlock;

    /* Restore previous mode without RESTART bit */
    if (pca9685_state.controllers[0].initialized)
    {
        ret = pca9685_write_reg_client(pca9685_state.controllers[0].client, PCA9685_REG_MODE1, oldmode1[0]);
        if (ret < 0)
            goto unlock;
    }

    if (pca9685_state.controllers[1].initialized)
    {
        ret = pca9685_write_reg_client(pca9685_state.controllers[1].client, PCA9685_REG_MODE1, oldmode1[1]);
        if (ret < 0)
            goto unlock;
    }

    /* Wait for oscillator to stabilize */
    schedule_timeout_interruptible(msecs_to_jiffies(PCA9685_OSCSTART_DELAY));

    /* Restart PWM */
    if (pca9685_state.controllers[0].initialized)
    {
        ret = pca9685_write_reg_client(pca9685_state.controllers[0].client,
                                       PCA9685_REG_MODE1,
                                       oldmode1[0] | PCA9685_BIT_RESTART);
        if (ret < 0)
            goto unlock;
    }

    if (pca9685_state.controllers[1].initialized)
    {
        ret = pca9685_write_reg_client(pca9685_state.controllers[1].client,
                                       PCA9685_REG_MODE1,
                                       oldmode1[1] | PCA9685_BIT_RESTART);
        if (ret < 0)
            goto unlock;
    }

    /* Store new frequency */
    pca9685_state.current_freq_hz = freq_hz;
    pr_info("PCA9685: Frequency set to %d Hz\n", freq_hz);

unlock:
    mutex_unlock(&pca9685_state.lock);
    return ret;
}
EXPORT_SYMBOL_GPL(pca9685_set_pwm_freq);

/**
 * Set PWM values for a specific channel
 *
 * @param channel Channel number (0-31)
 * @param on On-time count (0-4095)
 * @param off Off-time count (0-4095)
 * @return 0 on success, negative error code on failure
 */
int pca9685_set_pwm(u8 channel, u16 on, u16 off)
{
    int ret = 0;
    u8 reg;
    struct i2c_client *client;
    u8 data[4];

    if (!pca9685_state.initialized)
        return -EINVAL;

    if (channel >= PCA9685_MAX_CHANNELS || on > 4095 || off > 4095)
        return -EINVAL;

    /* Get the appropriate client */
    client = pca9685_get_client(channel);
    if (!client)
        return -ENODEV;

    /* Calculate register address for specific channel */
    reg = PCA9685_REG_LED0_ON_L + (pca9685_get_channel_offset(channel) * 4);

    /* Prepare data for all 4 registers */
    data[0] = on & 0xFF;
    data[1] = (on >> 8) & 0x0F;
    data[2] = off & 0xFF;
    data[3] = (off >> 8) & 0x0F;

    mutex_lock(&pca9685_state.lock);
    ret = pca9685_write_block(client, reg, 4, data);
    mutex_unlock(&pca9685_state.lock);

    return ret;
}
EXPORT_SYMBOL_GPL(pca9685_set_pwm);

/**
 * Set PWM pulse width in microseconds for a channel
 *
 * @param channel Channel number (0-31)
 * @param us Pulse width in microseconds
 * @return 0 on success, negative error code on failure
 */
int pca9685_set_pwm_us(u8 channel, u16 us)
{
    unsigned int pulse;

    if (!pca9685_state.initialized)
        return -EINVAL;

    if (channel >= PCA9685_MAX_CHANNELS)
        return -EINVAL;

    /* Enforce pulse width limits */
    if (us < PCA9685_PWM_MIN_US)
        us = PCA9685_PWM_MIN_US;
    else if (us > PCA9685_PWM_MAX_US)
        us = PCA9685_PWM_MAX_US;

    /* Convert microseconds to PWM value */
    pulse = (us * PCA9685_PWM_RES * pca9685_state.current_freq_hz) / 1000000UL;
    if (pulse > 4095)
        pulse = 4095;

    return pca9685_set_pwm(channel, 0, pulse);
}
EXPORT_SYMBOL_GPL(pca9685_set_pwm_us);

/**
 * Enable all PWM outputs on all controllers
 *
 * @return 0 on success, negative error code on failure
 */
int pca9685_enable_outputs(void)
{
    int ret;

    if (!pca9685_state.initialized)
        return -EINVAL;

    mutex_lock(&pca9685_state.lock);

    /* Set normal operation with auto-increment */
    ret = pca9685_write_reg(PCA9685_REG_MODE1, PCA9685_BIT_AI, -1);
    if (ret < 0)
        goto unlock;

    /* Set output mode to totem pole (stronger drive) */
    ret = pca9685_write_reg(PCA9685_REG_MODE2, PCA9685_BIT_OUTDRV, -1);
    if (ret < 0)
        goto unlock;

    /* Clear the "full off" state for all channels */
    ret = pca9685_write_reg(PCA9685_REG_ALL_LED_OFF_H, 0x00, -1);

unlock:
    mutex_unlock(&pca9685_state.lock);
    return ret;
}
EXPORT_SYMBOL_GPL(pca9685_enable_outputs);

/**
 * Release resources used by PCA9685
 */
void pca9685_cleanup(void)
{
    if (!pca9685_state.initialized)
        return;

    pr_info("PCA9685: Cleaning up resources\n");

    mutex_lock(&pca9685_state.lock);

    /* Turn off all channels */
    if (pca9685_state.controllers[0].client)
    {
        pca9685_write_reg_client(pca9685_state.controllers[0].client,
                                 PCA9685_REG_ALL_LED_OFF_L, 0);
        pca9685_write_reg_client(pca9685_state.controllers[0].client,
                                 PCA9685_REG_ALL_LED_OFF_H, 0x10);
        i2c_unregister_device(pca9685_state.controllers[0].client);
        pca9685_state.controllers[0].client = NULL;
        pca9685_state.controllers[0].initialized = 0;
    }

    if (pca9685_state.controllers[1].client)
    {
        pca9685_write_reg_client(pca9685_state.controllers[1].client,
                                 PCA9685_REG_ALL_LED_OFF_L, 0);
        pca9685_write_reg_client(pca9685_state.controllers[1].client,
                                 PCA9685_REG_ALL_LED_OFF_H, 0x10);
        i2c_unregister_device(pca9685_state.controllers[1].client);
        pca9685_state.controllers[1].client = NULL;
        pca9685_state.controllers[1].initialized = 0;
    }

    pca9685_state.initialized = 0;
    mutex_unlock(&pca9685_state.lock);
}
EXPORT_SYMBOL_GPL(pca9685_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PCA9685 PWM Controller Driver");
MODULE_AUTHOR("StrongFood");
MODULE_VERSION("1.0");