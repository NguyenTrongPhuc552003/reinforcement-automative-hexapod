#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/math64.h>
#include "mpu6050.h"

/* Helper functions for I2C operations */
static inline int mpu6050_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
    int ret = i2c_smbus_write_byte_data(client, reg, value);
    if (ret < 0)
        dev_err(&client->dev, "Write to reg 0x%02x failed: %d\n", reg, ret);
    return ret;
}

static inline int mpu6050_read_reg(struct i2c_client *client, u8 reg)
{
    int ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0)
        dev_err(&client->dev, "Read from reg 0x%02x failed: %d\n", reg, ret);
    return ret;
}

static int mpu6050_read_block(struct i2c_client *client, u8 reg, u8 len, u8 *buf)
{
    int ret = i2c_smbus_read_i2c_block_data(client, reg, len, buf);
    if (ret < 0)
        dev_err(&client->dev, "Block read from reg 0x%02x failed: %d\n", reg, ret);
    return ret;
}

/* Power management functions */
static int mpu6050_set_power_mode(struct i2c_client *client, bool on)
{
    int ret;
    u8 pwr_mgmt = on ? MPU6050_CLOCK_PLL : MPU6050_SLEEP_MODE;

    ret = mpu6050_write_reg(client, MPU6050_PWR_MGMT_1, pwr_mgmt);
    if (ret < 0)
        return ret;

    if (on)
        msleep(100); // Wait for oscillators to stabilize

    return 0;
}

/* Device initialization */
int mpu6050_init(struct i2c_client *client)
{
    int ret;
    int retries;
    u8 id = 0; /* Initialize to avoid warning */

    if (!client)
        return -EINVAL;

    /* Verify device ID with retries */
    retries = 3;
    while (retries--)
    {
        ret = mpu6050_read_reg(client, MPU6050_WHO_AM_I);
        if (ret < 0)
        {
            dev_err(&client->dev, "Failed to read WHO_AM_I: %d\n", ret);
            continue;
        }

        id = (u8)ret;
        if (id == MPU6050_DEVICE_ID || id == MPU6050_DEVICE_ID_ALT)
        {
            dev_info(&client->dev, "MPU6050 ID verified: 0x%02x\n", id);
            break;
        }
        msleep(10);
    }

    if (retries < 0)
    {
        dev_err(&client->dev, "Invalid device ID: %02x\n", id);
        return -ENODEV;
    }

    /* Reset device */
    ret = mpu6050_write_reg(client, MPU6050_PWR_MGMT_1, MPU6050_RESET);
    if (ret < 0)
        return ret;
    msleep(100);

    /* Initialize device */
    ret = mpu6050_set_power_mode(client, true);
    if (ret < 0)
        return ret;

    /* Configure sensors */
    ret = mpu6050_write_reg(client, MPU6050_GYRO_CONFIG, MPU6050_GYRO_FS_500);
    if (ret < 0)
        return ret;

    ret = mpu6050_write_reg(client, MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_2G);
    if (ret < 0)
        return ret;

    /* Set sample rate divider for 100Hz operation */
    ret = mpu6050_write_reg(client, MPU6050_SMPLRT_DIV, 9);
    if (ret < 0)
        return ret;

    /* Configure digital low-pass filter */
    ret = mpu6050_write_reg(client, MPU6050_CONFIG, MPU6050_DLPF_10HZ);
    if (ret < 0)
        return ret;

    dev_info(&client->dev, "MPU6050 initialized successfully\n");
    return 0;
}

/* Cleanup function */
void mpu6050_remove(struct i2c_client *client)
{
    mpu6050_set_power_mode(client, false);
}

/* Sensor reading with integer math for kernel */
int mpu6050_read_sensors(struct i2c_client *client, struct imu_data *data)
{
    u8 buf[14];
    int ret;
    // s16 raw_val;

    if (!client || !data)
        return -EINVAL;

    ret = mpu6050_read_block(client, MPU6050_ACCEL_XOUT_H, 14, buf);
    if (ret < 0)
        return ret;

    /* Raw data conversion - store raw values in kernel space */
    data->accel_x = (s16)((buf[0] << 8) | buf[1]);
    data->accel_y = (s16)((buf[2] << 8) | buf[3]);
    data->accel_z = (s16)((buf[4] << 8) | buf[5]);
    data->gyro_x = (s16)((buf[8] << 8) | buf[9]);
    data->gyro_y = (s16)((buf[10] << 8) | buf[11]);
    data->gyro_z = (s16)((buf[12] << 8) | buf[13]);

    return 0;
}

static struct i2c_client *mpu6050_client;

int mpu6050_probe(struct i2c_adapter *adapter)
{
    if (!adapter)
        return -EINVAL;

    mpu6050_client = i2c_new_dummy(adapter, MPU6050_I2C_ADDR);
    if (!mpu6050_client)
    {
        pr_err("Failed to create MPU6050 I2C client\n");
        return -ENOMEM;
    }

    return mpu6050_init(mpu6050_client);
}

void mpu6050_shutdown(void)
{
    if (mpu6050_client)
    {
        mpu6050_remove(mpu6050_client);
        i2c_unregister_device(mpu6050_client);
        mpu6050_client = NULL;
    }
}

int mpu6050_read_imu_data(struct imu_data *data)
{
    if (!mpu6050_client || !data)
        return -EINVAL;

    return mpu6050_read_sensors(mpu6050_client, data);
}

EXPORT_SYMBOL_GPL(mpu6050_init);
EXPORT_SYMBOL_GPL(mpu6050_remove);
EXPORT_SYMBOL_GPL(mpu6050_read_sensors);
EXPORT_SYMBOL_GPL(mpu6050_probe);
EXPORT_SYMBOL_GPL(mpu6050_shutdown);
EXPORT_SYMBOL_GPL(mpu6050_read_imu_data);
