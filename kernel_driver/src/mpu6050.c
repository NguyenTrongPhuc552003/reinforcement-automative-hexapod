#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "mpu6050.h"

/* MPU6050 registers */
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_GYRO_XOUT_H     0x43

/* Device info */
#define MPU6050_I2C_ADDR      0x68
#define MPU6050_DEVICE_ID_1   0x68  /* Common device ID */
#define MPU6050_DEVICE_ID_2   0x98  /* Alternate device ID for some MPU6050 versions */

/* Configuration values */
#define MPU6050_RESET         0x80  /* Reset bit in PWR_MGMT_1 */
#define MPU6050_CLOCK_PLL     0x01  /* PLL with X axis gyroscope reference */
#define MPU6050_DLPF_CFG      0x03  /* Digital Low Pass Filter ~44Hz */
#define MPU6050_FS_SEL        0x08  /* Gyro Full Scale = ±500°/s */
#define MPU6050_AFS_SEL       0x08  /* Accel Full Scale = ±4g */

static struct i2c_client *mpu6050_client;

/* Helper Functions */
static int mpu6050_write_reg(u8 reg, u8 val)
{
    int ret = i2c_smbus_write_byte_data(mpu6050_client, reg, val);
    if (ret < 0)
        pr_err("Failed to write 0x%02x to reg 0x%02x\n", val, reg);
    return ret;
}

static int mpu6050_read_reg(u8 reg)
{
    int ret = i2c_smbus_read_byte_data(mpu6050_client, reg);
    if (ret < 0)
        pr_err("Failed to read reg 0x%02x\n", reg);
    return ret;
}

/* Initialize MPU6050 */
int mpu6050_init(void)
{
    int ret;
    struct i2c_adapter *adapter;
    u8 device_id;

    /* Get I2C adapter for bus 3 */
    adapter = i2c_get_adapter(3);
    if (!adapter) {
        pr_err("Failed to get I2C adapter\n");
        return -ENODEV;
    }

    /* Create I2C client */
    mpu6050_client = i2c_new_dummy(adapter, MPU6050_I2C_ADDR);
    i2c_put_adapter(adapter);

    if (!mpu6050_client) {
        pr_err("Failed to create I2C client\n");
        return -ENOMEM;
    }

    /* Check device ID */
    device_id = mpu6050_read_reg(MPU6050_WHO_AM_I);
    if (device_id != MPU6050_DEVICE_ID_1 && device_id != MPU6050_DEVICE_ID_2) {
        pr_err("Invalid device ID: 0x%02x (expected 0x%02x or 0x%02x)\n",
               device_id, MPU6050_DEVICE_ID_1, MPU6050_DEVICE_ID_2);
        ret = -ENODEV;
        goto err_remove;
    }
    pr_info("MPU6050 device ID: 0x%02x\n", device_id);

    /* Reset device */
    ret = mpu6050_write_reg(MPU6050_PWR_MGMT_1, MPU6050_RESET);
    if (ret < 0)
        goto err_remove;
    msleep(100);  /* Wait for reset to complete */

    /* Wake up device and set clock source */
    ret = mpu6050_write_reg(MPU6050_PWR_MGMT_1, MPU6050_CLOCK_PLL);
    if (ret < 0)
        goto err_remove;

    /* Set DLPF config */
    ret = mpu6050_write_reg(MPU6050_CONFIG, MPU6050_DLPF_CFG);
    if (ret < 0)
        goto err_remove;

    /* Configure gyroscope range */
    ret = mpu6050_write_reg(MPU6050_GYRO_CONFIG, MPU6050_FS_SEL);
    if (ret < 0)
        goto err_remove;

    /* Configure accelerometer range */
    ret = mpu6050_write_reg(MPU6050_ACCEL_CONFIG, MPU6050_AFS_SEL);
    if (ret < 0)
        goto err_remove;

    pr_info("MPU6050 initialized successfully\n");
    return 0;

err_remove:
    i2c_unregister_device(mpu6050_client);
    return ret;
}

/* Read all sensor data */
int mpu6050_read_all(s16 *accel_x, s16 *accel_y, s16 *accel_z,
                     s16 *gyro_x, s16 *gyro_y, s16 *gyro_z)
{
    u8 data[14];
    int ret;

    /* Read all sensor data in one transaction */
    ret = i2c_smbus_read_i2c_block_data(mpu6050_client,
                                       MPU6050_ACCEL_XOUT_H,
                                       14, data);
    if (ret < 0) {
        pr_err("Failed to read sensor data\n");
        return ret;
    }

    /* Combine high and low bytes */
    *accel_x = (s16)((data[0] << 8) | data[1]);
    *accel_y = (s16)((data[2] << 8) | data[3]);
    *accel_z = (s16)((data[4] << 8) | data[5]);
    /* Skip temperature data at data[6] and data[7] */
    *gyro_x = (s16)((data[8] << 8) | data[9]);
    *gyro_y = (s16)((data[10] << 8) | data[11]);
    *gyro_z = (s16)((data[12] << 8) | data[13]);

    return 0;
}

/* Cleanup MPU6050 */
void mpu6050_cleanup(void)
{
    if (mpu6050_client) {
        /* Put device in sleep mode */
        mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x40);
        i2c_unregister_device(mpu6050_client);
    }
}

EXPORT_SYMBOL_GPL(mpu6050_init);
EXPORT_SYMBOL_GPL(mpu6050_cleanup);
EXPORT_SYMBOL_GPL(mpu6050_read_all);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("StrongFood");
MODULE_DESCRIPTION("MPU6050 IMU Driver");
MODULE_VERSION("1.0");
