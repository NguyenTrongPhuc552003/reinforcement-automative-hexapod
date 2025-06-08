#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <linux/types.h>

/* I2C configuration */
#define MPU6050_I2C_BUS 3     /* BeagleBone I2C3 */
#define MPU6050_I2C_ADDR 0x68 /* Default I2C address for MPU6050 */

/* IMU data structure - self-contained definition */
struct mpu6050_imu_data
{
    s16 accel_x; /* Accelerometer X-axis */
    s16 accel_y; /* Accelerometer Y-axis */
    s16 accel_z; /* Accelerometer Z-axis */
    s16 gyro_x;  /* Gyroscope X-axis */
    s16 gyro_y;  /* Gyroscope Y-axis */
    s16 gyro_z;  /* Gyroscope Z-axis */
};

/* MPU6050 configuration constants */
/* Gyroscope range options */
#define MPU6050_GYRO_FS_250 0x00  /* ±250 degrees/second */
#define MPU6050_GYRO_FS_500 0x01  /* ±500 degrees/second */
#define MPU6050_GYRO_FS_1000 0x02 /* ±1000 degrees/second */
#define MPU6050_GYRO_FS_2000 0x03 /* ±2000 degrees/second */

/* Accelerometer range options */
#define MPU6050_ACCEL_FS_2G 0x00  /* ±2g */
#define MPU6050_ACCEL_FS_4G 0x01  /* ±4g */
#define MPU6050_ACCEL_FS_8G 0x02  /* ±8g */
#define MPU6050_ACCEL_FS_16G 0x03 /* ±16g */

/* Digital low-pass filter options */
#define MPU6050_DLPF_DISABLED 0x00 /* Disabled */
#define MPU6050_DLPF_188HZ 0x01    /* 188 Hz */
#define MPU6050_DLPF_98HZ 0x02     /* 98 Hz */
#define MPU6050_DLPF_42HZ 0x03     /* 42 Hz */
#define MPU6050_DLPF_20HZ 0x04     /* 20 Hz */
#define MPU6050_DLPF_10HZ 0x05     /* 10 Hz */
#define MPU6050_DLPF_5HZ 0x06      /* 5 Hz */

/* Configuration structure - allows for easy parameter passing */
struct mpu6050_config
{
    u8 gyro_range;      /* Gyroscope range setting */
    u8 accel_range;     /* Accelerometer range setting */
    u8 dlpf_setting;    /* Digital low-pass filter setting */
    u8 sample_rate_div; /* Sample rate divider */
};

/**
 * Initialize the MPU6050 module
 *
 * @return 0 on success, negative error code on failure
 */
int mpu6050_init(void);

/**
 * Initialize the MPU6050 with specific configuration
 *
 * @param config Pointer to configuration structure
 * @return 0 on success, negative error code on failure
 */
int mpu6050_init_with_config(const struct mpu6050_config *config);

/**
 * Read sensor data from MPU6050
 *
 * @param data Pointer to data structure to fill with sensor readings
 * @return 0 on success, negative error code on failure
 */
int mpu6050_read_sensors(struct mpu6050_imu_data *data);

/**
 * Cleanup the MPU6050 module
 */
void mpu6050_cleanup(void);

/**
 * Get device status and information
 *
 * @return non-zero value if device is initialized, 0 if not
 */
int mpu6050_is_initialized(void);

#endif /* _MPU6050_H_ */
