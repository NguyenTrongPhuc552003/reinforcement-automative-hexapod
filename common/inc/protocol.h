#ifndef HEXAPOD_PROTOCOL_H
#define HEXAPOD_PROTOCOL_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#else
#include <stdint.h>
#include <sys/ioctl.h>
#endif

/* Hexapod Configuration */
#define NUM_LEGS 6
#define NUM_JOINTS_PER_LEG 3
#define TOTAL_SERVOS (NUM_LEGS * NUM_JOINTS_PER_LEG)

/* Device Path */
#define HEXAPOD_DEVICE_PATH "/dev/hexapod"

/* Hardware Limits */
#define HIP_MIN_ANGLE -45
#define HIP_MAX_ANGLE 45
#define KNEE_MIN_ANGLE -90
#define KNEE_MAX_ANGLE 90
#define ANKLE_MIN_ANGLE -45
#define ANKLE_MAX_ANGLE 45

/* Hardware Addresses */
#define MPU6050_I2C_ADDR 0x68
#define PCA9685_I2C_ADDR_PRIMARY 0x40
#define PCA9685_I2C_ADDR_SECONDARY 0x70
#define HEXAPOD_I2C_BUS 3 /* Use I2C bus 3 */

/* IOCTL Commands */
#define HEXAPOD_IOC_MAGIC 'H'
#define SET_LEG_POSITION _IOW(HEXAPOD_IOC_MAGIC, 1, struct leg_command)
#define GET_IMU_DATA _IOR(HEXAPOD_IOC_MAGIC, 2, struct imu_data)
#define SET_CALIBRATION _IOW(HEXAPOD_IOC_MAGIC, 3, struct leg_calibration)

/* Data Structures */
struct leg_position
{
    int16_t hip;   /* Hip joint angle in centidegrees */
    int16_t knee;  /* Knee joint angle in centidegrees */
    int16_t ankle; /* Ankle joint angle in centidegrees */
};

struct leg_command
{
    uint8_t leg_num; /* Leg number (0-5) */
    struct leg_position position;
};

struct imu_data
{
    int16_t accel_x; /* Accelerometer X axis */
    int16_t accel_y; /* Accelerometer Y axis */
    int16_t accel_z; /* Accelerometer Z axis */
    int16_t gyro_x;  /* Gyroscope X axis */
    int16_t gyro_y;  /* Gyroscope Y axis */
    int16_t gyro_z;  /* Gyroscope Z axis */
};

struct leg_calibration
{
    uint8_t leg_num;
    int16_t hip_offset;
    int16_t knee_offset;
    int16_t ankle_offset;
};

/* Type conversion (for user space API compatibility) */
#ifndef __KERNEL__
/* Conversion functions for user space only */
static inline int16_t degree_to_centidegree(double degrees)
{
    return (int16_t)(degrees * 100.0);
}

static inline double centidegree_to_degree(int16_t centidegrees)
{
    return ((double)centidegrees) / 100.0;
}
#endif

#endif /* HEXAPOD_PROTOCOL_H */
