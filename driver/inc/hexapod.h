#ifndef _HEXAPOD_H_
#define _HEXAPOD_H_

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/mutex.h>

/**
 * @brief Robot configuration constants
 */
#define NUM_LEGS 6
#define NUM_JOINTS_PER_LEG 3
#define TOTAL_SERVOS (NUM_LEGS * NUM_JOINTS_PER_LEG)

/**
 * @brief IOCTL command definitions
 */
#define HEXAPOD_IOC_MAGIC 'H'
#define HEXAPOD_IOCTL_SET_LEG _IOW(HEXAPOD_IOC_MAGIC, 1, struct hexapod_leg_cmd)
#define HEXAPOD_IOCTL_GET_IMU _IOR(HEXAPOD_IOC_MAGIC, 2, struct hexapod_imu_data)
#define HEXAPOD_IOCTL_CALIBRATE _IOW(HEXAPOD_IOC_MAGIC, 3, struct hexapod_calibration)
#define HEXAPOD_IOCTL_CENTER_ALL _IO(HEXAPOD_IOC_MAGIC, 4)
#define HEXAPOD_IOCTL_MAX 4

/**
 * @brief Leg joint position data structure
 */
struct hexapod_leg_position
{
    s16 hip;   /**< Hip joint angle in degrees */
    s16 knee;  /**< Knee joint angle in degrees */
    s16 ankle; /**< Ankle joint angle in degrees */
};

/**
 * @brief Leg command data structure
 */
struct hexapod_leg_cmd
{
    u8 leg_num;                           /**< Leg number (0-5) */
    struct hexapod_leg_position position; /**< Target position */
};

/**
 * @brief IMU sensor data structure
 */
struct hexapod_imu_data
{
    s16 accel_x; /**< X-axis acceleration */
    s16 accel_y; /**< Y-axis acceleration */
    s16 accel_z; /**< Z-axis acceleration */
    s16 gyro_x;  /**< X-axis rotation rate */
    s16 gyro_y;  /**< Y-axis rotation rate */
    s16 gyro_z;  /**< Z-axis rotation rate */
};

/**
 * @brief Calibration parameters structure
 */
struct hexapod_calibration
{
    u8 leg_num;       /**< Leg number (0-5) */
    s16 hip_offset;   /**< Hip joint offset in degrees */
    s16 knee_offset;  /**< Knee joint offset in degrees */
    s16 ankle_offset; /**< Ankle joint offset in degrees */
};

/**
 * @brief Hexapod device data structure
 */
struct hexapod_data
{
    struct mutex lock;                                /**< Synchronization lock */
    struct hexapod_leg_position positions[NUM_LEGS];  /**< Current leg positions */
    struct hexapod_calibration calibration[NUM_LEGS]; /**< Calibration values */
    int initialized;                                 /**< Driver initialization flag */
};

/**
 * @brief Set a leg position
 *
 * @param dev Device data structure
 * @param leg Leg number (0-5)
 * @param pos Target position
 * @return 0 on success, negative error code on failure
 */
int hexapod_set_leg_position(struct hexapod_data *dev, u8 leg,
                             struct hexapod_leg_position *pos);

/**
 * @brief Read IMU sensor data
 *
 * @param dev Device data structure
 * @param data Pointer to store IMU data
 * @return 0 on success, negative error code on failure
 */
int hexapod_get_imu_data(struct hexapod_data *dev, struct hexapod_imu_data *data);

/**
 * @brief Set calibration offsets for a leg
 *
 * @param dev Device data structure
 * @param leg Leg number (0-5)
 * @param cal Calibration parameters
 * @return 0 on success, negative error code on failure
 */
int hexapod_set_calibration(struct hexapod_data *dev, u8 leg,
                            struct hexapod_calibration *cal);

/**
 * @brief Center all servos to their neutral positions
 *
 * @param dev Device data structure
 * @return 0 on success, negative error code on failure
 */
int hexapod_center_all(struct hexapod_data *dev);

/* External global device data */
extern struct hexapod_data hexapod_dev;

#endif /* _HEXAPOD_H_ */
