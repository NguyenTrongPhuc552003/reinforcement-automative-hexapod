/**
 * @file main.c
 * @brief Comprehensive hexapod robot device driver for Linux kernel
 *
 * This driver provides complete control functionality for a hexapod robot with:
 * - 6 legs with 3 servos each (18 total servos via PCA9685)
 * - IMU sensor support (MPU6050 or ADXL345)
 * - Calibration and position control
 * - Character device interface with IOCTL commands
 *
 * The driver integrates all hexapod-specific logic directly into the kernel
 * module, avoiding unnecessary abstraction layers while reusing existing
 * device-specific drivers (PCA9685, MPU6050, ADXL345, servo control).
 *
 * Hardware Interface:
 * - PCA9685: 16-channel PWM controller for servo motors
 * - MPU6050: 6-axis IMU (accelerometer + gyroscope)
 * - ADXL345: 3-axis accelerometer (alternative to MPU6050)
 * - GPIO: Optional status/debug pins
 *
 * @author Hexapod Development Team
 * @version 2.0
 * @date 2024
 * @license GPL v2
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/timer.h>

/* Include headers for existing device-specific drivers */
#include "pca9685.h"
#include "adxl345.h"
#include "mpu6050.h"
#include "servo.h"

/*============================================================================
 * Module Metadata and Parameters
 *============================================================================*/

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("StrongFood");
MODULE_DESCRIPTION("A big update for hexapod robot driver with complete functionality");
MODULE_VERSION("2.0");

/**
 * @brief Module debug parameter
 *
 * Enable/disable debug output during module operation
 * Usage: insmod hexapod_driver.ko debug=1
 */
static int debug = 0;
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debug output (0=disabled, 1=enabled)");

/**
 * @brief Default sensor type parameter
 *
 * Set which IMU sensor to use by default
 * 0=MPU6050, 1=ADXL345, 2=Auto-detect
 */
static int default_sensor = 2;
module_param(default_sensor, int, S_IRUGO);
MODULE_PARM_DESC(default_sensor, "Default sensor type (0=MPU6050, 1=ADXL345, 2=auto)");

/**
 * @brief Servo update frequency parameter
 *
 * PWM update frequency for servo control (Hz)
 */
static int servo_freq = SERVO_FREQ_HZ;
module_param(servo_freq, int, S_IRUGO);
MODULE_PARM_DESC(servo_freq, "Servo PWM frequency in Hz (default: 50)");

/*============================================================================
 * Hardware Configuration Constants
 *============================================================================*/

/**
 * @brief Device naming constants
 */
#define DEVICE_NAME "hexapod"
#define CLASS_NAME "hexapod"
#define DRIVER_NAME "hexapod_driver"

/**
 * @brief Hexapod physical configuration
 *
 * These constants define the robot's mechanical structure
 */
#define HEXAPOD_NUM_LEGS 6                                               ///< Number of legs
#define HEXAPOD_SERVOS_PER_LEG 3                                         ///< Servos per leg (hip, knee, ankle)
#define HEXAPOD_TOTAL_SERVOS (HEXAPOD_NUM_LEGS * HEXAPOD_SERVOS_PER_LEG) ///< Total servo count

/**
 * @brief Hexapod mechanical dimensions (millimeters)
 *
 * These values must match the physical robot construction
 */
#define HEXAPOD_COXA_LENGTH 30   ///< Hip segment length (mm)
#define HEXAPOD_FEMUR_LENGTH 85  ///< Upper leg segment length (mm)
#define HEXAPOD_TIBIA_LENGTH 130 ///< Lower leg segment length (mm)

/**
 * @brief Servo angle limits (degrees)
 *
 * Safety limits to prevent mechanical damage
 */
#define HEXAPOD_HIP_MIN_ANGLE -90   ///< Minimum hip angle
#define HEXAPOD_HIP_MAX_ANGLE 90    ///< Maximum hip angle
#define HEXAPOD_KNEE_MIN_ANGLE -90  ///< Minimum knee angle
#define HEXAPOD_KNEE_MAX_ANGLE 90   ///< Maximum knee angle
#define HEXAPOD_ANKLE_MIN_ANGLE -90 ///< Minimum ankle angle
#define HEXAPOD_ANKLE_MAX_ANGLE 90  ///< Maximum ankle angle

/**
 * @brief Servo PWM configuration
 *
 * PWM pulse width mappings for servo control
 */
#define SERVO_PWM_MIN 205    ///< PWM value for -90 degrees (1ms pulse)
#define SERVO_PWM_CENTER 307 ///< PWM value for 0 degrees (1.5ms pulse)
#define SERVO_PWM_MAX 409    ///< PWM value for +90 degrees (2ms pulse)

/**
 * @brief Sensor type enumeration
 *
 * Supported IMU sensor types
 */
typedef enum
{
    HEXAPOD_SENSOR_MPU6050 = 0, ///< MPU6050 6-axis IMU
    HEXAPOD_SENSOR_ADXL345 = 1, ///< ADXL345 3-axis accelerometer
    HEXAPOD_SENSOR_AUTO = 2     ///< Auto-detect available sensor
} hexapod_sensor_type_t;

/*============================================================================
 * IOCTL Interface Definitions
 *============================================================================*/

/**
 * @brief IOCTL magic number for hexapod driver
 *
 * Unique identifier for this driver's IOCTL commands
 */
#define HEXAPOD_IOC_MAGIC 'H'

/**
 * @brief Joint angles structure for leg control
 *
 * Contains angle values for all three joints in a leg
 */
struct hexapod_leg_joint
{
    int16_t hip;   ///< Hip joint angle in degrees (-90 to +90)
    int16_t knee;  ///< Knee joint angle in degrees (-90 to +90)
    int16_t ankle; ///< Ankle joint angle in degrees (-90 to +90)
};

/**
 * @brief Leg command structure for IOCTL interface
 *
 * Used to specify which leg to control and target angles
 */
struct hexapod_leg_cmd
{
    uint8_t leg_num;                ///< Leg number (0-5)
    struct hexapod_leg_joint joint; ///< Target joint angles
};

/**
 * @brief IMU data structure for sensor readings
 *
 * Contains raw accelerometer and gyroscope data
 */
struct hexapod_imu_data
{
    int16_t accel_x; ///< X-axis acceleration (raw sensor value)
    int16_t accel_y; ///< Y-axis acceleration (raw sensor value)
    int16_t accel_z; ///< Z-axis acceleration (raw sensor value)
    int16_t gyro_x;  ///< X-axis angular velocity (raw, 0 for ADXL345)
    int16_t gyro_y;  ///< Y-axis angular velocity (raw, 0 for ADXL345)
    int16_t gyro_z;  ///< Z-axis angular velocity (raw, 0 for ADXL345)
};

/**
 * @brief Calibration data structure for leg adjustment
 *
 * Contains offset values to compensate for mechanical variations
 */
struct hexapod_calibration
{
    uint8_t leg_num;                  ///< Leg number (0-5)
    struct hexapod_leg_joint offsets; ///< Calibration offset angles
};

/**
 * @brief IOCTL command definitions
 *
 * These commands provide the user-space API for robot control
 */
#define HEXAPOD_IOCTL_SET_LEG _IOW(HEXAPOD_IOC_MAGIC, 1, struct hexapod_leg_cmd)       ///< Set leg position
#define HEXAPOD_IOCTL_GET_IMU _IOR(HEXAPOD_IOC_MAGIC, 2, struct hexapod_imu_data)      ///< Get IMU data
#define HEXAPOD_IOCTL_CALIBRATE _IOW(HEXAPOD_IOC_MAGIC, 3, struct hexapod_calibration) ///< Set calibration
#define HEXAPOD_IOCTL_CENTER_ALL _IO(HEXAPOD_IOC_MAGIC, 4)                             ///< Center all legs
#define HEXAPOD_IOCTL_SET_SENSOR_TYPE _IOW(HEXAPOD_IOC_MAGIC, 5, int)                  ///< Set sensor type
#define HEXAPOD_IOCTL_GET_SENSOR_TYPE _IOR(HEXAPOD_IOC_MAGIC, 6, int)                  ///< Get sensor type

/*============================================================================
 * Device State Structure
 *============================================================================*/

/**
 * @brief Main hexapod device state structure
 *
 * Contains all runtime state and configuration for the hexapod device
 */
struct hexapod_device
{
    /* Character device infrastructure */
    struct cdev cdev;      ///< Character device structure
    struct class *class;   ///< Device class pointer
    struct device *device; ///< Device structure pointer
    dev_t devno;           ///< Device number

    /* Hardware state */
    bool pca9685_initialized;          ///< PCA9685 PWM controller status
    bool sensor_initialized;           ///< IMU sensor initialization status
    hexapod_sensor_type_t sensor_type; ///< Current active sensor type

    /* Current robot state */
    struct hexapod_leg_joint leg_positions[HEXAPOD_NUM_LEGS]; ///< Current leg positions
    struct hexapod_leg_joint calibration[HEXAPOD_NUM_LEGS];   ///< Calibration offsets
    struct hexapod_imu_data imu_data;                         ///< Last IMU reading

    /* Synchronization */
    struct mutex device_lock; ///< Device access mutex

    /* Operational statistics */
    unsigned long cmd_count;   ///< Total commands processed
    unsigned long error_count; ///< Total errors encountered
    ktime_t last_update;       ///< Time of last state update
};

/*============================================================================
 * Global Variables
 *============================================================================*/

/**
 * @brief Global device state pointer
 *
 * Single instance of the hexapod device
 */
static struct hexapod_device *hexapod_dev = NULL;

/**
 * @brief Device number for character device
 */
static dev_t hexapod_major;

/*============================================================================
 * Debug and Logging Macros
 *============================================================================*/

/**
 * @brief Debug print macro
 *
 * Prints debug messages when debug parameter is enabled
 */
#define hexapod_dbg(fmt, ...)                             \
    do                                                    \
    {                                                     \
        if (debug)                                        \
            pr_info(DRIVER_NAME ": " fmt, ##__VA_ARGS__); \
    } while (0)

/**
 * @brief Error print macro
 *
 * Always prints error messages with consistent formatting
 */
#define hexapod_err(fmt, ...) \
    pr_err(DRIVER_NAME ": " fmt, ##__VA_ARGS__)

/**
 * @brief Warning print macro
 *
 * Prints warning messages with consistent formatting
 */
#define hexapod_warn(fmt, ...) \
    pr_warn(DRIVER_NAME ": " fmt, ##__VA_ARGS__)

/**
 * @brief Info print macro
 *
 * Prints informational messages with consistent formatting
 */
#define hexapod_info(fmt, ...) \
    pr_info(DRIVER_NAME ": " fmt, ##__VA_ARGS__)

/*============================================================================
 * Utility Functions
 *============================================================================*/

/**
 * @brief Convert servo angle to PWM pulse width
 *
 * Maps an angle in degrees to the appropriate PWM pulse width value
 * for servo control via PCA9685
 *
 * @param angle Angle in degrees (-90 to +90)
 * @return PWM pulse width value (205-409)
 */
static uint16_t hexapod_angle_to_pwm(int16_t angle)
{
    int32_t pwm_value;

    /* Clamp angle to valid range for safety */
    if (angle < -90)
        angle = -90;
    if (angle > 90)
        angle = 90;

    /* Linear mapping: [-90, +90] -> [205, 409]
     * Formula: PWM = ((angle + 90) * (409 - 205) / 180) + 205
     */
    pwm_value = ((angle + 90) * (SERVO_PWM_MAX - SERVO_PWM_MIN) / 180) + SERVO_PWM_MIN;

    hexapod_dbg("Angle %d mapped to PWM %d", angle, pwm_value);

    return (uint16_t)pwm_value;
}

/**
 * @brief Validate leg number
 *
 * Checks if the provided leg number is within valid range
 *
 * @param leg_num Leg number to validate
 * @return true if valid, false otherwise
 */
static bool hexapod_is_valid_leg(uint8_t leg_num)
{
    return (leg_num < HEXAPOD_NUM_LEGS);
}

/**
 * @brief Validate joint angles
 *
 * Checks if all joint angles are within safe operating ranges
 *
 * @param joint Pointer to joint angles structure
 * @return true if all angles are valid, false otherwise
 */
static bool hexapod_validate_joint_angles(const struct hexapod_leg_joint *joint)
{
    if (!joint)
    {
        hexapod_err("NULL joint pointer");
        return false;
    }

    /* Check hip angle range */
    if (joint->hip < HEXAPOD_HIP_MIN_ANGLE || joint->hip > HEXAPOD_HIP_MAX_ANGLE)
    {
        hexapod_err("Hip angle %d out of range [%d, %d]",
                    joint->hip, HEXAPOD_HIP_MIN_ANGLE, HEXAPOD_HIP_MAX_ANGLE);
        return false;
    }

    /* Check knee angle range */
    if (joint->knee < HEXAPOD_KNEE_MIN_ANGLE || joint->knee > HEXAPOD_KNEE_MAX_ANGLE)
    {
        hexapod_err("Knee angle %d out of range [%d, %d]",
                    joint->knee, HEXAPOD_KNEE_MIN_ANGLE, HEXAPOD_KNEE_MAX_ANGLE);
        return false;
    }

    /* Check ankle angle range */
    if (joint->ankle < HEXAPOD_ANKLE_MIN_ANGLE || joint->ankle > HEXAPOD_ANKLE_MAX_ANGLE)
    {
        hexapod_err("Ankle angle %d out of range [%d, %d]",
                    joint->ankle, HEXAPOD_ANKLE_MIN_ANGLE, HEXAPOD_ANKLE_MAX_ANGLE);
        return false;
    }

    return true;
}

/*============================================================================
 * Hardware Initialization Functions
 *============================================================================*/

/**
 * @brief Initialize PCA9685 PWM controller
 *
 * Sets up the PCA9685 device for servo control with appropriate frequency
 * and configuration. Uses the existing pca9685.h API functions.
 *
 * @return 0 on success, negative error code on failure
 */
static int hexapod_init_pca9685(void)
{
    int ret;

    hexapod_dbg("Initializing PCA9685 PWM controller");

    /* Initialize PCA9685 using existing driver API */
    ret = pca9685_init();
    if (ret < 0)
    {
        hexapod_err("Failed to initialize PCA9685: %d", ret);
        return ret;
    }

    /* Set PWM frequency for servo control */
    ret = pca9685_set_pwm_freq(servo_freq);
    if (ret < 0)
    {
        hexapod_err("Failed to set PCA9685 frequency to %d Hz: %d", servo_freq, ret);
        return ret;
    }

    hexapod_info("PCA9685 initialized with %d Hz frequency", servo_freq);
    return 0;
}

/**
 * @brief Initialize IMU sensor
 *
 * Initializes the appropriate IMU sensor based on configuration.
 * Supports MPU6050, ADXL345, or auto-detection.
 *
 * @param dev Hexapod device structure
 * @return 0 on success, negative error code on failure
 */
static int hexapod_init_sensor(struct hexapod_device *dev)
{
    int ret = -EINVAL;

    if (!dev)
    {
        hexapod_err("NULL device pointer");
        return -EINVAL;
    }

    /* Skip if already initialized */
    if (dev->sensor_initialized)
    {
        hexapod_dbg("Sensor already initialized");
        return 0;
    }

    hexapod_dbg("Initializing sensor type %d", dev->sensor_type);

    switch (dev->sensor_type)
    {
    case HEXAPOD_SENSOR_MPU6050:
        hexapod_info("Initializing MPU6050 sensor");
        ret = mpu6050_init();
        if (ret == 0)
        {
            hexapod_info("MPU6050 sensor initialized successfully");
        }
        break;

    case HEXAPOD_SENSOR_ADXL345:
        hexapod_info("Initializing ADXL345 sensor");
        ret = adxl345_init(ADXL345_I2C_BUS);
        if (ret == 0)
        {
            hexapod_info("ADXL345 sensor initialized successfully");
        }
        break;

    case HEXAPOD_SENSOR_AUTO:
        hexapod_info("Auto-detecting IMU sensor");
        /* Try MPU6050 first */
        ret = mpu6050_init();
        if (ret == 0)
        {
            dev->sensor_type = HEXAPOD_SENSOR_MPU6050;
            hexapod_info("Auto-detected MPU6050 sensor");
        }
        else
        {
            /* Fall back to ADXL345 */
            ret = adxl345_init(ADXL345_I2C_BUS);
            if (ret == 0)
            {
                dev->sensor_type = HEXAPOD_SENSOR_ADXL345;
                hexapod_info("Auto-detected ADXL345 sensor");
            }
            else
            {
                hexapod_err("No IMU sensor detected during auto-detection");
                return -ENODEV;
            }
        }
        break;

    default:
        hexapod_err("Invalid sensor type: %d", dev->sensor_type);
        return -EINVAL;
    }

    if (ret < 0)
    {
        hexapod_err("Failed to initialize sensor: %d", ret);
        return ret;
    }

    dev->sensor_initialized = true;
    return 0;
}

/*============================================================================
 * Servo Control Functions
 *============================================================================*/

/**
 * @brief Set position for a single servo
 *
 * Controls a specific servo by converting angle to PWM and using PCA9685
 *
 * @param servo_num Servo number (0-17)
 * @param angle Target angle in degrees (-90 to +90)
 * @return 0 on success, negative error code on failure
 */
static int hexapod_set_servo_position(uint8_t servo_num, int16_t angle)
{
    uint16_t pwm_value;
    int ret;

    /* Validate servo number */
    if (servo_num >= HEXAPOD_TOTAL_SERVOS)
    {
        hexapod_err("Invalid servo number: %d (max: %d)", servo_num, HEXAPOD_TOTAL_SERVOS - 1);
        return -EINVAL;
    }

    /* Convert angle to PWM value */
    pwm_value = hexapod_angle_to_pwm(angle);

    /* Set servo position using PCA9685 driver */
    ret = pca9685_set_pwm(servo_num, 0, pwm_value);
    if (ret < 0)
    {
        hexapod_err("Failed to set servo %d to angle %d: %d", servo_num, angle, ret);
        return ret;
    }

    hexapod_dbg("Servo %d set to angle %d (PWM %d)", servo_num, angle, pwm_value);
    return 0;
}

/**
 * @brief Set position for all joints in a leg
 *
 * Controls all three servos in a specific leg, applying calibration offsets
 *
 * @param dev Hexapod device structure
 * @param cmd Leg command with target angles
 * @return 0 on success, negative error code on failure
 */
static int hexapod_set_leg_position(struct hexapod_device *dev,
                                    const struct hexapod_leg_cmd *cmd)
{
    uint8_t base_servo;
    int16_t hip_angle, knee_angle, ankle_angle;
    int ret;

    if (!dev || !cmd)
    {
        hexapod_err("NULL pointer in set_leg_position");
        return -EINVAL;
    }

    /* Validate leg number */
    if (!hexapod_is_valid_leg(cmd->leg_num))
    {
        hexapod_err("Invalid leg number: %d", cmd->leg_num);
        return -EINVAL;
    }

    /* Validate joint angles */
    if (!hexapod_validate_joint_angles(&cmd->joint))
    {
        return -EINVAL;
    }

    /* Calculate base servo number for this leg */
    base_servo = cmd->leg_num * HEXAPOD_SERVOS_PER_LEG;

    /* Apply calibration offsets */
    hip_angle = cmd->joint.hip + dev->calibration[cmd->leg_num].hip;
    knee_angle = cmd->joint.knee + dev->calibration[cmd->leg_num].knee;
    ankle_angle = cmd->joint.ankle + dev->calibration[cmd->leg_num].ankle;

    hexapod_dbg("Setting leg %d: hip=%d, knee=%d, ankle=%d (with calibration)",
                cmd->leg_num, hip_angle, knee_angle, ankle_angle);

    /* Set servo positions for each joint */
    ret = hexapod_set_servo_position(base_servo, hip_angle);
    if (ret < 0)
    {
        hexapod_err("Failed to set hip servo for leg %d", cmd->leg_num);
        return ret;
    }

    ret = hexapod_set_servo_position(base_servo + 1, knee_angle);
    if (ret < 0)
    {
        hexapod_err("Failed to set knee servo for leg %d", cmd->leg_num);
        return ret;
    }

    ret = hexapod_set_servo_position(base_servo + 2, ankle_angle);
    if (ret < 0)
    {
        hexapod_err("Failed to set ankle servo for leg %d", cmd->leg_num);
        return ret;
    }

    /* Store current position (without calibration for user reference) */
    dev->leg_positions[cmd->leg_num] = cmd->joint;

    hexapod_dbg("Leg %d positioned successfully", cmd->leg_num);
    return 0;
}

/**
 * @brief Center all legs to neutral position
 *
 * Sets all leg joints to 0 degrees (center position)
 *
 * @param dev Hexapod device structure
 * @return 0 on success, negative error code on failure
 */
static int hexapod_center_all_legs(struct hexapod_device *dev)
{
    struct hexapod_leg_cmd cmd;
    int ret;
    int i;

    if (!dev)
    {
        hexapod_err("NULL device pointer");
        return -EINVAL;
    }

    hexapod_info("Centering all legs");

    /* Set all legs to center position */
    cmd.joint.hip = 0;
    cmd.joint.knee = 0;
    cmd.joint.ankle = 0;

    for (i = 0; i < HEXAPOD_NUM_LEGS; i++)
    {
        cmd.leg_num = i;

        ret = hexapod_set_leg_position(dev, &cmd);
        if (ret < 0)
        {
            hexapod_err("Failed to center leg %d: %d", i, ret);
            return ret;
        }

        /* Small delay between leg movements for smoother operation */
        mdelay(50);
    }

    hexapod_info("All legs centered successfully");
    return 0;
}

/*============================================================================
 * IMU Sensor Functions
 *============================================================================*/

/**
 * @brief Read data from the active IMU sensor
 *
 * Reads accelerometer and gyroscope data from the configured sensor
 *
 * @param dev Hexapod device structure
 * @return 0 on success, negative error code on failure
 */
static int hexapod_read_imu_data(struct hexapod_device *dev)
{
    int ret;

    if (!dev)
    {
        hexapod_err("NULL device pointer");
        return -EINVAL;
    }

    /* Initialize sensor if not already done */
    if (!dev->sensor_initialized)
    {
        ret = hexapod_init_sensor(dev);
        if (ret < 0)
        {
            return ret;
        }
    }

    /* Read from appropriate sensor */
    switch (dev->sensor_type)
    {
    case HEXAPOD_SENSOR_MPU6050:
    {
        /* Read accelerometer and gyroscope data */
        struct mpu6050_imu_data mpu_data;

        /* Use existing MPU6050 API to read sensor data */
        ret = mpu6050_read_sensors(&mpu_data);

        /* Copy values to our imu_data structure */
        if (ret < 0)
        {
            hexapod_err("Failed to read from MPU6050 sensor (%d)", ret);
        }
        else
        {
            dev->imu_data.accel_x = mpu_data.accel_x;
            dev->imu_data.accel_y = mpu_data.accel_y;
            dev->imu_data.accel_z = mpu_data.accel_z;
            dev->imu_data.gyro_x = mpu_data.gyro_x;
            dev->imu_data.gyro_y = mpu_data.gyro_y;
            dev->imu_data.gyro_z = mpu_data.gyro_z;
        }
    }
    break;

    case HEXAPOD_SENSOR_ADXL345:
    {
        /* Create temporary variable of correct type for ADXL345 API */
        struct adxl345_accel_data accel_data;

        /* Read accelerometer data using proper API function */
        ret = adxl345_read_accel(&accel_data);

        /* Copy values to our imu_data structure */
        if (ret < 0)
        {
            hexapod_err("Failed to read from ADXL345 sensor (%d)", ret);
        }
        else
        {
            /* ADXL345 only provides accelerometer data */
            dev->imu_data.accel_x = accel_data.x;
            dev->imu_data.accel_y = accel_data.y;
            dev->imu_data.accel_z = accel_data.z;

            /* ADXL345 has no gyroscope, so set gyro values to 0 */
            dev->imu_data.gyro_x = 0;
            dev->imu_data.gyro_y = 0;
            dev->imu_data.gyro_z = 0;
        }
    }
    break;

    default:
        hexapod_err("Invalid sensor type: %d", dev->sensor_type);
        return -EINVAL;
    }

    if (ret < 0)
    {
        hexapod_err("Failed to read from IMU sensor (%d)", ret);
        return ret;
    }

    hexapod_dbg("IMU data: ax=%d, ay=%d, az=%d, gx=%d, gy=%d, gz=%d\n",
                dev->imu_data.accel_x, dev->imu_data.accel_y, dev->imu_data.accel_z,
                dev->imu_data.gyro_x, dev->imu_data.gyro_y, dev->imu_data.gyro_z);

    return 0;
}

/*============================================================================
 * Calibration Functions
 *============================================================================*/

/**
 * @brief Set calibration offsets for a specific leg
 *
 * Stores calibration values to compensate for mechanical variations
 *
 * @param dev Hexapod device structure
 * @param cal Calibration data
 * @return 0 on success, negative error code on failure
 */
static int hexapod_set_calibration(struct hexapod_device *dev,
                                   const struct hexapod_calibration *cal)
{
    if (!dev || !cal)
    {
        hexapod_err("NULL pointer in set_calibration");
        return -EINVAL;
    }

    /* Validate leg number */
    if (!hexapod_is_valid_leg(cal->leg_num))
    {
        hexapod_err("Invalid leg number for calibration: %d", cal->leg_num);
        return -EINVAL;
    }

    /* Validate calibration offsets (should be reasonable) */
    if (!hexapod_validate_joint_angles(&cal->offsets))
    {
        hexapod_err("Invalid calibration offsets for leg %d", cal->leg_num);
        return -EINVAL;
    }

    /* Store calibration values */
    dev->calibration[cal->leg_num] = cal->offsets;

    hexapod_info("Calibration set for leg %d: hip=%d, knee=%d, ankle=%d",
                 cal->leg_num, cal->offsets.hip, cal->offsets.knee, cal->offsets.ankle);

    return 0;
}

/*============================================================================
 * Character Device Operations
 *============================================================================*/

/**
 * @brief Device file open operation
 *
 * Called when a user-space process opens the hexapod device file
 *
 * @param inode Inode structure
 * @param filp File structure
 * @return 0 on success, negative error code on failure
 */
static int hexapod_open(struct inode *inode, struct file *filp)
{
    struct hexapod_device *dev;

    hexapod_dbg("Device opened");

    /* Get device structure from inode */
    dev = container_of(inode->i_cdev, struct hexapod_device, cdev);
    if (!dev)
    {
        hexapod_err("Failed to get device structure");
        return -ENODEV;
    }

    /* Store device pointer in file private data */
    filp->private_data = dev;

    return 0;
}

/**
 * @brief Device file release operation
 *
 * Called when a user-space process closes the hexapod device file
 *
 * @param inode Inode structure
 * @param filp File structure
 * @return 0 on success
 */
static int hexapod_release(struct inode *inode, struct file *filp)
{
    hexapod_dbg("Device closed");
    return 0;
}

/**
 * @brief Device file IOCTL operation
 *
 * Handles control commands from user-space applications
 *
 * @param filp File structure
 * @param cmd IOCTL command code
 * @param arg Command argument (user-space pointer)
 * @return 0 on success, negative error code on failure
 */
static long hexapod_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct hexapod_device *dev = filp->private_data;
    int ret = 0;

    if (!dev)
    {
        hexapod_err("Invalid device structure");
        return -ENODEV;
    }

    /* Verify IOCTL magic number */
    if (_IOC_TYPE(cmd) != HEXAPOD_IOC_MAGIC)
    {
        hexapod_err("Invalid IOCTL magic: 0x%x", _IOC_TYPE(cmd));
        return -ENOTTY;
    }

    /* Initialize PCA9685 if needed for servo commands */
    if ((cmd == HEXAPOD_IOCTL_SET_LEG || cmd == HEXAPOD_IOCTL_CENTER_ALL) &&
        !dev->pca9685_initialized)
    {
        ret = hexapod_init_pca9685();
        if (ret < 0)
        {
            hexapod_err("Failed to initialize PCA9685 for command");
            return ret;
        }
        dev->pca9685_initialized = true;
    }

    /* Lock device for exclusive access */
    if (mutex_lock_interruptible(&dev->device_lock))
    {
        return -ERESTARTSYS;
    }

    switch (cmd)
    {
    case HEXAPOD_IOCTL_SET_LEG:
    {
        struct hexapod_leg_cmd leg_cmd;

        hexapod_dbg("Processing SET_LEG command");

        /* Copy command from user space */
        if (copy_from_user(&leg_cmd, (void __user *)arg, sizeof(leg_cmd)))
        {
            hexapod_err("Failed to copy leg command from user space");
            ret = -EFAULT;
            goto unlock;
        }

        /* Execute leg positioning */
        ret = hexapod_set_leg_position(dev, &leg_cmd);
        if (ret == 0)
        {
            dev->cmd_count++;
        }
        else
        {
            dev->error_count++;
        }
    }
    break;

    case HEXAPOD_IOCTL_GET_IMU:
    {
        hexapod_dbg("Processing GET_IMU command");

        /* Read fresh IMU data */
        ret = hexapod_read_imu_data(dev);
        if (ret < 0)
        {
            hexapod_err("Failed to read IMU data");
            dev->error_count++;
            goto unlock;
        }

        /* Copy data to user space */
        if (copy_to_user((void __user *)arg, &dev->imu_data, sizeof(dev->imu_data)))
        {
            hexapod_err("Failed to copy IMU data to user space");
            ret = -EFAULT;
            dev->error_count++;
            goto unlock;
        }

        dev->cmd_count++;
    }
    break;

    case HEXAPOD_IOCTL_CALIBRATE:
    {
        struct hexapod_calibration cal;

        hexapod_dbg("Processing CALIBRATE command");

        /* Copy calibration from user space */
        if (copy_from_user(&cal, (void __user *)arg, sizeof(cal)))
        {
            hexapod_err("Failed to copy calibration from user space");
            ret = -EFAULT;
            goto unlock;
        }

        /* Apply calibration */
        ret = hexapod_set_calibration(dev, &cal);
        if (ret == 0)
        {
            dev->cmd_count++;
        }
        else
        {
            dev->error_count++;
        }
    }
    break;

    case HEXAPOD_IOCTL_CENTER_ALL:
    {
        hexapod_dbg("Processing CENTER_ALL command");

        /* Center all legs */
        ret = hexapod_center_all_legs(dev);
        if (ret == 0)
        {
            dev->cmd_count++;
        }
        else
        {
            dev->error_count++;
        }
    }
    break;

    case HEXAPOD_IOCTL_SET_SENSOR_TYPE:
    {
        int sensor_type;

        hexapod_dbg("Processing SET_SENSOR_TYPE command");

        /* Copy sensor type from user space */
        if (copy_from_user(&sensor_type, (void __user *)arg, sizeof(sensor_type)))
        {
            hexapod_err("Failed to copy sensor type from user space");
            ret = -EFAULT;
            goto unlock;
        }

        /* Validate sensor type */
        if (sensor_type < HEXAPOD_SENSOR_MPU6050 || sensor_type > HEXAPOD_SENSOR_AUTO)
        {
            hexapod_err("Invalid sensor type: %d", sensor_type);
            ret = -EINVAL;
            goto unlock;
        }

        /* Reset sensor initialization if type changed */
        if (dev->sensor_type != sensor_type)
        {
            dev->sensor_initialized = false;
            dev->sensor_type = sensor_type;
            hexapod_info("Sensor type changed to %d", sensor_type);
        }

        dev->cmd_count++;
    }
    break;

    case HEXAPOD_IOCTL_GET_SENSOR_TYPE:
    {
        hexapod_dbg("Processing GET_SENSOR_TYPE command");

        /* Copy sensor type to user space */
        if (copy_to_user((void __user *)arg, &dev->sensor_type, sizeof(dev->sensor_type)))
        {
            hexapod_err("Failed to copy sensor type to user space");
            ret = -EFAULT;
            goto unlock;
        }

        dev->cmd_count++;
    }
    break;

    default:
        hexapod_err("Unknown IOCTL command: 0x%x", cmd);
        ret = -ENOTTY;
        dev->error_count++;
        break;
    }

unlock:
    mutex_unlock(&dev->device_lock);
    return ret;
}

/**
 * @brief File operations structure
 *
 * Defines the operations supported by the hexapod character device
 */
static const struct file_operations hexapod_fops = {
    .owner = THIS_MODULE,
    .open = hexapod_open,
    .release = hexapod_release,
    .unlocked_ioctl = hexapod_ioctl,
    .llseek = no_llseek,
};

/*============================================================================
 * Module Initialization and Cleanup
 *============================================================================*/

/**
 * @brief Initialize hexapod device structure
 *
 * Allocates and initializes the main device structure
 *
 * @return Pointer to device structure, or NULL on failure
 */
static struct hexapod_device *hexapod_init_device(void)
{
    struct hexapod_device *dev;
    int i;

    /* Allocate device structure */
    dev = kzalloc(sizeof(struct hexapod_device), GFP_KERNEL);
    if (!dev)
    {
        hexapod_err("Failed to allocate device structure");
        return NULL;
    }

    /* Initialize mutex */
    mutex_init(&dev->device_lock);

    /* Initialize sensor type from module parameter */
    dev->sensor_type = default_sensor;

    /* Initialize all leg positions and calibrations to zero */
    for (i = 0; i < HEXAPOD_NUM_LEGS; i++)
    {
        dev->leg_positions[i].hip = 0;
        dev->leg_positions[i].knee = 0;
        dev->leg_positions[i].ankle = 0;

        dev->calibration[i].hip = 0;
        dev->calibration[i].knee = 0;
        dev->calibration[i].ankle = 0;
    }

    /* Initialize IMU data to zero */
    memset(&dev->imu_data, 0, sizeof(dev->imu_data));

    /* Initialize statistics */
    dev->cmd_count = 0;
    dev->error_count = 0;
    dev->last_update = ktime_get();

    hexapod_dbg("Device structure initialized");
    return dev;
}

/**
 * @brief Module initialization function
 *
 * Called when the module is loaded into the kernel
 *
 * @return 0 on success, negative error code on failure
 */
static int __init hexapod_init(void)
{
    int ret;

    hexapod_info("Initializing integrated hexapod driver v2.0");

    /* Initialize device structure */
    hexapod_dev = hexapod_init_device();
    if (!hexapod_dev)
    {
        ret = -ENOMEM;
        goto fail_device;
    }

    /* Allocate character device region */
    ret = alloc_chrdev_region(&hexapod_major, 0, 1, DEVICE_NAME);
    if (ret < 0)
    {
        hexapod_err("Failed to allocate character device region: %d", ret);
        goto fail_chrdev;
    }

    hexapod_dev->devno = hexapod_major;

    /* Create device class */
    hexapod_dev->class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(hexapod_dev->class))
    {
        ret = PTR_ERR(hexapod_dev->class);
        hexapod_err("Failed to create device class: %d", ret);
        goto fail_class;
    }

    /* Create device file */
    hexapod_dev->device = device_create(hexapod_dev->class, NULL, hexapod_major,
                                        NULL, DEVICE_NAME);
    if (IS_ERR(hexapod_dev->device))
    {
        ret = PTR_ERR(hexapod_dev->device);
        hexapod_err("Failed to create device: %d", ret);
        goto fail_device_create;
    }

    /* Initialize character device */
    cdev_init(&hexapod_dev->cdev, &hexapod_fops);
    hexapod_dev->cdev.owner = THIS_MODULE;

    /* Add character device to system */
    ret = cdev_add(&hexapod_dev->cdev, hexapod_major, 1);
    if (ret < 0)
    {
        hexapod_err("Failed to add character device: %d", ret);
        goto fail_cdev;
    }

    /* Optional: Try to initialize hardware early */
    ret = hexapod_init_pca9685();
    if (ret == 0)
    {
        hexapod_dev->pca9685_initialized = true;
        hexapod_info("PCA9685 pre-initialized successfully");

        /* Center all legs for safety */
        ret = hexapod_center_all_legs(hexapod_dev);
        if (ret < 0)
        {
            hexapod_warn("Failed to center legs during initialization: %d", ret);
        }
    }
    else
    {
        hexapod_warn("PCA9685 pre-initialization failed: %d (will retry on first use)", ret);
    }

    /* Try to initialize sensor */
    ret = hexapod_init_sensor(hexapod_dev);
    if (ret < 0)
    {
        hexapod_warn("Sensor initialization failed: %d (will retry on first use)", ret);
    }

    hexapod_info("Hexapod driver initialized successfully");
    hexapod_info("Device: /dev/%s, Major: %d", DEVICE_NAME, MAJOR(hexapod_major));
    hexapod_info("Debug mode: %s", debug ? "enabled" : "disabled");
    hexapod_info("Default sensor: %d, Servo frequency: %d Hz", default_sensor, servo_freq);

    return 0;

    /* Error handling - cleanup in reverse order */
fail_cdev:
    device_destroy(hexapod_dev->class, hexapod_major);
fail_device_create:
    class_destroy(hexapod_dev->class);
fail_class:
    unregister_chrdev_region(hexapod_major, 1);
fail_chrdev:
    mutex_destroy(&hexapod_dev->device_lock);
    kfree(hexapod_dev);
    hexapod_dev = NULL;
fail_device:
    return ret;
}

/**
 * @brief Module cleanup function
 *
 * Called when the module is unloaded from the kernel
 */
static void __exit hexapod_exit(void)
{
    hexapod_info("Cleaning up hexapod driver");

    if (!hexapod_dev)
    {
        hexapod_warn("Device structure is NULL during cleanup");
        return;
    }

    /* Center legs for safety before shutdown */
    if (hexapod_dev->pca9685_initialized)
    {
        int ret = hexapod_center_all_legs(hexapod_dev);
        if (ret < 0)
        {
            hexapod_warn("Failed to center legs during shutdown: %d", ret);
        }
        else
        {
            /* Allow time for servos to reach center position */
            mdelay(500);
        }
    }

    /* Print final statistics */
    hexapod_info("Final statistics: %lu commands processed, %lu errors encountered",
                 hexapod_dev->cmd_count, hexapod_dev->error_count);

    /* Remove character device */
    cdev_del(&hexapod_dev->cdev);

    /* Remove device file and class */
    device_destroy(hexapod_dev->class, hexapod_major);
    class_destroy(hexapod_dev->class);

    /* Unregister character device region */
    unregister_chrdev_region(hexapod_major, 1);

    /* Destroy mutex and free device structure */
    mutex_destroy(&hexapod_dev->device_lock);
    kfree(hexapod_dev);
    hexapod_dev = NULL;

    hexapod_info("Hexapod driver cleanup completed");
}

/* Register module init and exit functions */
module_init(hexapod_init);
module_exit(hexapod_exit);

/*============================================================================
 * Module Information
 *============================================================================*/

/**
 * @brief Module alias for automatic loading
 */
MODULE_ALIAS("hexapod-robot-driver");

/**
 * @brief Supported device table (if using device tree)
 */
#ifdef CONFIG_OF
static const struct of_device_id hexapod_of_match[] = {
    {.compatible = "hexapod,robot-v2"},
    {},
};
MODULE_DEVICE_TABLE(of, hexapod_of_match);
#endif
