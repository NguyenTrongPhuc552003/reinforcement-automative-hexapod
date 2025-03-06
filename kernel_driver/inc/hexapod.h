#ifndef _HEXAPOD_H_
#define _HEXAPOD_H_

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>

/* Hardware Configuration */
#define HEXAPOD_I2C_BUS 3
#define MPU6050_I2C_ADDR 0x68
#define PCA9685_I2C_ADDR 0x40

/* Robot dimensions (mm) */
#define COXA_LENGTH 30
#define FEMUR_LENGTH 85
#define TIBIA_LENGTH 130

/* Leg/Servo Configuration */
#define NUM_LEGS 6
#define NUM_JOINTS_PER_LEG 3
#define TOTAL_SERVOS (NUM_LEGS * NUM_JOINTS_PER_LEG)

/* Servo angle limits (degrees) */
#define MIN_ANGLE -90
#define MAX_ANGLE 90

/* IOCTL commands */
#define HEXAPOD_IOC_MAGIC 'H'
#define HEXAPOD_IOCTL_SET_LEG _IOW(HEXAPOD_IOC_MAGIC, 1, struct hexapod_leg_cmd)
#define HEXAPOD_IOCTL_GET_IMU _IOR(HEXAPOD_IOC_MAGIC, 2, struct hexapod_imu_data)
#define HEXAPOD_IOCTL_CALIBRATE _IOW(HEXAPOD_IOC_MAGIC, 3, struct hexapod_calibration)
#define HEXAPOD_IOCTL_CENTER_ALL _IO(HEXAPOD_IOC_MAGIC, 4)
#define HEXAPOD_IOCTL_MAX 4

/* Data structures */
struct hexapod_leg_position
{
    s16 hip;
    s16 knee;
    s16 ankle;
};

struct hexapod_leg_cmd
{
    u8 leg_num;
    struct hexapod_leg_position position;
};

struct hexapod_imu_data
{
    s16 accel_x;
    s16 accel_y;
    s16 accel_z;
    s16 gyro_x;
    s16 gyro_y;
    s16 gyro_z;
};

struct hexapod_calibration
{
    u8 leg_num;
    s16 hip_offset;
    s16 knee_offset;
    s16 ankle_offset;
};

/* Main driver data structure */
struct hexapod_data
{
    struct i2c_client *mpu6050;
    struct mutex lock;
    struct hexapod_leg_position positions[NUM_LEGS];
    struct hexapod_calibration calibration[NUM_LEGS];
    bool initialized;
};

/* Function declarations */
int hexapod_set_leg_position(struct hexapod_data *dev, u8 leg, struct hexapod_leg_position *pos);
int hexapod_get_imu_data(struct hexapod_data *dev, struct hexapod_imu_data *data);
int hexapod_set_calibration(struct hexapod_data *dev, u8 leg, struct hexapod_calibration *cal);
int hexapod_center_all(struct hexapod_data *dev);

/* External global device data */
extern struct hexapod_data hexapod_dev;

#endif /* _HEXAPOD_H_ */
