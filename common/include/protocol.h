#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#include <stdint.h>
#endif

/* PWM Configuration */
#define PWM_FREQUENCY 50
#define PWM_MIN_US 500     // 0.5ms
#define PWM_MAX_US 2500    // 2.5ms
#define PWM_CENTER_US 1500 // 1.5ms

/* Device and IOCTL definitions */
#define DEVICE_PATH "/dev/hexapod"
#define HEXAPOD_IOC_MAGIC 'H'

/* Structure type definitions */
typedef struct leg_position leg_position_t;
typedef struct imu_data imu_data_t;

/* Common structures for IOCTL */
struct leg_position
{
    double hip;
    double knee;
    double ankle;
} __attribute__((packed));

struct leg_command
{
    uint8_t leg_num;
    struct leg_position position;
} __attribute__((packed));

struct imu_data
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} __attribute__((packed));

/* IOCTL commands */
#define SET_LEG_POSITION _IOW(HEXAPOD_IOC_MAGIC, 1, struct leg_command)
#define GET_IMU_DATA _IOR(HEXAPOD_IOC_MAGIC, 2, struct imu_data)

/* Hardware configuration */
#define NUM_LEGS 6
#define JOINTS_PER_LEG 3
#define TOTAL_SERVOS (NUM_LEGS * JOINTS_PER_LEG)

#endif /* _PROTOCOL_H_ */
