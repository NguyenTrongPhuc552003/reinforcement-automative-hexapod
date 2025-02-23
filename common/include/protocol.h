#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#include <stdint.h>
#endif

/* Device configuration */
#define DEVICE_PATH "/dev/hexapod"

/* Hardware limits */
#define SERVO_MIN_ANGLE -90
#define SERVO_MAX_ANGLE 90
#define SERVO_CENTER_ANGLE 0

/* PWM configuration */
#define PWM_FREQUENCY 50
#define PWM_MIN_US 500     // 0.5ms
#define PWM_MAX_US 2500    // 2.5ms
#define PWM_CENTER_US 1500 // 1.5ms
#define SERVO_CENTER_PWM PWM_CENTER_US

/* IOCTL commands */
#define HEXAPOD_IOC_MAGIC 'H'
#define SET_LEG_POSITION _IOW(HEXAPOD_IOC_MAGIC, 1, struct leg_command)
#define GET_IMU_DATA _IOR(HEXAPOD_IOC_MAGIC, 2, struct imu_data)

/* Hardware configuration */
#define NUM_LEGS 6
#define SERVOS_PER_LEG 3
#define TOTAL_SERVOS (NUM_LEGS * SERVOS_PER_LEG)

/* Common structures */
struct leg_command
{
    uint8_t leg_num;
    int32_t hip_angle;   // centidegrees
    int32_t knee_angle;  // centidegrees
    int32_t ankle_angle; // centidegrees
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

#endif /* _PROTOCOL_H_ */
