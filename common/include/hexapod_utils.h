#ifndef _HEXAPOD_UTILS_H_
#define _HEXAPOD_UTILS_H_

#include "protocol.h"

#ifdef __KERNEL__
#include <linux/fs.h>
#else
#include <sys/types.h>
struct inode;
struct file;
#endif

/* PWM definitions */
#define PWM_CENTER_US 1500
#define SERVO_CENTER_PWM PWM_CENTER_US

/* Conversion utilities */
double cents_to_degrees(int32_t cents);
int32_t degrees_to_cents(double degrees);
uint16_t angle_to_pwm_us(double angle);
double imu_accel_to_g(int16_t raw);
double imu_gyro_to_dps(int16_t raw);
double imu_temp_to_celsius(int16_t raw);

/* Device operations */
#ifndef __KERNEL__
int hexapod_open(struct inode *inode, struct file *file);
int hexapod_release(struct inode *inode, struct file *file);
int hexapod_set_leg(int fd, uint8_t leg, double hip, double knee, double ankle);
int hexapod_get_imu(int fd, struct imu_data *data);
#endif

#endif /* _HEXAPOD_UTILS_H_ */
