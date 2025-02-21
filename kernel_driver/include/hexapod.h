#ifndef _HEXAPOD_H_
#define _HEXAPOD_H_

#include <linux/types.h>
#include <linux/ioctl.h>

/* Device Info */
#define DEVICE_NAME "hexapod"
#define CLASS_NAME  "hexapod"

/* Leg Configuration */
#define NUM_LEGS            6
#define JOINTS_PER_LEG      3
#define TOTAL_SERVOS        (NUM_LEGS * JOINTS_PER_LEG)

/* IOCTL Commands */
#define HEXAPOD_MAGIC 'H'
#define SET_LEG_POSITION _IOW(HEXAPOD_MAGIC, 1, struct leg_position)
#define GET_IMU_DATA     _IOR(HEXAPOD_MAGIC, 2, struct imu_data)

/* IOCTL Structures */
struct leg_position {
    u8 leg_num;
    int hip_angle;
    int knee_angle;
    int ankle_angle;
};

struct imu_data {
    s16 accel_x;
    s16 accel_y;
    s16 accel_z;
    s16 gyro_x;
    s16 gyro_y;
    s16 gyro_z;
};

#endif /* _HEXAPOD_H_ */
