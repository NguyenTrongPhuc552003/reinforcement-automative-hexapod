#ifndef HEXAPOD_H
#define HEXAPOD_H

#include <stdint.h>
#include <linux/ioctl.h>

#define DEVICE_FILE "/dev/hexapod"
#define HEXAPOD_MAGIC 'H'

#define NUM_LEGS 6
#define JOINTS_PER_LEG 3

// Kernel driver structs for IOCTL
struct leg_position {
    uint8_t leg_num;
    int hip_angle;
    int knee_angle;
    int ankle_angle;
};

struct imu_data {
    double roll;
    double pitch;
    double yaw;
    double accel_x;
    double accel_y;
    double accel_z;
};

// IOCTL commands
#define SET_LEG_POSITION _IOW(HEXAPOD_MAGIC, 1, struct leg_position)
#define GET_IMU_DATA     _IOR(HEXAPOD_MAGIC, 2, struct imu_data)

// Leg position structure matching kernel driver
typedef struct {
    double hip;
    double knee;
    double ankle;
} leg_position_t;

// IMU data structure matching kernel driver
typedef struct {
    double roll;
    double pitch;
    double yaw;
    double accel_x;
    double accel_y;
    double accel_z;
} imu_data_t;

// Hexapod state
typedef struct {
    int initialized;
    leg_position_t legs[NUM_LEGS];
    imu_data_t imu_data;
} hexapod_state_t;

// Function declarations
int hexapod_init(void);
void hexapod_cleanup(void);
int hexapod_set_leg_position(uint8_t leg_num, const leg_position_t *position);
int hexapod_get_leg_position(uint8_t leg_num, leg_position_t *position);
int hexapod_get_imu_data(imu_data_t *data);
int hexapod_center_all_legs(void);

#endif /* HEXAPOD_H */