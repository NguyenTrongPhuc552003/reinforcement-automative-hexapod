#ifndef USER_HEXAPOD_H
#define USER_HEXAPOD_H

#include <stdint.h>
#include "protocol.h"

/* Typedefs to match the common protocol types */
typedef struct leg_position leg_position_t;
typedef struct leg_command leg_command_t;
typedef struct imu_data imu_data_t;

/* Hexapod state for gait control */
typedef struct
{
    leg_position_t legs[NUM_LEGS];
    imu_data_t imu;
    double heading;
    double velocity;
} hexapod_state_t;

/* Initialization and cleanup */
int hexapod_init(void);
void hexapod_cleanup(void);

/* Leg control */
int hexapod_set_leg_position(uint8_t leg_num, const leg_position_t *position);
int hexapod_get_leg_position(uint8_t leg_num, leg_position_t *position);
int hexapod_center_all_legs(void);

/* IMU functions */
int hexapod_get_imu_data(imu_data_t *data);

/* Timing functions */
double hexapod_get_time(void);

/* Utility functions */
void hexapod_sleep(double seconds);

#endif /* USER_HEXAPOD_H */