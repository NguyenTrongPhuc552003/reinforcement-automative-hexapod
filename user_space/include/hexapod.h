#ifndef _HEXAPOD_USER_H_
#define _HEXAPOD_USER_H_

#include <stdint.h>
#include "protocol.h"

/* State structure */
typedef struct {
    int initialized;
    leg_command legs[NUM_LEGS];
    imu_data imu_data;
} hexapod_state_t;

/* Core functions */
int hexapod_init(void);
void hexapod_cleanup(void);

/* Leg control functions */
int hexapod_set_leg_position(uint8_t leg_num, const leg_command *position);
int hexapod_get_leg_position(uint8_t leg_num, leg_command *position);
int hexapod_center_all_legs(void);

/* Sensor functions */
int hexapod_get_imu_data(imu_data *data);

#endif /* _HEXAPOD_USER_H_ */