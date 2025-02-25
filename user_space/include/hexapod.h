#ifndef _HEXAPOD_USER_H_
#define _HEXAPOD_USER_H_

#include <stdint.h>
#include "protocol.h"

/* Hexapod state structure */
typedef struct
{
    int initialized;
    leg_position_t legs[NUM_LEGS];
    imu_data_t imu_data;
} hexapod_state_t;

/* Function declarations */
int hexapod_init(void);
void hexapod_cleanup(void);
int hexapod_set_leg_position(uint8_t leg_num, const leg_position_t *position);
int hexapod_get_leg_position(uint8_t leg_num, leg_position_t *position);
int hexapod_get_imu_data(imu_data_t *data);
int hexapod_center_all_legs(void);

#endif /* _HEXAPOD_USER_H_ */