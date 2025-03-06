#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "hexapod.h"
#include "gait.h"

/* Controller states */
typedef enum
{
    CTRL_STATE_IDLE,
    CTRL_STATE_WALKING,
    CTRL_STATE_ROTATING,
    CTRL_STATE_TILTING
} controller_state_t;

/* Movement parameters */
typedef struct
{
    double speed;     /* 0.0 to 1.0 */
    double direction; /* Degrees, 0 = forward */
    double height;    /* Body height adjustment in mm */
    double tilt_x;    /* Forward/backward tilt in degrees */
    double tilt_y;    /* Left/right tilt in degrees */
    gait_type_t gait; /* Current gait pattern */
    controller_state_t state;
} controller_params_t;

/* Controller functions */
int controller_init(hexapod_t *hex);
void controller_cleanup(void);
int controller_process_key(char key);
int controller_update(void);

#endif /* CONTROLLER_H */
