#ifndef GAIT_H
#define GAIT_H

#include "hexapod.h"
#include "kinematics.h"

/* Gait types */
typedef enum gait_type
{
    GAIT_TRIPOD, /* Alternating tripod (3 legs at a time) */
    GAIT_WAVE,   /* Wave gait (one leg at a time) */
    GAIT_RIPPLE  /* Ripple gait (overlapping movement) */
} gait_type_t;

/* Gait parameters */
typedef struct gait_params
{
    gait_type_t type;   /* Type of gait pattern */
    double step_height; /* Maximum step height (mm) */
    double step_length; /* Maximum stride length (mm) */
    double cycle_time;  /* Time for complete gait cycle (seconds) */
    double duty_factor; /* Portion of cycle in stance phase (0.0-1.0) */
} gait_params_t;

/* Gait control functions */
int gait_init(hexapod_t *hexapod, gait_params_t *params);
int gait_update(hexapod_t *hexapod, double time, double direction, double speed);
void gait_cleanup(void);
int gait_center(hexapod_t *hexapod);

/* Utility functions */
void compute_leg_trajectory(point3d_t *position, double phase,
                            double direction, double speed,
                            const gait_params_t *params);

#endif /* GAIT_H */
