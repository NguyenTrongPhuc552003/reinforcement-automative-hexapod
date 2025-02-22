#ifndef _GAIT_H_
#define _GAIT_H_

#include <stdint.h>
#include "hexapod.h"
#include "kinematics.h"

/* Gait Types */
typedef enum {
    GAIT_TRIPOD,
    GAIT_WAVE,
    GAIT_RIPPLE
} gait_type_t;

/* Gait Parameters */
typedef struct {
    gait_type_t type;
    double step_height;
    double step_length;
    double cycle_time;
    double duty_factor;
} gait_params_t;

/* Gait Functions */
int gait_init(const gait_params_t *params);
int gait_update(double time, hexapod_state_t *state);
void gait_cleanup(void);

/* Gait Steps */
int tripod_gait_step(int step, const gait_params_t *params);
int wave_gait_step(int step, const gait_params_t *params);
int ripple_gait_step(int step, const gait_params_t *params);

/* Trajectory Generation */
int generate_trajectory(const point3d_t *start, const point3d_t *end, 
                      double duration, uint32_t num_points,
                      point3d_t *trajectory);

#endif /* _GAIT_H_ */
