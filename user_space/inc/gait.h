#ifndef USER_GAIT_H
#define USER_GAIT_H

#include "hexapod.h"
#include "kinematics.h"

/* Gait types */
typedef enum
{
    GAIT_TRIPOD = 0, // Tripod gait (alternates 3 legs at a time)
    GAIT_WAVE,       // Wave gait (one leg at a time)
    GAIT_RIPPLE      // Ripple gait (overlapping movement)
} gait_type_t;

/* Gait parameters */
typedef struct
{
    gait_type_t type;   // Gait pattern type
    double step_height; // Height of leg lift (mm)
    double step_length; // Length of step (mm)
    double cycle_time;  // Time for complete gait cycle (seconds)
    double duty_factor; // Portion of cycle in support phase (0.0-1.0)
    double direction;   // Direction of movement (degrees, 0=forward)
    double speed;       // Speed factor (0.0-1.0)
} gait_params_t;

/* Gait control functions */
int gait_init(const gait_params_t *params);
int gait_update(double time, hexapod_state_t *state);
void gait_cleanup(void);

/* Specific gait pattern implementations */
int tripod_gait_step(int step, const gait_params_t *params);
int wave_gait_step(int step, const gait_params_t *params);
int ripple_gait_step(int step, const gait_params_t *params);

/* Gait transitions */
void gait_transition_to_wave(const gait_params_t *params);

/* Trajectory generation */
int generate_trajectory(const point3d_t *start, const point3d_t *end,
                        double duration, uint32_t num_points,
                        point3d_t *trajectory);

#endif /* USER_GAIT_H */
