#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include "gait.h"
#include "hexapod.h"
#include "kinematics.h"

/* Default leg positions (standing stance) */
static const point3d_t default_positions[NUM_LEGS] = {
    {100.0, 100.0, -100.0},  /* Front-right */
    {0.0, 120.0, -100.0},    /* Middle-right */
    {-100.0, 100.0, -100.0}, /* Back-right */
    {100.0, -100.0, -100.0}, /* Front-left */
    {0.0, -120.0, -100.0},   /* Middle-left */
    {-100.0, -100.0, -100.0} /* Back-left */
};

/* Gait controller state */
static struct
{
    hexapod_t *hexapod;
    gait_params_t params;
    double cycle_time;
    double last_time;
    int initialized;

    /* Leg state tracking */
    double phase_offset[NUM_LEGS];
    int leg_state[NUM_LEGS]; /* 0 = stance, 1 = swing */
} gait_state;

/* Initialize gait controller */
int gait_init(hexapod_t *hexapod, gait_params_t *params)
{
    if (!hexapod || !params)
        return -EINVAL;

    /* Store parameters */
    memset(&gait_state, 0, sizeof(gait_state));
    gait_state.hexapod = hexapod;
    gait_state.cycle_time = params->cycle_time;
    memcpy(&gait_state.params, params, sizeof(gait_params_t));

    /* Configure phase offsets based on gait type */
    switch (params->type)
    {
    case GAIT_TRIPOD:
        /* Legs 0, 2, 4 move together, and 1, 3, 5 move together */
        gait_state.phase_offset[0] = 0.0;
        gait_state.phase_offset[1] = 0.5;
        gait_state.phase_offset[2] = 0.0;
        gait_state.phase_offset[3] = 0.5;
        gait_state.phase_offset[4] = 0.0;
        gait_state.phase_offset[5] = 0.5;
        break;

    case GAIT_WAVE:
        /* Each leg is 1/6 cycle out of phase with the next */
        gait_state.phase_offset[0] = 0.0; /* Front right */
        gait_state.phase_offset[2] = 0.2; /* Middle right */
        gait_state.phase_offset[4] = 0.4; /* Back right */
        gait_state.phase_offset[1] = 0.5; /* Front left */
        gait_state.phase_offset[3] = 0.7; /* Middle left */
        gait_state.phase_offset[5] = 0.9; /* Back left */
        params->duty_factor = 0.85;       /* Increase stance phase */
        params->step_height = 40.0;       /* Higher steps for better clearance */
        break;

    case GAIT_RIPPLE:
        /* Right legs: 0, 2, 4; Left legs: 1, 3, 5 */
        gait_state.phase_offset[0] = 0.0;
        gait_state.phase_offset[1] = 0.5;
        gait_state.phase_offset[2] = 0.33;
        gait_state.phase_offset[3] = 0.83;
        gait_state.phase_offset[4] = 0.67;
        gait_state.phase_offset[5] = 0.17;
        break;

    default:
        return -EINVAL;
    }

    /* Set initial state */
    gait_state.initialized = 1;
    gait_state.last_time = 0.0;

    return 0;
}

/* Update gait for all legs */
int gait_update(hexapod_t *hexapod, double time, double direction, double speed)
{
    double phase, leg_phase;
    point3d_t foot_pos;
    leg_position_t angles;
    int result, i;

    if (!hexapod || !gait_state.initialized)
        return -EINVAL;

    /* Calculate global phase (0.0 - 1.0) */
    phase = fmod(time / gait_state.cycle_time, 1.0);

    /* Store current time for next update */
    gait_state.last_time = time;

    /* Update each leg */
    for (i = 0; i < NUM_LEGS; i++)
    {
        /* Calculate leg phase (with offset) */
        leg_phase = fmod(phase + gait_state.phase_offset[i], 1.0);

        /* Get default position for this leg */
        memcpy(&foot_pos, &default_positions[i], sizeof(point3d_t));

        /* Apply stride pattern based on leg phase */
        compute_leg_trajectory(&foot_pos, leg_phase, direction, speed, &gait_state.params);

        /* Convert position to joint angles */
        result = inverse_kinematics(&foot_pos, &angles);
        if (result != 0)
        {
            fprintf(stderr, "Inverse kinematics failed for leg %d\n", i);
            continue;
        }

        /* Send command to hexapod */
        result = hexapod_set_leg_position(hexapod, i, &angles);
        if (result != 0)
        {
            fprintf(stderr, "Failed to set position for leg %d: %s\n",
                    i, hexapod_error_string(result));
            return result;
        }
    }

    return 0;
}

/* Compute leg trajectory for a given phase */
void compute_leg_trajectory(point3d_t *position, double phase,
                            double direction, double speed,
                            const gait_params_t *params)
{
    double stride_x, stride_y;
    double angle_rad = direction * M_PI / 180.0;
    double stance_phase, swing_phase;
    double stride_length = params->step_length * speed;

    /* Calculate stride vector components */
    stride_x = stride_length * cos(angle_rad);
    stride_y = stride_length * sin(angle_rad);

    /* Determine if stance or swing phase */
    if (phase < params->duty_factor)
    {
        /* Stance phase - foot is on ground, moving backward */
        stance_phase = phase / params->duty_factor;
        position->x += stride_x * (0.5 - stance_phase);
        position->y += stride_y * (0.5 - stance_phase);
        /* Height is constant during stance phase */
    }
    else
    {
        /* Swing phase - foot is in air, moving forward */
        swing_phase = (phase - params->duty_factor) / (1.0 - params->duty_factor);
        position->x += stride_x * (swing_phase - 0.5);
        position->y += stride_y * (swing_phase - 0.5);

        /* Add vertical component - parabolic trajectory */
        position->z += params->step_height *
                       (1.0 - 4.0 * pow(swing_phase - 0.5, 2.0));
    }
}

/* Stop gait controller */
void gait_cleanup(void)
{
    gait_state.initialized = 0;
}

/* Center all legs in default position */
int gait_center(hexapod_t *hexapod)
{
    leg_position_t angles;
    point3d_t pos;
    int ret, i;

    if (!hexapod)
        return -EINVAL;

    for (i = 0; i < NUM_LEGS; i++)
    {
        /* Get default position */
        memcpy(&pos, &default_positions[i], sizeof(point3d_t));

        /* Convert to joint angles */
        ret = inverse_kinematics(&pos, &angles);
        if (ret != 0)
        {
            fprintf(stderr, "Inverse kinematics failed for leg %d\n", i);
            continue;
        }

        /* Send command */
        ret = hexapod_set_leg_position(hexapod, i, &angles);
        if (ret != 0)
        {
            fprintf(stderr, "Failed to center leg %d: %s\n",
                    i, hexapod_error_string(ret));
            return ret;
        }

        /* Small delay between legs */
        usleep(50000);
    }

    return 0;
}
