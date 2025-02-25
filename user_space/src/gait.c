#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "gait.h"

/* Gait state */
static struct
{
    gait_params_t params;
    double phase_offset[NUM_LEGS];
    bool initialized;
} gait_state = {0};

/* Tripod gait phase offsets */
static const double tripod_offsets[NUM_LEGS] = {
    0.0, /* Leg 0: Right Front */
    0.5, /* Leg 1: Right Middle */
    0.0, /* Leg 2: Right Back */
    0.5, /* Leg 3: Left Front */
    0.0, /* Leg 4: Left Middle */
    0.5  /* Leg 5: Left Back */
};

/* Wave gait phase offsets */
static const double wave_offsets[NUM_LEGS] = {
    0.0,   /* Leg 0 */
    0.167, /* Leg 1 */
    0.333, /* Leg 2 */
    0.5,   /* Leg 3 */
    0.667, /* Leg 4 */
    0.833  /* Leg 5 */
};

/* Ripple gait phase offsets */
static const double ripple_offsets[NUM_LEGS] = {
    0.0,  /* Leg 0 */
    0.5,  /* Leg 1 */
    0.25, /* Leg 2 */
    0.75, /* Leg 3 */
    0.5,  /* Leg 4 */
    0.0   /* Leg 5 */
};

/* Initialize gait controller */
int gait_init(const gait_params_t *params)
{
    if (!params)
    {
        return -1;
    }

    /* Store parameters */
    memcpy(&gait_state.params, params, sizeof(gait_params_t));

    /* Set phase offsets based on gait type */
    switch (params->type)
    {
    case GAIT_TRIPOD:
        memcpy(gait_state.phase_offset, tripod_offsets, sizeof(tripod_offsets));
        break;
    case GAIT_WAVE:
        memcpy(gait_state.phase_offset, wave_offsets, sizeof(wave_offsets));
        break;
    case GAIT_RIPPLE:
        memcpy(gait_state.phase_offset, ripple_offsets, sizeof(ripple_offsets));
        break;
    default:
        return -1;
    }

    gait_state.initialized = true;
    return 0;
}

/* Calculate leg position for given phase */
static void calculate_leg_position(double phase, point3d_t *pos)
{
    double x, y, z;

    /* Normalize phase to [0, 1] */
    phase = fmod(phase, 1.0);
    if (phase < 0)
        phase += 1.0;

    /* Support phase */
    if (phase >= gait_state.params.duty_factor)
    {
        /* Swing phase - generate trajectory */
        double swing_phase = (phase - gait_state.params.duty_factor) /
                             (1.0 - gait_state.params.duty_factor);

        /* Parabolic trajectory for swing */
        x = gait_state.params.step_length * (swing_phase - 0.5);
        z = gait_state.params.step_height * (1.0 - 4.0 * pow(swing_phase - 0.5, 2));
    }
    else
    {
        /* Support phase - linear motion */
        double support_phase = phase / gait_state.params.duty_factor;
        x = gait_state.params.step_length * (support_phase - 0.5);
        z = 0;
    }

    y = 0; /* Lateral position is constant */

    pos->x = x;
    pos->y = y;
    pos->z = z;
}

/* Update gait for all legs */
int gait_update(double time, hexapod_state_t *state)
{
    if (!gait_state.initialized || !state)
    {
        return -1;
    }

    /* Calculate phase for each leg */
    double base_phase = fmod(time / gait_state.params.cycle_time, 1.0);

    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        /* Calculate leg phase */
        double phase = base_phase + gait_state.phase_offset[leg];

        /* Get trajectory point */
        point3d_t target_pos;
        calculate_leg_position(phase, &target_pos);

        /* Convert to joint angles */
        leg_position_t angles = {0}; // Initialize to zero
        if (inverse_kinematics(&target_pos, &angles) < 0)
        {
            fprintf(stderr, "Inverse kinematics failed for leg %d\n", leg);
            continue;
        }

        /* Update leg position */
        if (hexapod_set_leg_position(leg, &angles) < 0)
        {
            fprintf(stderr, "Failed to set position for leg %d\n", leg);
            return -1;
        }
    }

    return 0;
}

/* Generate trajectory between two points */
int generate_trajectory(const point3d_t *start, const point3d_t *end,
                        double duration, uint32_t num_points,
                        point3d_t *trajectory)
{
    if (!start || !end || !trajectory || num_points < 2 || duration <= 0)
    {
        return -1;
    }

    for (uint32_t i = 0; i < num_points; i++)
    {
        double t = (double)i / (num_points - 1);

        /* Linear interpolation */
        trajectory[i].x = start->x + t * (end->x - start->x);
        trajectory[i].y = start->y + t * (end->y - start->y);
        trajectory[i].z = start->z + t * (end->z - start->z);
    }

    return 0;
}

/* Cleanup gait controller */
void gait_cleanup(void)
{
    gait_state.initialized = false;
}

/* Gait Step Functions */
int tripod_gait_step(int step, const gait_params_t *params)
{
    if (!params)
    {
        return -1;
    }

    // Legs 0,2,4 move together, then 1,3,5
    bool is_first_group = (step % 2 == 0);
    double lift_height = params->step_height;
    double move_distance = params->step_length;

    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        if ((leg % 2 == 0) == is_first_group)
        {
            /* Lifting phase */
            leg_position_t pos = {
                .hip = 0.0,
                .knee = lift_height,
                .ankle = 0.0};
            hexapod_set_leg_position(leg, &pos);
        }
        else
        {
            // Moving phase
            leg_position_t pos = {
                .hip = move_distance * ((step % 2) ? 1 : -1),
                .knee = 0.0,
                .ankle = 0.0};
            hexapod_set_leg_position(leg, &pos);
        }
    }

    return 0;
}

int wave_gait_step(int step, const gait_params_t *params)
{
    if (!params)
    {
        return -1;
    }

    // Wave gait moves one leg at a time
    int current_leg = step % NUM_LEGS;
    double lift_height = params->step_height;
    double move_distance = params->step_length;

    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        if (leg == current_leg)
        {
            // Current leg is in lifting phase
            leg_position_t pos = {
                .hip = 0.0,
                .knee = lift_height,
                .ankle = 0.0};
            hexapod_set_leg_position(leg, &pos);
        }
        else
        {
            // Other legs are in support phase
            leg_position_t pos = {
                .hip = move_distance * ((step < NUM_LEGS) ? -1 : 1),
                .knee = 0.0,
                .ankle = 0.0};
            hexapod_set_leg_position(leg, &pos);
        }
    }

    return 0;
}

/* Ripple gait function */
int ripple_gait_step(int step, const gait_params_t *params)
{
    if (!params)
    {
        return -1;
    }

    // Ripple gait moves legs in sequence based on step number
    double lift_height = params->step_height;
    int current_leg = step % NUM_LEGS;

    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        leg_position_t pos = {
            .hip = (leg == current_leg) ? params->step_length : 0.0,
            .knee = (leg == current_leg) ? lift_height : 0.0,
            .ankle = 0.0};
        hexapod_set_leg_position(leg, &pos);
    }

    return 0;
}