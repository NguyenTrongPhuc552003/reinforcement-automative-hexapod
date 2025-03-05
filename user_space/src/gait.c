#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <unistd.h> // Add this include for usleep
#include "gait.h"
#include "kinematics.h"
#include "hexapod.h"

/* Default parameters */
static gait_params_t current_params;
static int gait_initialized = 0;

/* Internal gait state */
typedef struct
{
    double phase[NUM_LEGS];      /* Phase of each leg (0-1) */
    point3d_t neutral[NUM_LEGS]; /* Neutral position for each leg */
    point3d_t current[NUM_LEGS]; /* Current target position */
    double last_time;            /* Last update time */
} gait_state_t;

static gait_state_t gait_state;

/* Compute leg phase based on gait type */
static void compute_leg_phases(gait_type_t type)
{
    /* Default leg phases for different gaits */
    switch (type)
    {
    case GAIT_TRIPOD:
        /* Tripod: legs 0,2,4 in one group, legs 1,3,5 in another */
        gait_state.phase[0] = 0.0;
        gait_state.phase[2] = 0.0;
        gait_state.phase[4] = 0.0;
        gait_state.phase[1] = 0.5;
        gait_state.phase[3] = 0.5;
        gait_state.phase[5] = 0.5;
        break;

    case GAIT_WAVE:
        /* Wave: all legs equally spaced */
        for (int i = 0; i < NUM_LEGS; i++)
        {
            gait_state.phase[i] = (double)i / NUM_LEGS;
        }
        break;

    case GAIT_RIPPLE:
        /* Ripple: legs 0,3 | 1,4 | 2,5 in groups */
        gait_state.phase[0] = 0.0;
        gait_state.phase[3] = 0.0;
        gait_state.phase[1] = 0.33;
        gait_state.phase[4] = 0.33;
        gait_state.phase[2] = 0.67;
        gait_state.phase[5] = 0.67;
        break;

    default:
        /* Default to wave gait */
        for (int i = 0; i < NUM_LEGS; i++)
        {
            gait_state.phase[i] = (double)i / NUM_LEGS;
        }
    }
}

/* Set up neutral stance positions for each leg */
static void setup_neutral_positions(void)
{
    /* Hexapod layout (top view)
     *    0 --- 1
     *   /|     |\
     *  5 |     | 2
     *   \|     |/
     *    4 --- 3
     */

    /* These values depend on the specific robot dimensions */
    /* Using simple symmetrical arrangement for now */
    double radius = 100.0; /* Distance from center to each leg */
    double height = -80.0; /* Default standing height */

    /* Front right leg (0) */
    gait_state.neutral[0].x = radius * cos(M_PI / 4);
    gait_state.neutral[0].y = radius * sin(M_PI / 4);
    gait_state.neutral[0].z = height;

    /* Front left leg (1) */
    gait_state.neutral[1].x = radius * cos(3 * M_PI / 4);
    gait_state.neutral[1].y = radius * sin(3 * M_PI / 4);
    gait_state.neutral[1].z = height;

    /* Middle left leg (2) */
    gait_state.neutral[2].x = 0.0;
    gait_state.neutral[2].y = radius;
    gait_state.neutral[2].z = height;

    /* Back left leg (3) */
    gait_state.neutral[3].x = -radius * cos(M_PI / 4);
    gait_state.neutral[3].y = radius * sin(M_PI / 4);
    gait_state.neutral[3].z = height;

    /* Back right leg (4) */
    gait_state.neutral[4].x = -radius * cos(M_PI / 4);
    gait_state.neutral[4].y = -radius * sin(M_PI / 4);
    gait_state.neutral[4].z = height;

    /* Middle right leg (5) */
    gait_state.neutral[5].x = 0.0;
    gait_state.neutral[5].y = -radius;
    gait_state.neutral[5].z = height;

    /* Initialize current positions to neutral */
    memcpy(gait_state.current, gait_state.neutral, sizeof(gait_state.neutral));
}

/* Initialize gait controller */
int gait_init(const gait_params_t *params)
{
    if (!params)
    {
        return -1;
    }

    /* Store parameters */
    memcpy(&current_params, params, sizeof(gait_params_t));

    /* Set up phases based on gait type */
    compute_leg_phases(params->type);

    /* Set up neutral positions */
    setup_neutral_positions();

    /* Initialize timing */
    gait_state.last_time = 0.0;

    gait_initialized = 1;
    return 0;
}

/* Calculate leg position during walking cycle */
static int calculate_leg_position(int leg_num, double cycle_phase, point3d_t *position)
{
    double leg_phase, phase;
    double direction_rad;
    double stride_x, stride_y;

    if (leg_num >= NUM_LEGS || !position)
    {
        return -1;
    }

    /* Adjust phase for this leg */
    leg_phase = gait_state.phase[leg_num];
    phase = fmod(cycle_phase + leg_phase, 1.0);

    /* Convert direction to radians */
    direction_rad = current_params.direction * M_PI / 180.0;

    /* Calculate stride vector */
    stride_x = current_params.step_length * cos(direction_rad) * current_params.speed;
    stride_y = current_params.step_length * sin(direction_rad) * current_params.speed;

    /* Start with neutral position */
    *position = gait_state.neutral[leg_num];

    /* Support phase (on ground) */
    if (phase >= current_params.duty_factor)
    {
        /* Flight phase (leg in air) */
        double flight_phase = (phase - current_params.duty_factor) / (1.0 - current_params.duty_factor);
        double swing_x = stride_x * (1.0 - 2.0 * flight_phase);
        double swing_y = stride_y * (1.0 - 2.0 * flight_phase);

        /* Parabolic trajectory for leg height */
        double lift = current_params.step_height * sin(flight_phase * M_PI);

        position->x += swing_x;
        position->y += swing_y;
        position->z -= lift; /* Negative because z is down in our coordinate system */
    }
    else
    {
        /* Support phase (on ground) */
        double support_phase = phase / current_params.duty_factor;

        position->x += stride_x * (1.0 - support_phase);
        position->y += stride_y * (1.0 - support_phase);
    }

    return 0;
}

/* Update gait and calculate new leg positions */
int gait_update(double time, hexapod_state_t *state)
{
    double cycle_phase;
    leg_position_t angles;
    point3d_t position;
    int ret;

    if (!gait_initialized || !state)
    {
        return -1;
    }

    /* Calculate cycle phase (0-1) */
    cycle_phase = fmod(time / current_params.cycle_time, 1.0);

    /* Update each leg */
    for (int i = 0; i < NUM_LEGS; i++)
    {
        ret = calculate_leg_position(i, cycle_phase, &position);
        if (ret < 0)
        {
            continue;
        }

        /* Convert position to angles using inverse kinematics */
        ret = inverse_kinematics(&position, &angles);
        if (ret < 0)
        {
            /* If position is unreachable, use last valid position */
            continue;
        }

        /* Update leg position */
        ret = hexapod_set_leg_position(i, &angles);
        if (ret < 0)
        {
            continue;
        }

        /* Store position in state */
        state->legs[i] = angles;

        /* Remember current position */
        gait_state.current[i] = position;
    }

    /* Store last update time */
    gait_state.last_time = time;

    return 0;
}

/* Clean up gait controller */
void gait_cleanup(void)
{
    /* Return legs to neutral position */
    for (int i = 0; i < NUM_LEGS; i++)
    {
        leg_position_t angles;
        if (inverse_kinematics(&gait_state.neutral[i], &angles) == 0)
        {
            hexapod_set_leg_position(i, &angles);
        }
    }

    gait_initialized = 0;
}