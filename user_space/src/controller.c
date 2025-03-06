#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include "controller.h"

static hexapod_t *hexapod;
static controller_params_t params;
static struct termios orig_termios;
static struct timespec last_update;

/* Initialize controller */
int controller_init(hexapod_t *hex)
{
    struct termios new_termios;

    if (!hex)
        return -1;
    hexapod = hex;

    /* Initialize parameters */
    params.speed = 0.5;
    params.direction = 0.0;
    params.height = 0.0;
    params.tilt_x = 0.0;
    params.tilt_y = 0.0;
    params.gait = GAIT_TRIPOD;
    params.state = CTRL_STATE_IDLE;

    /* Set up terminal for immediate key input */
    tcgetattr(STDIN_FILENO, &orig_termios);
    new_termios = orig_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

    /* Initialize gait controller */
    gait_init(hexapod, &(gait_params_t){
                           .type = params.gait,
                           .step_height = 30.0,
                           .step_length = 60.0,
                           .cycle_time = 1.0,
                           .duty_factor = 0.5});

    clock_gettime(CLOCK_MONOTONIC, &last_update);
    return 0;
}

/* Process keyboard input */
int controller_process_key(char key)
{
    switch (key)
    {
    case 'w': /* Forward */
        params.direction = 0.0;
        params.state = CTRL_STATE_WALKING;
        break;
    case 's': /* Backward */
        params.direction = 180.0;
        params.state = CTRL_STATE_WALKING;
        break;
    case 'a': /* Rotate left */
        params.direction = -90.0;
        params.state = CTRL_STATE_ROTATING;
        break;
    case 'd': /* Rotate right */
        params.direction = 90.0;
        params.state = CTRL_STATE_ROTATING;
        break;
    case 'i': /* Raise body */
        params.height += 10.0;
        break;
    case 'k': /* Lower body */
        params.height -= 10.0;
        break;
    case 'j': /* Tilt left */
        params.tilt_y = -15.0;
        params.state = CTRL_STATE_TILTING;
        break;
    case 'l': /* Tilt right */
        params.tilt_y = 15.0;
        params.state = CTRL_STATE_TILTING;
        break;
    case ' ': /* Stop */
        params.state = CTRL_STATE_IDLE;
        return hexapod_center_all(hexapod);
    default:
        return 0;
    }
    return 0;
}

/* Update hexapod state */
int controller_update(void)
{
    struct timespec now;
    double time_delta;

    clock_gettime(CLOCK_MONOTONIC, &now);
    time_delta = (now.tv_sec - last_update.tv_sec) +
                 (now.tv_nsec - last_update.tv_nsec) / 1.0e9;

    switch (params.state)
    {
    case CTRL_STATE_WALKING:
    case CTRL_STATE_ROTATING:
        return gait_update(hexapod, time_delta,
                           params.direction, params.speed);
    case CTRL_STATE_TILTING:
        /* Implement tilt control here */
        break;
    case CTRL_STATE_IDLE:
        /* Do nothing */
        break;
    }

    last_update = now;
    return 0;
}

/* Cleanup */
void controller_cleanup(void)
{
    /* Restore terminal settings */
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);

    /* Center hexapod */
    hexapod_center_all(hexapod);
    gait_cleanup();
}
