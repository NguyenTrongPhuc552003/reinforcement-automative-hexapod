#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <time.h>
#include "hexapod.h"
#include "protocol.h"
#include "common.h"

/* Internal state structure */
typedef struct {
    int device_fd;
    int initialized;
    leg_position_t leg_positions[NUM_LEGS];
    imu_data_t imu_data;
} hexapod_internal_state_t;

static int hexapod_fd = -1;
static hexapod_internal_state_t hexapod_state = {0};

/* Initialization time for relative timing */
static struct timespec start_time;

/**
 * Initialize the hexapod control system
 */
int hexapod_init(void)
{
    if (hexapod_state.initialized)
    {
        return HEXAPOD_SUCCESS; // Already initialized
    }

    // Open device
    hexapod_fd = hexapod_open_device();
    if (hexapod_fd < 0)
    {
        return hexapod_fd;
    }

    // Initialize state
    memset(&hexapod_state, 0, sizeof(hexapod_internal_state_t));
    hexapod_state.device_fd = hexapod_fd;
    hexapod_state.initialized = 1;

    printf("Hexapod initialized successfully\n");
    hexapod_print_version();

    /* Record start time for timing functions */
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    return HEXAPOD_SUCCESS;
}

/**
 * Clean up resources
 */
void hexapod_cleanup(void)
{
    if (hexapod_state.initialized)
    {
        hexapod_close_device(hexapod_fd);
        hexapod_fd = -1;
        hexapod_state.initialized = 0;
    }
}

/**
 * Set position for a specific leg
 */
int hexapod_set_leg_position(uint8_t leg_num, const leg_position_t *position)
{
    struct leg_command cmd;
    int ret;

    if (!hexapod_state.initialized)
    {
        return HEXAPOD_ENOTINIT;
    }

    ret = hexapod_validate_leg_num(leg_num);
    if (ret != HEXAPOD_SUCCESS)
    {
        return ret;
    }

    // Check if position is valid
    if (!position)
    {
        return HEXAPOD_EINVAL;
    }

    cmd.leg_num = leg_num;
    memcpy(&cmd.position, position, sizeof(struct leg_position));

    ret = hexapod_send_command(hexapod_fd, SET_LEG_POSITION, &cmd);
    if (ret == HEXAPOD_SUCCESS)
    {
        // Update local state
        memcpy(&hexapod_state.leg_positions[leg_num], position, sizeof(struct leg_position));
    }

    return ret;
}

/**
 * Get current position of a specific leg
 */
int hexapod_get_leg_position(uint8_t leg_num, leg_position_t *position)
{
    if (!hexapod_state.initialized)
    {
        return HEXAPOD_ENOTINIT;
    }

    int ret = hexapod_validate_leg_num(leg_num);
    if (ret != HEXAPOD_SUCCESS)
    {
        return ret;
    }

    if (!position)
    {
        return HEXAPOD_EINVAL;
    }

    // In real implementation, we might want to get actual position from hardware
    // But for now we just return the cached position
    memcpy(position, &hexapod_state.leg_positions[leg_num], sizeof(struct leg_position));

    return HEXAPOD_SUCCESS;
}

/**
 * Get IMU data
 */
int hexapod_get_imu_data(imu_data_t *data)
{
    if (!hexapod_state.initialized)
    {
        return HEXAPOD_ENOTINIT;
    }

    if (!data)
    {
        return HEXAPOD_EINVAL;
    }

    int ret = hexapod_send_command(hexapod_fd, GET_IMU_DATA, data);
    if (ret == HEXAPOD_SUCCESS)
    {
        // Update local state
        memcpy(&hexapod_state.imu_data, data, sizeof(imu_data_t));
    }

    return ret;
}

/**
 * Center all legs
 */
int hexapod_center_all_legs(void)
{
    int ret = HEXAPOD_SUCCESS;
    leg_position_t center_position = {0, 0, 0};

    if (!hexapod_state.initialized)
    {
        return HEXAPOD_ENOTINIT;
    }

    for (uint8_t i = 0; i < NUM_LEGS; i++)
    {
        int leg_ret = hexapod_set_leg_position(i, &center_position);
        if (leg_ret != HEXAPOD_SUCCESS)
        {
            ret = leg_ret; // Return last error
        }
    }

    return ret;
}

/**
 * Set calibration for a specific leg
 */
int hexapod_set_calibration(int fd, uint8_t leg_num, int16_t hip_offset, int16_t knee_offset, int16_t ankle_offset)
{
    if (!hexapod_state.initialized)
    {
        return HEXAPOD_ENOTINIT;
    }

    int ret = hexapod_validate_leg_num(leg_num);
    if (ret != HEXAPOD_SUCCESS)
    {
        return ret;
    }

    return hexapod_set_calibration(fd, leg_num, hip_offset, knee_offset, ankle_offset);
}

/**
 * Get the elapsed time since initialization
 */
double hexapod_get_time(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    /* Calculate elapsed time in seconds */
    double elapsed = (now.tv_sec - start_time.tv_sec) +
                     (now.tv_nsec - start_time.tv_nsec) / 1.0e9;

    return elapsed;
}

/**
 * Sleep for a specified duration
 */
void hexapod_sleep(double seconds)
{
    struct timespec ts;
    ts.tv_sec = (time_t)seconds;
    ts.tv_nsec = (long)((seconds - ts.tv_sec) * 1.0e9);
    nanosleep(&ts, NULL);
}