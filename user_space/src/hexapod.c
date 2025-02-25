#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include "hexapod.h"
#include "protocol.h"

static int device_fd = -1;
static hexapod_state_t current_state = {0};

int hexapod_init(void)
{
    device_fd = open(DEVICE_PATH, O_RDWR);
    if (device_fd < 0)
    {
        fprintf(stderr, "Failed to open hexapod device: %s\n", strerror(errno));
        return -1;
    }

    current_state.initialized = 1;
    return hexapod_center_all_legs();
}

void hexapod_cleanup(void)
{
    if (device_fd >= 0)
    {
        hexapod_center_all_legs();
        close(device_fd);
        device_fd = -1;
    }
    current_state.initialized = 0;
}

int hexapod_set_leg_position(uint8_t leg_num, const leg_position_t *position)
{
    if (!current_state.initialized || !position || leg_num >= NUM_LEGS)
    {
        return -1;
    }

    struct leg_command cmd = {
        .leg_num = leg_num,
        .position = {
            .hip = position->hip * 100, // Convert to centidegrees
            .knee = position->knee * 100,
            .ankle = position->ankle * 100}};

    if (ioctl(device_fd, SET_LEG_POSITION, &cmd) < 0)
    {
        perror("Failed to set leg position");
        return -1;
    }

    current_state.legs[leg_num] = *position;
    return 0;
}

int hexapod_get_leg_position(uint8_t leg_num, leg_position_t *position)
{
    if (!current_state.initialized || !position || leg_num >= NUM_LEGS)
    {
        return -EINVAL;
    }

    *position = current_state.legs[leg_num];
    return 0;
}

int hexapod_get_imu_data(imu_data_t *data)
{
    if (!current_state.initialized || !data)
    {
        return -EINVAL;
    }

    if (ioctl(device_fd, GET_IMU_DATA, data) < 0)
    {
        fprintf(stderr, "Failed to get IMU data: %s\n", strerror(errno));
        return -1;
    }

    current_state.imu_data = *data;
    return 0;
}

int hexapod_center_all_legs(void)
{
    if (!current_state.initialized)
    {
        return -EINVAL;
    }

    leg_position_t center_pos = {0.0, 0.0, 0.0};
    int ret = 0;

    for (uint8_t i = 0; i < NUM_LEGS; i++)
    {
        if (hexapod_set_leg_position(i, &center_pos) < 0)
        {
            ret = -1;
        }
    }

    return ret;
}