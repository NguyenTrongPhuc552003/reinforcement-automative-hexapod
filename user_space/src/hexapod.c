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
static leg_position_t last_positions[NUM_LEGS] = {0}; // Add this to track positions

int hexapod_init(void)
{
    device_fd = open(DEVICE_PATH, O_RDWR);
    if (device_fd < 0)
    {
        fprintf(stderr, "Failed to open hexapod device: %s\n", strerror(errno));
        return -1;
    }
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
}

int hexapod_set_leg_position(uint8_t leg_num, const leg_position_t *position)
{
    if (device_fd < 0 || !position || leg_num >= NUM_LEGS)
    {
        fprintf(stderr, "ERROR: Invalid parameters (fd=%d, leg=%d)\n",
                device_fd, leg_num);
        return -1;
    }

    struct leg_command cmd = {
        .leg_num = leg_num,
        .position = {
            .hip = position->hip * 100, // Convert to centidegrees
            .knee = position->knee * 100,
            .ankle = position->ankle * 100}};

    // printf("DEBUG: hexapod_set_leg_position: fd=%d, leg=%d, angles=(%.2f,%.2f,%.2f)\n",
    //        device_fd, leg_num, position->hip, position->knee, position->ankle);
    // printf("DEBUG: Sending IOCTL cmd=0x%x\n", SET_LEG_POSITION);

    int ret = ioctl(device_fd, SET_LEG_POSITION, &cmd);
    // printf("DEBUG: ioctl SET_LEG_POSITION returned %d\n", ret);
    if (ret < 0)
    {
        fprintf(stderr, "ERROR: IOCTL failed: %s\n", strerror(errno));
        return -1;
    }

    // Store the position
    last_positions[leg_num] = *position;
    return 0;
}

int hexapod_get_leg_position(uint8_t leg_num, leg_position_t *position)
{
    if (device_fd < 0 || !position || leg_num >= NUM_LEGS)
    {
        return -EINVAL;
    }

    // Return the last known position
    *position = last_positions[leg_num];
    return 0;
}

int hexapod_get_imu_data(imu_data_t *data)
{
    if (device_fd < 0 || !data)
    {
        return -EINVAL;
    }

    if (ioctl(device_fd, GET_IMU_DATA, data) < 0)
    {
        fprintf(stderr, "Failed to get IMU data: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}

int hexapod_center_all_legs(void)
{
    if (device_fd < 0)
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