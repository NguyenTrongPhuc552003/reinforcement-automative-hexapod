#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "common.h"
#include "protocol.h"
#include "hexerror.h"

/**
 * Check if leg number is valid
 */
int hexapod_validate_leg_num(uint8_t leg_num)
{
    if (leg_num >= NUM_LEGS)
        return HEXAPOD_EINVAL;
    return HEXAPOD_SUCCESS;
}

/**
 * Check if angles are within valid ranges
 */
int hexapod_validate_angles(const struct leg_position *position)
{
    if (!position)
        return HEXAPOD_EINVAL;

    // Convert centidegrees to actual degrees for comparison
    int hip = position->hip / 100;
    int knee = position->knee / 100;
    int ankle = position->ankle / 100;

    if (hip < HIP_MIN_ANGLE || hip > HIP_MAX_ANGLE ||
        knee < KNEE_MIN_ANGLE || knee > KNEE_MAX_ANGLE ||
        ankle < ANKLE_MIN_ANGLE || ankle > ANKLE_MAX_ANGLE)
    {
        return HEXAPOD_ERANGE;
    }

    return HEXAPOD_SUCCESS;
}

/**
 * Initialize leg position to default neutral position
 */
void hexapod_init_leg_position(struct leg_position *position)
{
    if (position)
    {
        position->hip = 0;
        position->knee = 0;
        position->ankle = 0;
    }
}

/**
 * Open the hexapod device
 */
int hexapod_open_device(void)
{
    int fd = open(HEXAPOD_DEVICE_PATH, O_RDWR);
    if (fd < 0)
    {
        perror("Failed to open hexapod device");
        return HEXAPOD_ENODEV;
    }
    return fd;
}

/**
 * Close the hexapod device
 */
void hexapod_close_device(int fd)
{
    if (fd >= 0)
    {
        close(fd);
    }
}

/**
 * Send command to device using ioctl
 */
int hexapod_send_command(int fd, unsigned long cmd, void *data)
{
    if (fd < 0 || !data)
        return HEXAPOD_EINVAL;

    int ret = ioctl(fd, cmd, data);
    if (ret < 0)
    {
        perror("Hexapod ioctl failed");
        return HEXAPOD_EIO;
    }

    return HEXAPOD_SUCCESS;
}

/**
 * Set calibration for a leg
 */
int hexapod_set_calibration(int fd, uint8_t leg_num,
                            int16_t hip_offset, int16_t knee_offset,
                            int16_t ankle_offset)
{
    struct leg_calibration cal;

    if (fd < 0 || leg_num >= NUM_LEGS)
        return HEXAPOD_EINVAL;

    cal.leg_num = leg_num;
    cal.hip_offset = hip_offset;
    cal.knee_offset = knee_offset;
    cal.ankle_offset = ankle_offset;

    return hexapod_send_command(fd, SET_CALIBRATION, &cal);
}

/**
 * Print hexapod version information
 */
void hexapod_print_version(void)
{
    printf("Hexapod Control Library\n");
    printf("Version: 1.0.0\n");
    printf("Protocol Version: 1.0\n");
}
