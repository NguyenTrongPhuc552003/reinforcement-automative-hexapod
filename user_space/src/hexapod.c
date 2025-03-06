#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "hexapod.h"

/* Device file path */
#define HEXAPOD_DEVICE "/dev/hexapod"

/* IOCTL commands - must match kernel definitions */
#define HEXAPOD_IOC_MAGIC 'H'
#define HEXAPOD_IOCTL_SET_LEG _IOW(HEXAPOD_IOC_MAGIC, 1, leg_command_t)
#define HEXAPOD_IOCTL_GET_IMU _IOR(HEXAPOD_IOC_MAGIC, 2, imu_data_t)
#define HEXAPOD_IOCTL_CALIBRATE _IOW(HEXAPOD_IOC_MAGIC, 3, calibration_t)
#define HEXAPOD_IOCTL_CENTER_ALL _IO(HEXAPOD_IOC_MAGIC, 4)

/* Error code mapping */
const char *hexapod_error_string(int error_code)
{
    switch (error_code)
    {
    case 0:
        return "Success";
    case -EINVAL:
        return "Invalid argument";
    case -ENODEV:
        return "No such device";
    case -ENOMEM:
        return "Out of memory";
    case -EBUSY:
        return "Device or resource busy";
    case -ETIMEDOUT:
        return "Operation timed out";
    case -ENOENT:
        return "No such entry";
    case -EBADMSG:
        return "Bad message";
    default:
        return strerror(-error_code);
    }
}

/* Initialize the hexapod system */
int hexapod_init(hexapod_t *hex)
{
    if (!hex)
        return -EINVAL;

    /* Clear structure */
    memset(hex, 0, sizeof(hexapod_t));

    /* Open the device */
    hex->fd = open(HEXAPOD_DEVICE, O_RDWR);
    if (hex->fd < 0)
    {
        int err = -errno;
        fprintf(stderr, "Failed to open hexapod device: %s\n", strerror(errno));
        return err;
    }

    hex->initialized = 1;
    printf("Hexapod initialized successfully\n");

    return 0;
}

/* Clean up hexapod resources */
void hexapod_cleanup(hexapod_t *hex)
{
    if (hex && hex->initialized)
    {
        if (hex->fd >= 0)
        {
            close(hex->fd);
            hex->fd = -1;
        }
        hex->initialized = 0;
    }
}

/* Set leg position */
int hexapod_set_leg_position(hexapod_t *hex, uint8_t leg_num, const leg_position_t *position)
{
    leg_command_t cmd;
    int ret;

    if (!hex || !hex->initialized || leg_num >= NUM_LEGS || !position)
        return -EINVAL;

    if (hex->fd < 0)
        return -EBADF;

    /* Prepare command */
    cmd.leg_num = leg_num;
    memcpy(&cmd.position, position, sizeof(leg_position_t));

    /* Send command to kernel */
    ret = ioctl(hex->fd, HEXAPOD_IOCTL_SET_LEG, &cmd);
    if (ret < 0)
    {
        ret = -errno;
        fprintf(stderr, "Failed to set leg position: %s\n", strerror(errno));
        return ret;
    }

    /* Store position in local state */
    memcpy(&hex->leg_positions[leg_num], position, sizeof(leg_position_t));
    return 0;
}

/* Get leg position */
int hexapod_get_leg_position(hexapod_t *hex, uint8_t leg_num, leg_position_t *position)
{
    if (!hex || !hex->initialized || leg_num >= NUM_LEGS || !position)
        return -EINVAL;

    /* We don't have feedback, so return last command */
    memcpy(position, &hex->leg_positions[leg_num], sizeof(leg_position_t));
    return 0;
}

/* Get IMU data */
int hexapod_get_imu_data(hexapod_t *hex, imu_data_t *data)
{
    int ret;

    if (!hex || !hex->initialized || !data)
        return -EINVAL;

    if (hex->fd < 0)
        return -EBADF;

    /* Get IMU data from kernel */
    ret = ioctl(hex->fd, HEXAPOD_IOCTL_GET_IMU, data);
    if (ret < 0)
    {
        ret = -errno;
        fprintf(stderr, "Failed to get IMU data: %s\n", strerror(errno));
        return ret;
    }

    /* Store data in local state */
    memcpy(&hex->imu_data, data, sizeof(imu_data_t));
    return 0;
}

/* Set calibration values */
int hexapod_set_calibration(hexapod_t *hex, uint8_t leg_num,
                            int16_t hip_offset, int16_t knee_offset, int16_t ankle_offset)
{
    calibration_t cal;
    int ret;

    if (!hex || !hex->initialized || leg_num >= NUM_LEGS)
        return -EINVAL;

    if (hex->fd < 0)
        return -EBADF;

    /* Prepare calibration data */
    cal.leg_num = leg_num;
    cal.hip_offset = hip_offset;
    cal.knee_offset = knee_offset;
    cal.ankle_offset = ankle_offset;

    /* Send calibration to kernel */
    ret = ioctl(hex->fd, HEXAPOD_IOCTL_CALIBRATE, &cal);
    if (ret < 0)
    {
        ret = -errno;
        fprintf(stderr, "Failed to set calibration: %s\n", strerror(errno));
        return ret;
    }

    return 0;
}

/* Center all legs */
int hexapod_center_all(hexapod_t *hex)
{
    int ret;
    uint8_t i;

    if (!hex || !hex->initialized)
        return -EINVAL;

    if (hex->fd < 0)
        return -EBADF;

    /* Send center command to kernel */
    ret = ioctl(hex->fd, HEXAPOD_IOCTL_CENTER_ALL);
    if (ret < 0)
    {
        ret = -errno;
        fprintf(stderr, "Failed to center all legs: %s\n", strerror(errno));
        return ret;
    }

    /* Clear local state */
    for (i = 0; i < NUM_LEGS; i++)
    {
        hex->leg_positions[i].hip = 0;
        hex->leg_positions[i].knee = 0;
        hex->leg_positions[i].ankle = 0;
    }

    return 0;
}

/* Utility functions */
void hexapod_print_leg_position(const leg_position_t *position)
{
    if (!position)
        return;

    printf("Hip: %d, Knee: %d, Ankle: %d\n",
           position->hip, position->knee, position->ankle);
}

void hexapod_print_imu_data(const imu_data_t *data)
{
    if (!data)
        return;

    printf("Accel: X=%d Y=%d Z=%d, Gyro: X=%d Y=%d Z=%d\n",
           data->accel_x, data->accel_y, data->accel_z,
           data->gyro_x, data->gyro_y, data->gyro_z);
}
