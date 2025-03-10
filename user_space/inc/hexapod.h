#ifndef HEXAPOD_H
#define HEXAPOD_H

#include <stdint.h>

/* Robot dimensions in mm */
#define COXA_LENGTH 30
#define FEMUR_LENGTH 85
#define TIBIA_LENGTH 130

/* Number of legs and servos */
#define NUM_LEGS 6
#define SERVOS_PER_LEG 3
#define TOTAL_SERVOS (NUM_LEGS * SERVOS_PER_LEG)

/* Angle limits */
#define HIP_MIN_ANGLE -90
#define HIP_MAX_ANGLE 90
#define KNEE_MIN_ANGLE -90
#define KNEE_MAX_ANGLE 90
#define ANKLE_MIN_ANGLE -90
#define ANKLE_MAX_ANGLE 90

/* Structures for communication */
typedef struct leg_position
{
    int16_t hip;
    int16_t knee;
    int16_t ankle;
} leg_position_t;

typedef struct imu_data
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} imu_data_t;

typedef struct leg_command
{
    uint8_t leg_num;
    leg_position_t position;
} leg_command_t;

typedef struct calibration
{
    uint8_t leg_num;
    int16_t hip_offset;
    int16_t knee_offset;
    int16_t ankle_offset;
} calibration_t;

/* Hexapod state */
typedef struct hexapod
{
    int fd;                                 /* File descriptor for device */
    int initialized;                        /* Initialization flag */
    leg_position_t leg_positions[NUM_LEGS]; /* Current leg positions */
    imu_data_t imu_data;                    /* Last read IMU data */
} hexapod_t;

/* Main API functions */
int hexapod_init(hexapod_t *hex);
void hexapod_cleanup(hexapod_t *hex);

int hexapod_set_leg_position(hexapod_t *hex, uint8_t leg_num, const leg_position_t *position);
int hexapod_get_leg_position(hexapod_t *hex, uint8_t leg_num, leg_position_t *position);
int hexapod_get_imu_data(hexapod_t *hex, imu_data_t *data);
int hexapod_set_calibration(hexapod_t *hex, uint8_t leg_num,
                            int16_t hip_offset, int16_t knee_offset, int16_t ankle_offset);
int hexapod_center_all(hexapod_t *hex);

/* Utility functions */
const char *hexapod_error_string(int error_code);
void hexapod_print_leg_position(const leg_position_t *position);
void hexapod_print_imu_data(const imu_data_t *data);

#endif /* HEXAPOD_H */
