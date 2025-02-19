#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>
#include "../../kernel_driver/include/hexapod_ioctl.h"

// Movement patterns
#define PATTERN_TRIPOD 0
#define PATTERN_WAVE   1
#define PATTERN_RIPPLE 2

void test_single_servo(int fd, int leg_id, int joint_id, int angle)
{
    struct servo_control ctrl = {
        .leg_id = leg_id,
        .joint_id = joint_id,
        .angle = angle
    };

    if (ioctl(fd, IOCTL_SET_SERVO, &ctrl) < 0) {
        printf("Error setting leg %d joint %d to angle %d: %s\n", 
               leg_id, joint_id, angle, strerror(errno));
    } else {
        printf("Set leg %d joint %d to angle %d\n", leg_id, joint_id, angle);
    }
}

void test_move_leg(int fd, int leg_id, int coxa, int femur, int tibia)
{
    printf("Moving leg %d to position (coxa=%d, femur=%d, tibia=%d)\n",
           leg_id, coxa, femur, tibia);

    struct servo_control ctrl = {
        .leg_id = leg_id,
        .joint_id = 0,  // Doesn't matter, all joints moved at once
        .angle = 0      // Will be set in the kernel
    };

    if (ioctl(fd, IOCTL_MOVE_LEG, &ctrl) < 0) {
        printf("Error moving leg %d: %s\n", leg_id, strerror(errno));
    }
}

void test_pattern(int fd, int pattern_id, int speed, int direction)
{
    struct movement_pattern pattern = {
        .pattern_id = pattern_id,
        .speed = speed,
        .direction = direction
    };

    if (ioctl(fd, IOCTL_SET_PATTERN, &pattern) < 0) {
        printf("Error setting pattern %d: %s\n", pattern_id, strerror(errno));
    } else {
        printf("Started pattern %d with speed %d and direction %d\n",
               pattern_id, speed, direction);
    }
}

void test_mpu6050(int fd)
{
    struct mpu6050_data data;

    if (ioctl(fd, IOCTL_GET_MPU6050, &data) < 0) {
        printf("Error reading MPU6050: %s\n", strerror(errno));
        return;
    }

    printf("MPU6050 Data:\n");
    printf("  Accelerometer: X=%6d Y=%6d Z=%6d\n", 
           data.accel_x, data.accel_y, data.accel_z);
    printf("  Gyroscope:    X=%6d Y=%6d Z=%6d\n",
           data.gyro_x, data.gyro_y, data.gyro_z);
    printf("  Temperature:  %6d\n", data.temp);
}

void test_wave_pattern(int fd)
{
    printf("Running wave pattern test...\n");
    test_pattern(fd, PATTERN_WAVE, 5, 1);  // Speed 5, forward direction
    sleep(10);  // Run for 10 seconds
    test_pattern(fd, PATTERN_WAVE, 0, 0);  // Stop
}

void test_tripod_pattern(int fd)
{
    printf("Running tripod pattern test...\n");
    test_pattern(fd, PATTERN_TRIPOD, 7, 1);  // Speed 7, forward direction
    sleep(10);  // Run for 10 seconds
    test_pattern(fd, PATTERN_TRIPOD, 0, 0);  // Stop
}

void print_usage(const char *program_name)
{
    printf("Usage: %s [OPTIONS]\n", program_name);
    printf("Options:\n");
    printf("  -s <leg_id> -j <joint_id> -a <angle>  Set single servo\n");
    printf("  -l <leg_id> -c <coxa> -f <femur> -t <tibia>  Move entire leg\n");
    printf("  -p <pattern_id> -v <speed> -d <direction>  Run movement pattern\n");
    printf("  -m  Read MPU6050 sensor data\n");
    printf("  -w  Run wave pattern test\n");
    printf("  -r  Run tripod pattern test\n");
    printf("  -h  Show this help message\n");
}

int main(int argc, char *argv[])
{
    int fd, opt;
    int leg_id = -1, joint_id = -1, angle = -1;
    int coxa = -1, femur = -1, tibia = -1;
    int pattern_id = -1, speed = 5, direction = 1;
    int read_mpu = 0;
    int run_wave = 0;
    int run_tripod = 0;

    // Parse command line arguments
    while ((opt = getopt(argc, argv, "s:j:a:l:c:f:t:p:v:d:mwrh")) != -1) {
        switch (opt) {
            case 's': leg_id = atoi(optarg); break;
            case 'j': joint_id = atoi(optarg); break;
            case 'a': angle = atoi(optarg); break;
            case 'l': leg_id = atoi(optarg); break;
            case 'c': coxa = atoi(optarg); break;
            case 'f': femur = atoi(optarg); break;
            case 't': tibia = atoi(optarg); break;
            case 'p': pattern_id = atoi(optarg); break;
            case 'v': speed = atoi(optarg); break;
            case 'd': direction = atoi(optarg); break;
            case 'm': read_mpu = 1; break;
            case 'w': run_wave = 1; break;
            case 'r': run_tripod = 1; break;
            case 'h':
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    // Open device
    fd = open("/dev/hexapod", O_RDWR);
    if (fd < 0) {
        printf("Error opening device: %s\n", strerror(errno));
        return 1;
    }

    // Execute requested operation
    if (leg_id >= 0 && joint_id >= 0 && angle >= 0) {
        test_single_servo(fd, leg_id, joint_id, angle);
    }
    else if (leg_id >= 0 && coxa >= 0 && femur >= 0 && tibia >= 0) {
        test_move_leg(fd, leg_id, coxa, femur, tibia);
    }
    else if (pattern_id >= 0) {
        test_pattern(fd, pattern_id, speed, direction);
    }
    else if (read_mpu) {
        test_mpu6050(fd);
    }
    else if (run_wave) {
        test_wave_pattern(fd);
    }
    else if (run_tripod) {
        test_tripod_pattern(fd);
    }
    else {
        print_usage(argv[0]);
        close(fd);
        return 1;
    }

    close(fd);
    return 0;
}