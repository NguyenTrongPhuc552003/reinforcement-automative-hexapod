#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include "../include/hexapod_ioctl_user.h"

#define DEVICE_PATH "/dev/hexapod"

// Movement patterns
#define PATTERN_TRIPOD 0
#define PATTERN_WAVE   1
#define PATTERN_RIPPLE 2

#define DIRECTION_FORWARD 1
#define DIRECTION_STOP 0

void test_single_servo(int fd, uint8_t leg_id, uint8_t joint_id, int16_t angle)
{
    struct servo_control ctrl = {
        .leg_id = leg_id,
        .joint_id = joint_id,
        .angle = angle
    };

    if (ioctl(fd, IOCTL_SET_SERVO, &ctrl) < 0) {
        perror("Failed to control servo");
        return;
    }

    printf("Set leg %d joint %d to angle %d\n", leg_id, joint_id, angle);
}

void test_move_leg(int fd, uint8_t leg_id, int16_t hip, int16_t knee, int16_t ankle)
{
    // Test hip joint
    test_single_servo(fd, leg_id, 0, hip);
    usleep(500000);  // 500ms delay

    // Test knee joint
    test_single_servo(fd, leg_id, 1, knee);
    usleep(500000);

    // Test ankle joint
    test_single_servo(fd, leg_id, 2, ankle);
    usleep(500000);
}

void test_pattern(int fd, uint8_t pattern_id, uint8_t speed, int8_t direction)
{
    struct movement_pattern pattern = {
        .pattern_id = pattern_id,
        .speed = speed,
        .direction = direction
    };

    if (ioctl(fd, IOCTL_SET_PATTERN, &pattern) < 0) {
        perror("Failed to set movement pattern");
        return;
    }

    printf("Set pattern %d with speed %d and direction %d\n", 
           pattern_id, speed, direction);
}

void test_mpu6050(int fd)
{
    struct mpu6050_data data;
    
    if (ioctl(fd, IOCTL_GET_MPU6050, &data) < 0) {
        perror("Failed to read MPU6050");
        return;
    }
    
    printf("MPU6050 Data:\n");
    printf("Accel: X=%d Y=%d Z=%d\n", data.accel_x, data.accel_y, data.accel_z);
    printf("Gyro:  X=%d Y=%d Z=%d\n", data.gyro_x, data.gyro_y, data.gyro_z);
    printf("Temp:  %d\n", data.temp);
}

void test_uart(int fd)
{
    struct uart_message msg;
    const char *test_str = "Hello from user space!\n";
    
    strncpy((char *)msg.data, test_str, sizeof(msg.data));
    msg.len = strlen(test_str);
    
    if (ioctl(fd, IOCTL_UART_WRITE, &msg) < 0) {
        perror("Failed to write UART");
        return;
    }
    
    // Try to read response
    memset(&msg, 0, sizeof(msg));
    msg.len = sizeof(msg.data);
    
    if (ioctl(fd, IOCTL_UART_READ, &msg) < 0) {
        perror("Failed to read UART");
        return;
    }
    
    if (msg.len > 0) {
        printf("Received: %.*s\n", (int)msg.len, msg.data);
    } else {
        printf("No data received\n");
    }
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
                printf("Usage: %s [OPTIONS]\n", argv[0]);
                printf("Options:\n");
                printf("  -s <leg_id> -j <joint_id> -a <angle>  Set single servo\n");
                printf("  -l <leg_id> -c <coxa> -f <femur> -t <tibia>  Move entire leg\n");
                printf("  -p <pattern_id> -v <speed> -d <direction>  Run movement pattern\n");
                printf("  -m  Read MPU6050 sensor data\n");
                printf("  -w  Run wave pattern test\n");
                printf("  -r  Run tripod pattern test\n");
                printf("  -h  Show this help message\n");
                return 1;
        }
    }

    // Open device
    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
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
        printf("Running wave pattern test...\n");
        test_pattern(fd, PATTERN_WAVE, 5, DIRECTION_FORWARD);  // Speed 5, forward direction
        sleep(10);  // Run for 10 seconds
        test_pattern(fd, PATTERN_WAVE, 0, DIRECTION_STOP);  // Stop
    }
    else if (run_tripod) {
        printf("Running tripod pattern test...\n");
        test_pattern(fd, PATTERN_TRIPOD, 7, DIRECTION_FORWARD);  // Speed 7, forward direction
        sleep(10);  // Run for 10 seconds
        test_pattern(fd, PATTERN_TRIPOD, 0, DIRECTION_STOP);  // Stop
    }
    else {
        printf("Testing individual servo control...\n");
        // Test leg 0 movements
        test_move_leg(fd, 0, 0, 45, -45);  // Neutral, raised, extended
        
        printf("\nTesting movement patterns...\n");
        // Test tripod gait
        test_pattern(fd, PATTERN_TRIPOD, 5, DIRECTION_FORWARD);
        sleep(5);  // Let it walk for 5 seconds
        test_pattern(fd, PATTERN_TRIPOD, 0, DIRECTION_STOP);
        
        printf("\nTesting MPU6050 sensor...\n");
        test_mpu6050(fd);
        
        printf("\nTesting UART communication...\n");
        test_uart(fd);
    }

    close(fd);
    return 0;
}