#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include "../include/test_ioctl.h"

#define DEVICE_PATH "/dev/hexapod"

// Function prototypes
static int set_servo_angle(int fd, unsigned char leg_id, unsigned char joint_id, short angle);
static int move_leg(int fd, unsigned char leg_id, short coxa, short femur, short tibia);
static int reset_all_servos(int fd);
static void test_single_leg_movement(int fd);
static void test_wave_pattern(int fd);

int main() {
    int fd;
    
    // Open the hexapod device
    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return -1;
    }

    printf("Hexapod Movement Test Program\n");
    printf("1. Testing single leg movement\n");
    test_single_leg_movement(fd);
    
    printf("\n2. Testing wave pattern\n");
    test_wave_pattern(fd);
    
    // Reset all servos to neutral position
    printf("\nResetting all servos to neutral position...\n");
    reset_all_servos(fd);
    
    close(fd);
    return 0;
}

static int set_servo_angle(int fd, unsigned char leg_id, unsigned char joint_id, short angle) {
    struct servo_command cmd;
    
    if (angle < ANGLE_MIN || angle > ANGLE_MAX) {
        printf("Error: Angle %d out of range [%d, %d]\n", angle, ANGLE_MIN, ANGLE_MAX);
        return -1;
    }
    
    cmd.leg_id = leg_id;
    cmd.joint_id = joint_id;
    cmd.angle = angle;
    
    if (ioctl(fd, IOCTL_SET_SERVO_ANGLE, &cmd) < 0) {
        perror("IOCTL_SET_SERVO_ANGLE failed");
        return -1;
    }
    
    return 0;
}

static int move_leg(int fd, unsigned char leg_id, short coxa, short femur, short tibia) {
    struct leg_movement move;
    
    move.leg_id = leg_id;
    move.coxa_angle = coxa;
    move.femur_angle = femur;
    move.tibia_angle = tibia;
    
    if (ioctl(fd, IOCTL_MOVE_LEG, &move) < 0) {
        perror("IOCTL_MOVE_LEG failed");
        return -1;
    }
    
    return 0;
}

static int reset_all_servos(int fd) {
    if (ioctl(fd, IOCTL_RESET_ALL) < 0) {
        perror("IOCTL_RESET_ALL failed");
        return -1;
    }
    return 0;
}

static void test_single_leg_movement(int fd) {
    printf("Moving right front leg (leg_id: %d)\n", LEG_RIGHT_FRONT);
    
    // Test coxa joint
    printf("Testing coxa joint...\n");
    set_servo_angle(fd, LEG_RIGHT_FRONT, JOINT_COXA, 45);
    usleep(500000);
    set_servo_angle(fd, LEG_RIGHT_FRONT, JOINT_COXA, -45);
    usleep(500000);
    set_servo_angle(fd, LEG_RIGHT_FRONT, JOINT_COXA, 0);
    usleep(500000);
    
    // Test femur joint
    printf("Testing femur joint...\n");
    set_servo_angle(fd, LEG_RIGHT_FRONT, JOINT_FEMUR, 45);
    usleep(500000);
    set_servo_angle(fd, LEG_RIGHT_FRONT, JOINT_FEMUR, -45);
    usleep(500000);
    set_servo_angle(fd, LEG_RIGHT_FRONT, JOINT_FEMUR, 0);
    usleep(500000);
    
    // Test tibia joint
    printf("Testing tibia joint...\n");
    set_servo_angle(fd, LEG_RIGHT_FRONT, JOINT_TIBIA, 45);
    usleep(500000);
    set_servo_angle(fd, LEG_RIGHT_FRONT, JOINT_TIBIA, -45);
    usleep(500000);
    set_servo_angle(fd, LEG_RIGHT_FRONT, JOINT_TIBIA, 0);
    usleep(500000);
}

static void test_wave_pattern(int fd) {
    int i;
    const short wave_positions[][3] = {
        {30, 0, 0},    // Coxa out
        {30, -30, 45}, // Lift leg
        {-30, -30, 45}, // Move forward
        {-30, 0, 0},   // Lower leg
        {0, 0, 0}      // Return to neutral
    };
    
    printf("Performing wave pattern with all legs...\n");
    
    // Move each leg in sequence
    for (i = 0; i < NUM_LEGS; i++) {
        printf("Moving leg %d\n", i);
        for (int pos = 0; pos < 5; pos++) {
            move_leg(fd, i,
                    wave_positions[pos][0],
                    wave_positions[pos][1],
                    wave_positions[pos][2]);
            usleep(300000); // 300ms delay between movements
        }
    }
}