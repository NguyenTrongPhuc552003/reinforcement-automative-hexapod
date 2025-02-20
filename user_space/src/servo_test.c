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
static void sweep_test(int fd, unsigned char leg_id, unsigned char joint_id);

int main(int argc, char *argv[]) {
    int fd;
    unsigned char leg_id = 0;
    unsigned char joint_id = 0;
    
    if (argc != 3) {
        printf("Usage: %s <leg_id> <joint_id>\n", argv[0]);
        printf("leg_id: 0-5 (RF=0, RM=1, RB=2, LF=3, LM=4, LB=5)\n");
        printf("joint_id: 0-2 (Coxa=0, Femur=1, Tibia=2)\n");
        return -1;
    }
    
    leg_id = (unsigned char)atoi(argv[1]);
    joint_id = (unsigned char)atoi(argv[2]);
    
    if (leg_id >= NUM_LEGS || joint_id >= NUM_JOINTS_PER_LEG) {
        printf("Invalid leg_id or joint_id\n");
        return -1;
    }
    
    // Open the hexapod device
    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return -1;
    }
    
    printf("Testing servo: Leg %d, Joint %d\n", leg_id, joint_id);
    sweep_test(fd, leg_id, joint_id);
    
    // Reset servo to neutral position
    set_servo_angle(fd, leg_id, joint_id, 0);
    
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

static void sweep_test(int fd, unsigned char leg_id, unsigned char joint_id) {
    short angle;
    
    // Sweep from -45 to +45 degrees
    printf("Sweeping from -45 to +45 degrees...\n");
    for (angle = -45; angle <= 45; angle += 5) {
        if (set_servo_angle(fd, leg_id, joint_id, angle) == 0) {
            printf("Angle: %d\n", angle);
            usleep(100000); // 100ms delay
        }
    }
    
    // Sweep back from +45 to -45 degrees
    printf("Sweeping from +45 to -45 degrees...\n");
    for (angle = 45; angle >= -45; angle -= 5) {
        if (set_servo_angle(fd, leg_id, joint_id, angle) == 0) {
            printf("Angle: %d\n", angle);
            usleep(100000); // 100ms delay
        }
    }
}