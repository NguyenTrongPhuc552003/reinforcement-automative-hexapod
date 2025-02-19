// servo_test.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <unistd.h>
#include "hexapod_ioctl.h"

#define DEVICE_PATH "/dev/hexapod"

static int g_fd = -1;
static volatile int g_running = 1;

// Core servo testing functions
void test_single_servo(void);
void test_leg_movement(void);
void test_all_legs(void);

// Menu options
enum {
    OPTION_EXIT = 0,
    OPTION_TEST_SINGLE_SERVO,
    OPTION_TEST_LEG_MOVEMENT,
    OPTION_TEST_ALL_LEGS
};
// Signal handler for clean exit
static void signal_handler(int signo) {
    g_running = 0;
}

// Initialize device and signal handling
static int initialize(void) {
    g_fd = open(DEVICE_PATH, O_RDWR);
    if (g_fd < 0) {
        perror("Failed to open device");
        return -1;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    return 0;
}

// Clean up resources
static void cleanup(void) {
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
}

// Display menu and get user choice
static int display_menu(void) {
    printf("\nServo Test Menu:\n");
    printf("1. Test Single Servo\n");
    printf("2. Test Leg Movement\n");
    printf("3. Test All Legs\n");
    printf("0. Exit\n");
    printf("Enter choice: ");

    int choice;
    if (scanf("%d", &choice) != 1) {
        while (getchar() != '\n'); // Clear input buffer
        return -1;
    }
    return choice;
}

void test_single_servo(void) {
    struct servo_data data;
    printf("Enter servo number (0-17): ");
    if (scanf("%d", &data.servo_num) != 1 || data.servo_num < 0 || data.servo_num > 17) {
        printf("Invalid servo number\n");
        while (getchar() != '\n');
        return;
    }

    printf("Enter angle (0-180): ");
    if (scanf("%d", &data.angle) != 1 || data.angle < 0 || data.angle > 180) {
        printf("Invalid angle\n");
        while (getchar() != '\n');
        return;
    }

    if (ioctl(g_fd, SERVO_SET_ANGLE, &data) < 0) {
        perror("Failed to set servo angle");
    } else {
        printf("Servo %d set to %d degrees\n", data.servo_num, data.angle);
    }
}

void test_leg_movement(void) {
    int leg_num;
    printf("Enter leg number (0-5): ");
    if (scanf("%d", &leg_num) != 1 || leg_num < 0 || leg_num > 5) {
        printf("Invalid leg number\n");
        while (getchar() != '\n');
        return;
    }

    struct leg_data data = {
        .leg_num = leg_num,
        .hip_angle = 90,
        .knee_angle = 90,
        .ankle_angle = 90
    };

    printf("Testing leg %d...\n", leg_num);
    printf("Moving to neutral position...\n");
    if (ioctl(g_fd, LEG_SET_POSITION, &data) < 0) {
        perror("Failed to set leg position");
        return;
    }

    sleep(1);

    // Test range of motion
    const int test_angles[] = {45, 135, 90};
    const char *joint_names[] = {"hip", "knee", "ankle"};
    
    for (int i = 0; i < 3; i++) {
        for (int angle : test_angles) {
            switch (i) {
                case 0: data.hip_angle = angle; break;
                case 1: data.knee_angle = angle; break;
                case 2: data.ankle_angle = angle; break;
            }
            
            printf("Setting %s to %d degrees...\n", joint_names[i], angle);
            if (ioctl(g_fd, LEG_SET_POSITION, &data) < 0) {
                perror("Failed to set leg position");
                return;
            }
            sleep(1);
        }
    }
}

void test_all_legs(void) {
    printf("Testing all legs...\n");
    
    // Set all legs to neutral position
    for (int leg = 0; leg < 6; leg++) {
        struct leg_data data = {
            .leg_num = leg,
            .hip_angle = 90,
            .knee_angle = 90,
            .ankle_angle = 90
        };
        
        if (ioctl(g_fd, LEG_SET_POSITION, &data) < 0) {
            perror("Failed to set leg position");
            return;
        }
    }
    
    sleep(2);
    
    // Test simple movement for each leg
    for (int leg = 0; leg < 6 && g_running; leg++) {
        printf("Testing leg %d\n", leg);
        struct leg_data data = {
            .leg_num = leg,
            .hip_angle = 45,
            .knee_angle = 45,
            .ankle_angle = 45
        };
        
        if (ioctl(g_fd, LEG_SET_POSITION, &data) < 0) {
            perror("Failed to set leg position");
            return;
        }
        
        sleep(1);
        
        // Return to neutral
        data.hip_angle = 90;
        data.knee_angle = 90;
        data.ankle_angle = 90;
        
        if (ioctl(g_fd, LEG_SET_POSITION, &data) < 0) {
            perror("Failed to set leg position");
            return;
        }
        
        sleep(1);
    }
}

int main(void) {
    if (initialize() < 0) {
        return 1;
    }

    printf("Servo Test Program\n");
    printf("Press Ctrl+C to exit\n");

    while (g_running) {
        int choice = display_menu();
        
        switch (choice) {
            case OPTION_TEST_SINGLE_SERVO:
                test_single_servo();
                break;
            case OPTION_TEST_LEG_MOVEMENT:
                test_leg_movement();
                break;
            case OPTION_TEST_ALL_LEGS:
                test_all_legs();
                break;
            case OPTION_EXIT:
                g_running = 0;
                break;
            default:
                printf("Invalid choice\n");
                break;
        }
    }

    cleanup();
    printf("\nExiting servo test program\n");
    return 0;
}