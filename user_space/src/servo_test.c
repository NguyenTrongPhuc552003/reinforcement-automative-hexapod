#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>

// IOCTL commands (must match kernel module)
#define HEXAPOD_IOC_MAGIC 'H'
#define IOCTL_SET_SERVO _IOW(HEXAPOD_IOC_MAGIC, 2, struct servo_control)

// Servo control structure (must match kernel module)
struct servo_control {
    unsigned int channel;
    unsigned long duty_ns;
};

// Servo configuration
#define NUM_SERVOS 18
#define PCA9685_1_ADDR 0x40
#define PCA9685_2_ADDR 0x41

void test_single_servo(int fd, int servo_id, int angle) {
    struct servo_control ctrl;
    ctrl.channel = servo_id;
    ctrl.duty_ns = angle;  // We'll use angle directly in the kernel module

    if (ioctl(fd, IOCTL_SET_SERVO, &ctrl) < 0) {
        printf("Error setting servo %d to angle %d: %s\n", 
               servo_id, angle, strerror(errno));
    } else {
        printf("Set servo %d to angle %d\n", servo_id, angle);
    }
}

void test_wave_pattern(int fd) {
    printf("Running wave pattern test...\n");
    
    for (int angle = 0; angle <= 180; angle += 45) {
        printf("\nMoving all servos to %d degrees\n", angle);
        for (int servo = 0; servo < NUM_SERVOS; servo++) {
            test_single_servo(fd, servo, angle);
            usleep(100000);  // 100ms delay between servos
        }
        sleep(1);  // Wait 1 second between angles
    }
}

void test_sequential_movement(int fd) {
    printf("Running sequential movement test...\n");
    
    for (int servo = 0; servo < NUM_SERVOS; servo++) {
        printf("\nTesting servo %d\n", servo);
        
        // Move to 0 degrees
        test_single_servo(fd, servo, 0);
        sleep(1);
        
        // Move to 90 degrees
        test_single_servo(fd, servo, 90);
        sleep(1);
        
        // Move to 180 degrees
        test_single_servo(fd, servo, 180);
        sleep(1);
        
        // Return to neutral position (90 degrees)
        test_single_servo(fd, servo, 90);
        sleep(1);
    }
}

void test_leg_movement(int fd) {
    printf("Running leg movement test...\n");
    
    // Assuming 3 servos per leg (6 legs total)
    for (int leg = 0; leg < 6; leg++) {
        int base_servo = leg * 3;
        printf("\nTesting leg %d (servos %d, %d, %d)\n", 
               leg, base_servo, base_servo + 1, base_servo + 2);
        
        // Move all servos in the leg
        for (int i = 0; i < 3; i++) {
            test_single_servo(fd, base_servo + i, 45);
        }
        sleep(1);
        
        for (int i = 0; i < 3; i++) {
            test_single_servo(fd, base_servo + i, 135);
        }
        sleep(1);
        
        // Return to neutral
        for (int i = 0; i < 3; i++) {
            test_single_servo(fd, base_servo + i, 90);
        }
        sleep(1);
    }
}

void print_usage(const char *program_name) {
    printf("Usage: %s <test_type>\n", program_name);
    printf("Test types:\n");
    printf("  1 - Wave pattern (all servos)\n");
    printf("  2 - Sequential movement (test each servo)\n");
    printf("  3 - Leg movement test\n");
    printf("  4 - Center all servos\n");
    printf("  <servo_id> <angle> - Move specific servo to angle\n");
}

int main(int argc, char *argv[]) {
    int fd = open("/dev/hexapod", O_RDWR);
    if (fd < 0) {
        printf("Error opening device: %s\n", strerror(errno));
        return 1;
    }

    if (argc < 2) {
        print_usage(argv[0]);
        close(fd);
        return 1;
    }

    if (argc == 2) {
        int test_type = atoi(argv[1]);
        switch (test_type) {
            case 1:
                test_wave_pattern(fd);
                break;
            case 2:
                test_sequential_movement(fd);
                break;
            case 3:
                test_leg_movement(fd);
                break;
            case 4:
                printf("Centering all servos...\n");
                for (int i = 0; i < NUM_SERVOS; i++) {
                    test_single_servo(fd, i, 90);
                    usleep(100000);
                }
                break;
            default:
                print_usage(argv[0]);
                break;
        }
    } else if (argc == 3) {
        int servo_id = atoi(argv[1]);
        int angle = atoi(argv[2]);
        
        if (servo_id >= 0 && servo_id < NUM_SERVOS && 
            angle >= 0 && angle <= 180) {
            test_single_servo(fd, servo_id, angle);
        } else {
            printf("Invalid servo_id or angle!\n");
            printf("servo_id must be 0-%d\n", NUM_SERVOS - 1);
            printf("angle must be 0-180\n");
        }
    }

    close(fd);
    return 0;
} 