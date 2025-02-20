#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <signal.h>
#include "../include/test_ioctl.h"

#define DEVICE_PATH "/dev/hexapod"

// Global flag for continuous reading
static volatile int keep_running = 1;

// Signal handler for Ctrl+C
static void sig_handler(int _) {
    (void)_;
    keep_running = 0;
}

int main() {
    int fd;
    
    // Set up signal handler
    signal(SIGINT, sig_handler);
    
    // Open the hexapod device
    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return -1;
    }
    
    printf("Reading MPU6050 sensor data...\n");
    printf("Press Ctrl+C to stop\n\n");
    
    while (keep_running) {
        // TODO: Add IOCTL commands for reading MPU6050 data
        // For now, just add a placeholder
        printf("MPU6050 data reading will be implemented here\n");
        sleep(1);
    }
    
    printf("\nStopping sensor reading...\n");
    close(fd);
    return 0;
}