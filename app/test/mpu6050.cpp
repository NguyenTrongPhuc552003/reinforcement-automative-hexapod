#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include "hexapod.hpp"

// Global flag for termination
static volatile int running = 1;

// Signal handler
static void test_mpu6050_signal_handler(int sig)
{
    printf("\nReceived signal %d, shutting down...\n", sig);
    running = 0;
}

int main(void)
{
    // Set up signal handler for clean termination
    signal(SIGINT, test_mpu6050_signal_handler);

    // Initialize hexapod
    Hexapod hexapod;
    if (!hexapod.init())
    {
        fprintf(stderr, "Failed to initialize hexapod: %s\n",
                hexapod.getLastErrorMessage().c_str());
        return 1;
    }

    printf("IMU Test - Press Ctrl+C to exit\n");
    printf("================================\n");

    // Main loop
    while (running)
    {
        ImuData imuData;

        // Get IMU data
        if (!hexapod.getImuData(imuData))
        {
            fprintf(stderr, "Failed to read IMU: %s\n",
                    hexapod.getLastErrorMessage().c_str());
            break;
        }

        // Print formatted IMU data
        hexapod.printImuData(imuData);
        fflush(stdout);

        // Short delay
        usleep(100000); // 100ms
    }

    printf("\n\nExiting...\n");

    // Cleanup happens automatically in Hexapod destructor
    return 0;
}
