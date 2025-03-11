#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include "hexapod.hpp"

// Global flag for program control
static volatile int running = 1;

// Signal handler for graceful termination
void handle_signal(int sig)
{
    printf("\nReceived signal %d, shutting down...\n", sig);
    running = 0;
}

int main(int argc, char *argv[])
{
    uint8_t leg_num = 0;
    double angle = 0;

    // Set up signal handler for clean termination
    signal(SIGINT, handle_signal);

    // Check command line args
    if (argc > 1)
    {
        leg_num = static_cast<uint8_t>(atoi(argv[1]));
        if (leg_num >= NUM_LEGS)
        {
            fprintf(stderr, "Leg number must be 0-%d\n", NUM_LEGS - 1);
            return 1;
        }
    }

    // Initialize hexapod
    Hexapod hexapod;
    if (!hexapod.init())
    {
        fprintf(stderr, "Failed to initialize hexapod: %s\n",
                hexapod.getLastErrorMessage().c_str());
        return 1;
    }

    printf("Servo Test - Leg %d - Press Ctrl+C to exit\n", leg_num);
    printf("==========================================\n");

    // Center all servos to start
    if (!hexapod.centerAll())
    {
        fprintf(stderr, "Failed to center servos: %s\n",
                hexapod.getLastErrorMessage().c_str());
        hexapod.cleanup();
        return 1;
    }

    // Allow servos to reach position
    sleep(1);

    // Main loop - sweep each joint back and forth
    while (running)
    {
        // Sweep hip
        printf("Sweeping hip joint...\n");
        for (int i = 0; i < 100 && running; i++)
        {
            angle = sin(static_cast<double>(i) * 0.063) * 45.0; // -45 to +45 degrees
            LegPosition pos(static_cast<int16_t>(angle), 0, 0);

            if (!hexapod.setLegPosition(leg_num, pos))
            {
                fprintf(stderr, "Failed to set position: %s\n",
                        hexapod.getLastErrorMessage().c_str());
                break;
            }

            usleep(30000); // 30ms
        }

        if (!running)
            break;

        // Center
        LegPosition centerHip(0, 0, 0);
        hexapod.setLegPosition(leg_num, centerHip);
        usleep(300000); // 300ms

        // Sweep knee
        printf("Sweeping knee joint...\n");
        for (int i = 0; i < 100 && running; i++)
        {
            angle = sin(static_cast<double>(i) * 0.063) * 45.0;
            LegPosition pos(0, static_cast<int16_t>(angle), 0);

            if (!hexapod.setLegPosition(leg_num, pos))
            {
                fprintf(stderr, "Failed to set position: %s\n",
                        hexapod.getLastErrorMessage().c_str());
                break;
            }

            usleep(30000); // 30ms
        }

        if (!running)
            break;

        // Center
        LegPosition centerKnee(0, 0, 0);
        hexapod.setLegPosition(leg_num, centerKnee);
        usleep(300000); // 300ms

        // Sweep ankle
        printf("Sweeping ankle joint...\n");
        for (int i = 0; i < 100 && running; i++)
        {
            angle = sin(static_cast<double>(i) * 0.063) * 45.0;
            LegPosition pos(0, 0, static_cast<int16_t>(angle));

            if (!hexapod.setLegPosition(leg_num, pos))
            {
                fprintf(stderr, "Failed to set position: %s\n",
                        hexapod.getLastErrorMessage().c_str());
                break;
            }

            usleep(30000); // 30ms
        }

        // Center
        LegPosition centerAnkle(0, 0, 0);
        hexapod.setLegPosition(leg_num, centerAnkle);
        usleep(300000); // 300ms
    }

    // Center all servos before exit
    hexapod.centerAll();

    // Cleanup happens automatically in Hexapod destructor
    return 0;
}
