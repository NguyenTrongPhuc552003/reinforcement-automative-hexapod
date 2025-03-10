#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include "gait.hpp"

// Signal flag for graceful termination
static volatile int running = 1;

// Signal handler
static void handle_signal(int sig)
{
    printf("\nReceived signal %d, shutting down...\n", sig);
    running = 0;
}

// Get current time in seconds
static double get_time(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + (ts.tv_nsec / 1.0e9);
}

// Function to test leg movement
static bool test_leg_movement(Hexapod &hexapod)
{
    printf("Testing individual leg movement...\n");

    // Center all legs first
    printf("Centering all legs...\n");
    hexapod.centerAll();
    sleep(2);

    // Test each leg
    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        printf("Testing leg %d\n", leg);

        // Move hip joint
        printf("  Moving hip joint\n");
        for (int angle = -30; angle <= 30 && running; angle += 10)
        {
            LegPosition pos(angle, 0, 0);

            if (!hexapod.setLegPosition(leg, pos))
            {
                fprintf(stderr, "Failed to move hip: %s\n",
                        hexapod.getLastErrorMessage().c_str());
                return false;
            }
            usleep(200000); // 200ms delay
        }

        // Reset hip
        LegPosition centerHip(0, 0, 0);
        hexapod.setLegPosition(leg, centerHip);
        usleep(500000); // 500ms delay

        // Move knee joint
        printf("  Moving knee joint\n");
        for (int angle = 0; angle <= 45 && running; angle += 10)
        {
            LegPosition pos(0, angle, 0);

            if (!hexapod.setLegPosition(leg, pos))
            {
                fprintf(stderr, "Failed to move knee: %s\n",
                        hexapod.getLastErrorMessage().c_str());
                return false;
            }
            usleep(200000); // 200ms delay
        }

        // Reset knee
        LegPosition centerKnee(0, 0, 0);
        hexapod.setLegPosition(leg, centerKnee);
        usleep(500000); // 500ms delay

        // Move ankle joint
        printf("  Moving ankle joint\n");
        for (int angle = -30; angle <= 30 && running; angle += 10)
        {
            LegPosition pos(0, 0, angle);

            if (!hexapod.setLegPosition(leg, pos))
            {
                fprintf(stderr, "Failed to move ankle: %s\n",
                        hexapod.getLastErrorMessage().c_str());
                return false;
            }
            usleep(200000); // 200ms delay
        }

        // Reset ankle
        LegPosition centerAnkle(0, 0, 0);
        hexapod.setLegPosition(leg, centerAnkle);
        usleep(500000); // 500ms delay
    }

    // Center all legs again
    printf("Centering all legs...\n");
    hexapod.centerAll();

    return true;
}

// Test gait patterns
static bool test_gait(Hexapod &hexapod, GaitType gaitType)
{
    Gait gait;
    GaitParameters params;
    double start_time, current_time, elapsed_time;
    double direction = 0.0; // Forward
    double speed = 0.5;     // Half-speed

    // Initialize gait parameters
    params.type = gaitType;
    params.stepHeight = 30.0; // 30mm step height
    params.stepLength = 60.0; // 60mm stride length
    params.cycleTime = 2.0;   // 2 seconds per cycle

    switch (gaitType)
    {
    case GaitType::TRIPOD:
        printf("Testing tripod gait pattern\n");
        params.dutyFactor = 0.5; // 50% stance phase
        break;

    case GaitType::WAVE:
        printf("Testing wave gait pattern\n");
        params.dutyFactor = 0.75; // 75% stance phase
        break;

    case GaitType::RIPPLE:
        printf("Testing ripple gait pattern\n");
        params.dutyFactor = 0.65; // 65% stance phase
        break;

    default:
        fprintf(stderr, "Invalid gait type\n");
        return false;
    }

    // Initialize gait controller
    if (!gait.init(hexapod, params))
    {
        fprintf(stderr, "Failed to initialize gait\n");
        return false;
    }

    // Center legs before starting
    gait.centerLegs();
    sleep(1);

    // Run gait pattern for 15 seconds or until Ctrl+C
    printf("Running gait pattern for 15 seconds (press Ctrl+C to stop)...\n");
    start_time = get_time();
    elapsed_time = 0.0;

    while (elapsed_time < 15.0 && running)
    {
        current_time = get_time();
        elapsed_time = current_time - start_time;

        // Update direction to create circular movement
        direction = fmod(elapsed_time * 24.0, 360.0);

        // Update gait
        if (!gait.update(current_time, direction, speed))
        {
            fprintf(stderr, "Gait update error\n");
            break;
        }

        // Print status every second
        if ((int)elapsed_time != (int)(elapsed_time - 0.1))
        {
            printf("Time: %.1f s, Direction: %.1f°, Speed: %.1f\n",
                   elapsed_time, direction, speed);
        }

        usleep(20000); // 20ms (50Hz update rate)
    }

    // Clean up - gait will clean itself up in destructor

    // Center legs again
    printf("Centering legs...\n");
    gait.centerLegs();

    return true;
}

// Main test program
int main(int argc, char *argv[])
{
    // Set up signal handler
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    printf("Hexapod Movement Test Program\n");
    printf("----------------------------\n");

    // Initialize hexapod
    Hexapod hexapod;
    if (!hexapod.init())
    {
        fprintf(stderr, "Failed to initialize hexapod: %s\n",
                hexapod.getLastErrorMessage().c_str());
        return 1;
    }

    printf("Connected to hexapod device\n\n");

    // Run tests (or specific test if specified)
    if (argc > 1)
    {
        if (strcmp(argv[1], "leg") == 0)
        {
            test_leg_movement(hexapod);
        }
        else if (strcmp(argv[1], "tripod") == 0)
        {
            test_gait(hexapod, GaitType::TRIPOD);
        }
        else if (strcmp(argv[1], "wave") == 0)
        {
            test_gait(hexapod, GaitType::WAVE);
        }
        else if (strcmp(argv[1], "ripple") == 0)
        {
            test_gait(hexapod, GaitType::RIPPLE);
        }
        else
        {
            printf("Unknown test: %s\n", argv[1]);
            printf("Available tests: leg, tripod, wave, ripple\n");
        }
    }
    else
    {
        // Run all tests
        printf("Running all tests. Press Ctrl+C to skip to next test.\n\n");
        printf("\n=== Testing Individual Leg Movement ===\n");
        test_leg_movement(hexapod);
        if (!running)
            goto cleanup;
        sleep(1);

        printf("\n=== Testing Tripod Gait ===\n");
        test_gait(hexapod, GaitType::TRIPOD);
        if (!running)
            goto cleanup;
        sleep(1);

        printf("\n=== Testing Wave Gait ===\n");
        test_gait(hexapod, GaitType::WAVE);
        if (!running)
            goto cleanup;
        sleep(1);

        printf("\n=== Testing Ripple Gait ===\n");
        test_gait(hexapod, GaitType::RIPPLE);
    }

cleanup:
    // Center all legs before exit
    printf("\nCentering all legs...\n");
    hexapod.centerAll();

    printf("Test program completed.\n");
    return 0;
}