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
    bool success = true;

    // Initialize gait parameters with LARGER values for more noticeable movement
    params.type = gaitType;
    params.stepHeight = 40.0; // Increased from 30.0 to 40.0 mm
    params.stepLength = 80.0; // Increased from 60.0 to 80.0 mm
    params.cycleTime = 3.0;   // Increased from 2.0 to 3.0 seconds for slower, more visible movement

    // Set duty factor according to gait type
    switch (gaitType)
    {
    case GaitType::TRIPOD:
        printf("Testing tripod gait pattern\n");
        params.dutyFactor = 0.5; // 50% stance phase
        break;

    case GaitType::WAVE:
        printf("Testing wave gait pattern\n");
        params.dutyFactor = 0.8; // 80% stance phase
        break;

    case GaitType::RIPPLE:
        printf("Testing ripple gait pattern\n");
        params.dutyFactor = 0.65; // 65% stance phase
        break;

    default:
        fprintf(stderr, "Invalid gait type\n");
        return false;
    }

    printf("Initializing gait with parameters:\n");
    printf("- Step Height: %.1f mm\n", params.stepHeight);
    printf("- Step Length: %.1f mm\n", params.stepLength);
    printf("- Cycle Time: %.1f seconds\n", params.cycleTime);
    printf("- Duty Factor: %.2f\n", params.dutyFactor);

    // Initialize gait controller
    if (!gait.init(hexapod, params))
    {
        fprintf(stderr, "Failed to initialize gait\n");
        return false;
    }

    // Center legs before starting
    printf("Centering legs...\n");
    if (!gait.centerLegs())
    {
        fprintf(stderr, "Failed to center legs\n");
        return false;
    }
    sleep(1);

    // Run gait pattern for 15 seconds or until Ctrl+C
    printf("Running gait pattern for 15 seconds (press Ctrl+C to stop)...\n");
    start_time = get_time();
    elapsed_time = 0.0;

    printf("Starting movement loop...\n");

    // Use fixed direction initially to troubleshoot
    direction = 0.0; // straight forward

    // Increase speed for more visible movement
    speed = 0.8;
    printf("Using fixed direction %.1f° and speed %.1f\n", direction, speed);

    while (elapsed_time < 15.0 && running)
    {
        current_time = get_time();
        elapsed_time = current_time - start_time;

        // Update gait
        if (!gait.update(elapsed_time, direction, speed))
        {
            fprintf(stderr, "Gait update error at time %.2f\n", elapsed_time);
            success = false;
            break;
        }

        // Print status every second
        if ((int)elapsed_time != (int)(elapsed_time - 0.1))
        {
            printf("Time: %.1f s, Direction: %.1f°, Speed: %.1f\n",
                   elapsed_time, direction, speed);

            // Every 5 seconds, switch direction to confirm servos respond
            if ((int)elapsed_time % 5 == 0)
            {
                direction = ((int)elapsed_time % 10 == 0) ? 0.0 : 180.0; // alternate forward/backward
                printf("Switching direction to %.1f°\n", direction);
            }
        }

        // Reduce delay for smoother motion - 10ms (100Hz update rate)
        usleep(10000);
    }

    // Clean up - gait will clean itself up in destructor

    // Center legs again
    printf("Centering legs...\n");
    gait.centerLegs();

    return success;
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

    // Test a simple leg movement to verify servo communication
    printf("Testing basic leg movement...\n");
    LegPosition testPos(15, 15, 15); // small movement to verify servos respond
    if (hexapod.setLegPosition(0, testPos))
    {
        printf("Servo test command sent successfully\n");
    }
    else
    {
        printf("Warning: Servo test command failed: %s\n",
               hexapod.getLastErrorMessage().c_str());
    }
    sleep(1);
    hexapod.centerAll();
    sleep(1);

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