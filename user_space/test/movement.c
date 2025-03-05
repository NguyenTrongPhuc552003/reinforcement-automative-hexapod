#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include "kinematics.h"
#include "hexapod.h"
#include "gait.h"

#define TOLERANCE 1.0

/* Global flag for controlled program termination */
static volatile bool running = true;

/* Signal handler for clean termination */
static void sigint_handler(int sig)
{
    printf("\nReceived signal %d, terminating...\n", sig);
    running = false;
}

// Forward kinematics test
static void test_forward_kinematics(void)
{
    leg_position_t angles = {
        .hip = 45.0,
        .knee = 90.0,
        .ankle = -30.0};
    point3d_t position;

    printf("Forward Kinematics Test:\n");
    printf("Input angles: (%.2d, %.2d, %.2d)\n",
           angles.hip, angles.knee, angles.ankle);

    int ret = forward_kinematics(&angles, &position);
    assert(ret == 0);

    printf("Output position: (%.2f, %.2f, %.2f)\n",
           position.x, position.y, position.z);
}

// Inverse kinematics test
static void test_inverse_kinematics(void)
{
    // Test positions that should be reachable
    point3d_t test_positions[] = {
        {90.0, 0.0, -70.0},    // Straight forward, mid height
        {70.0, 0.0, -60.0},    // Closer position
        {50.0, 50.0, -50.0},   // 45-degree angle
        {120.0, 0.0, -40.0},   // Extended reach
        {100.0, -50.0, -45.0}, // Side reach
        {80.0, 30.0, -80.0}    // Combined movement
    };

    printf("\nInverse Kinematics Tests:\n");

    for (size_t i = 0; i < sizeof(test_positions) / sizeof(test_positions[0]); i++)
    {
        point3d_t target = test_positions[i];
        leg_position_t angles;
        point3d_t verification;

        printf("\nTest %zu:\nTarget position: (%.2f, %.2f, %.2f)\n",
               i + 1, target.x, target.y, target.z);

        if (inverse_kinematics(&target, &angles) == 0)
        {
            printf("Calculated angles: hip=%.2d, knee=%.2d, ankle=%.2d\n",
                   angles.hip, angles.knee, angles.ankle);

            if (forward_kinematics(&angles, &verification) == 0)
            {
                double error = sqrt(pow(verification.x - target.x, 2) +
                                    pow(verification.y - target.y, 2) +
                                    pow(verification.z - target.z, 2));

                printf("Verification position: (%.2f, %.2f, %.2f)\n",
                       verification.x, verification.y, verification.z);
                printf("Position error: %.2f mm\n", error);

                if (error >= TOLERANCE)
                {
                    printf("Error: Position error %.2f mm exceeds tolerance %.2f mm\n", error, TOLERANCE);
                }

                assert(error < TOLERANCE);
            }
        }
    }
}

// Edge cases test
static void test_edge_cases(void)
{
    printf("\nTesting edge cases:\n");

    // Test unreachable positions
    point3d_t invalid_positions[] = {
        {200.0, 0.0, 0.0},     // Too far
        {20.0, 0.0, 0.0},      // Too close (inside coxa offset)
        {50.0, 50.0, 150.0},   // Too high
        {150.0, 150.0, -50.0}, // Too far diagonally
        {100.0, 0.0, -200.0}   // Too low
    };

    for (size_t i = 0; i < sizeof(invalid_positions) / sizeof(invalid_positions[0]); i++)
    {
        leg_position_t angles;
        printf("\nTesting position %zu: (%.1f, %.1f, %.1f)\n",
               i + 1,
               invalid_positions[i].x,
               invalid_positions[i].y,
               invalid_positions[i].z);

        int result = inverse_kinematics(&invalid_positions[i], &angles);
        printf("Result: %s\n", (result < 0) ? "PASS (unreachable)" : "FAIL (shouldn't be reachable)");
        assert(result < 0);
    }
}

// Calibration positions test
static void test_calibration_positions(void)
{
    printf("\nTesting calibration positions...\n");

    // Test positions based on actual leg dimensions
    point3d_t calibration_positions[] = {
        {100.0, 0.0, 0.0},   // Straight out
        {0.0, 100.0, 0.0},   // Side position
        {70.0, 0.0, -100.0}, // Down position
        {50.0, 50.0, -70.0}  // Combined movement
    };

    for (size_t i = 0; i < sizeof(calibration_positions) / sizeof(calibration_positions[0]); i++)
    {
        point3d_t target = calibration_positions[i];
        leg_position_t angles;

        printf("\nCalibration Test %zu:\n", i + 1);
        printf("Target: (%.1f, %.1f, %.1f)\n", target.x, target.y, target.z);

        if (inverse_kinematics(&target, &angles) == 0)
        {
            printf("Servo angles: hip=%.1d°, knee=%.1d°, ankle=%.1d°\n",
                   angles.hip, angles.knee, angles.ankle);

            // Physical verification prompt
            printf("Please verify physical position and press Enter...\n");
            getchar();
        }
        else
        {
            printf("Position unreachable with current dimensions\n");
        }
    }
}

/* New function to test different gait patterns */
static void test_gait_patterns(void)
{
    printf("\nTesting gait patterns...\n");

    // Set up signal handler for Ctrl+C
    signal(SIGINT, sigint_handler);

    // Parameters for testing gaits
    gait_params_t params[] = {
        {.type = GAIT_TRIPOD,
         .step_height = 30.0,
         .step_length = 50.0,
         .cycle_time = 2.0,
         .duty_factor = 0.5,
         .direction = 0.0, // forward
         .speed = 1.0},
        {.type = GAIT_WAVE,
         .step_height = 25.0,
         .step_length = 40.0,
         .cycle_time = 6.0,
         .duty_factor = 0.85,
         .direction = 0.0, // forward
         .speed = 0.8},
        {.type = GAIT_RIPPLE,
         .step_height = 20.0,
         .step_length = 45.0,
         .cycle_time = 3.0,
         .duty_factor = 0.75,
         .direction = 0.0, // forward
         .speed = 0.9}};

    // Test each gait pattern
    for (unsigned int i = 0; i < sizeof(params) / sizeof(params[0]) && running; i++)
    {
        printf("\nTesting %s gait...\n",
               params[i].type == GAIT_TRIPOD ? "TRIPOD" : params[i].type == GAIT_WAVE ? "WAVE"
                                                                                      : "RIPPLE");

        // Initialize gait controller
        if (gait_init(&params[i]) < 0)
        {
            printf("Failed to initialize gait controller\n");
            continue;
        }

        printf("Press Ctrl+C to stop this gait or continue to next pattern...\n");

        // Run the gait for specified time or until interrupted
        double start_time = hexapod_get_time();
        double test_duration = 15.0; // Test each gait for 15 seconds

        while (running && (hexapod_get_time() - start_time) < test_duration)
        {
            double elapsed = hexapod_get_time() - start_time;

            // Update hexapod state based on gait pattern
            hexapod_state_t state = {0};
            if (gait_update(elapsed, &state) < 0)
            {
                printf("Gait update failed\n");
                break;
            }

            // Sleep to maintain appropriate frame rate (20Hz)
            usleep(50000);
        }

        // If user interrupted with Ctrl+C, exit the loop
        if (!running)
            break;

        // Clean up gait controller
        gait_cleanup();

        // Return to neutral position
        hexapod_center_all_legs();
        sleep(2);
    }

    printf("Gait testing complete\n");
}

/* New function to test turning movements */
static void test_turning_movements(void)
{
    printf("\nTesting turning movements...\n");

    // Center all legs before starting
    hexapod_center_all_legs();
    sleep(1);

    // Initialize parameters for turning
    gait_params_t turn_params = {
        .type = GAIT_TRIPOD,
        .step_height = 25.0,
        .step_length = 40.0,
        .cycle_time = 2.0,
        .duty_factor = 0.5,
        .direction = 0.0,
        .speed = 1.0};

    // Test clockwise turn
    printf("Testing clockwise turn...\n");
    turn_params.direction = 90.0; // 90 degrees - clockwise
    gait_init(&turn_params);

    double start_time = hexapod_get_time();
    while (running && (hexapod_get_time() - start_time) < 5.0)
    {
        double elapsed = hexapod_get_time() - start_time;
        hexapod_state_t state = {0};
        gait_update(elapsed, &state);
        usleep(50000);
    }

    // Return to center before next test
    gait_cleanup();
    hexapod_center_all_legs();
    sleep(1);

    // Test counter-clockwise turn
    if (running)
    {
        printf("Testing counter-clockwise turn...\n");
        turn_params.direction = -90.0; // -90 degrees - counter-clockwise
        gait_init(&turn_params);

        start_time = hexapod_get_time();
        while (running && (hexapod_get_time() - start_time) < 5.0)
        {
            double elapsed = hexapod_get_time() - start_time;
            hexapod_state_t state = {0};
            gait_update(elapsed, &state);
            usleep(50000);
        }

        gait_cleanup();
        hexapod_center_all_legs();
    }

    printf("Turning tests complete\n");
}

int main(void)
{
    printf("Starting kinematics and gait tests...\n");

    if (hexapod_init() < 0)
    {
        printf("Failed to initialize hexapod\n");
        return 1;
    }

    // Set up signal handler
    signal(SIGINT, sigint_handler);

    // Basic kinematics tests
    test_forward_kinematics();
    test_inverse_kinematics();
    test_edge_cases();

    if (running)
    {
        // Physical verification tests (requires actual robot)
        char response;
        printf("\nDo you want to run physical tests? (y/n): ");
        if (scanf(" %c", &response) != 1)
        {
            // Handle input error
            printf("Input error. Skipping physical tests.\n");
            response = 'n';
        }
        if (response == 'y' || response == 'Y')
        {
            test_calibration_positions();
            test_gait_patterns();
            test_turning_movements();
        }
    }

    hexapod_cleanup();
    printf("Tests completed!\n");
    return 0;
}