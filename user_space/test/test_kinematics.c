#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "kinematics.h"
#include "hexapod.h"

#define TOLERANCE 1.0

static void test_forward_kinematics(void)
{
    leg_position_t angles = {
        .hip = 45.0,
        .knee = 90.0,
        .ankle = -30.0};
    point3d_t position;

    printf("Forward Kinematics Test:\n");
    printf("Input angles: (%.2f, %.2f, %.2f)\n",
           angles.hip, angles.knee, angles.ankle);

    int ret = forward_kinematics(&angles, &position);
    assert(ret == 0);

    printf("Output position: (%.2f, %.2f, %.2f)\n",
           position.x, position.y, position.z);
}

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
            printf("Calculated angles: hip=%.2f, knee=%.2f, ankle=%.2f\n",
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

// Add new function to test edge cases
static void test_edge_cases(void)
{
    printf("\nTesting edge cases:\n");

    // Test unreachable positions
    point3d_t invalid_positions[] = {
        {200.0, 0.0, 0.0}, // Too far
        {0.0, 0.0, 0.0},   // Too close
        {50.0, 50.0, 50.0} // Too high
    };

    for (size_t i = 0; i < sizeof(invalid_positions) / sizeof(invalid_positions[0]); i++)
    {
        leg_position_t angles;
        int result = inverse_kinematics(&invalid_positions[i], &angles);
        printf("Testing invalid position (%.1f, %.1f, %.1f) - Expected failure: %s\n",
               invalid_positions[i].x, invalid_positions[i].y, invalid_positions[i].z,
               (result < 0) ? "PASS" : "FAIL");
        assert(result < 0);
    }
}

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
            printf("Servo angles: hip=%.1f°, knee=%.1f°, ankle=%.1f°\n",
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

int main(void)
{
    printf("Starting kinematics tests...\n");

    assert(hexapod_init() == 0);

    test_forward_kinematics();
    test_inverse_kinematics();
    test_edge_cases();
    test_calibration_positions(); // Add this line

    hexapod_cleanup();
    printf("All kinematics tests passed!\n");
    return 0;
}