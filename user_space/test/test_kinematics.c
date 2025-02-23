#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "kinematics.h"
#include "hexapod.h"

#define TOLERANCE 1.0

static void test_forward_kinematics(void)
{
    leg_angles_t angles = {
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
        {90.0, 0.0, -70.0},  // Straight forward, mid height
        {70.0, 0.0, -60.0},  // Closer position
        {50.0, 50.0, -50.0}, // 45-degree angle
        {120.0, 0.0, -40.0}  // Extended reach
    };

    printf("\nInverse Kinematics Tests:\n");

    for (size_t i = 0; i < sizeof(test_positions) / sizeof(test_positions[0]); i++)
    {
        point3d_t target = test_positions[i];
        leg_angles_t angles;
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

                assert(error < TOLERANCE);
            }
        }
    }
}

int main(void)
{
    printf("Starting kinematics tests...\n");

    test_forward_kinematics();
    test_inverse_kinematics();

    printf("All kinematics tests passed!\n");
    return 0;
}