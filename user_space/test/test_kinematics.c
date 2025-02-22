#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "kinematics.h"
#include "hexapod.h"

static void test_forward_kinematics(void)
{
    leg_position_t angles = {45.0, 90.0, -30.0};
    point3d_t position;
    
    int ret = forward_kinematics(&angles, &position);
    assert(ret == 0);
    
    // Add specific test cases based on your robot's geometry
    printf("Forward Kinematics Test:\n");
    printf("Input angles: (%.2f, %.2f, %.2f)\n", angles.hip, angles.knee, angles.ankle);
    printf("Output position: (%.2f, %.2f, %.2f)\n", position.x, position.y, position.z);
}

static void test_inverse_kinematics(void)
{
    point3d_t target = {100.0, 0.0, -50.0};
    leg_position_t angles;
    
    int ret = inverse_kinematics(&target, &angles);
    assert(ret == 0);
    
    printf("Inverse Kinematics Test:\n");
    printf("Input position: (%.2f, %.2f, %.2f)\n", target.x, target.y, target.z);
    printf("Output angles: (%.2f, %.2f, %.2f)\n", angles.hip, angles.knee, angles.ankle);
    
    // Verify by running forward kinematics
    point3d_t verification;
    ret = forward_kinematics(&angles, &verification);
    assert(ret == 0);
    
    // Check if we got back to the original position (within tolerance)
    const double tolerance = 1.0;
    assert(fabs(verification.x - target.x) < tolerance);
    assert(fabs(verification.y - target.y) < tolerance);
    assert(fabs(verification.z - target.z) < tolerance);
}

int main(void)
{
    printf("Starting kinematics tests...\n");
    
    test_forward_kinematics();
    test_inverse_kinematics();
    
    printf("All kinematics tests passed!\n");
    return 0;
}