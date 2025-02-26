#include <math.h>
#include <stdio.h>
#include "kinematics.h"

/* Robot dimensions (in mm) - Actual measurements */
#define COXA_LENGTH 30.0
#define FEMUR_LENGTH 85.0
#define TIBIA_LENGTH 123.0
#define COXA_OFFSET COXA_LENGTH

static double degrees_to_radians(double degrees)
{
    return degrees * M_PI / 180.0;
}

static double radians_to_degrees(double radians)
{
    return radians * 180.0 / M_PI;
}

int forward_kinematics(const leg_position_t *angles, point3d_t *point)
{
    if (!angles || !point)
        return -1;

    double hip_rad = degrees_to_radians(angles->hip);
    double knee_rad = degrees_to_radians(angles->knee);
    double ankle_rad = degrees_to_radians(angles->ankle);

    // First, calculate position from coxa joint
    double coxa_x = COXA_LENGTH * cos(hip_rad);
    double coxa_y = COXA_LENGTH * sin(hip_rad);

    // Calculate femur endpoint
    double femur_proj = FEMUR_LENGTH * cos(knee_rad);
    double femur_x = coxa_x + femur_proj * cos(hip_rad);
    double femur_y = coxa_y + femur_proj * sin(hip_rad);
    double femur_z = -FEMUR_LENGTH * sin(knee_rad);

    // Calculate tibia endpoint
    double tibia_angle = knee_rad + ankle_rad;
    double tibia_proj = TIBIA_LENGTH * cos(tibia_angle);
    point->x = femur_x + tibia_proj * cos(hip_rad);
    point->y = femur_y + tibia_proj * sin(hip_rad);
    point->z = femur_z - TIBIA_LENGTH * sin(tibia_angle);

    return 0;
}

int inverse_kinematics(const point3d_t *point, leg_position_t *angles)
{
    if (!point || !angles)
        return -1;

    // Calculate hip angle (rotation in XY plane)
    double total_xy = sqrt(point->x * point->x + point->y * point->y);

    // Stricter XY plane validation
    if (total_xy > (FEMUR_LENGTH + TIBIA_LENGTH) * 0.9 || // 90% of max reach
        total_xy < COXA_OFFSET * 1.2)                     // 120% of min reach
    {
        printf("Point unreachable in XY plane. Distance: %.2f mm\n", total_xy);
        return -1;
    }

    double hip_rad = atan2(point->y, point->x);

    // Adjust target point to leg's local coordinate system
    double local_x = total_xy - COXA_OFFSET;
    double local_z = -point->z;

    // Calculate length from femur-tibia joint to end point
    double L = sqrt(local_x * local_x + local_z * local_z);

    // Stricter reach validation with more conservative limits
    double max_reach = (FEMUR_LENGTH + TIBIA_LENGTH) * 0.85;    // 85% of theoretical max
    double min_reach = fabs(FEMUR_LENGTH - TIBIA_LENGTH) * 1.2; // 120% of theoretical min

    if (L > max_reach || L < min_reach)
    {
        printf("Point unreachable. Required leg length %.2f mm outside valid range [%.2f, %.2f]\n",
               L, min_reach, max_reach);
        return -1;
    }

    // Z-axis validation (height limits)
    if (fabs(point->z) > (FEMUR_LENGTH + TIBIA_LENGTH) * 0.6) // 60% of max height
    {
        printf("Z coordinate %.2f mm exceeds safe working height\n", point->z);
        return -1;
    }

    // Calculate knee angle using law of cosines
    double knee_angle = acos((FEMUR_LENGTH * FEMUR_LENGTH +
                              TIBIA_LENGTH * TIBIA_LENGTH -
                              L * L) /
                             (2 * FEMUR_LENGTH * TIBIA_LENGTH));

    // Calculate ankle components
    double beta = acos((FEMUR_LENGTH * FEMUR_LENGTH +
                        L * L -
                        TIBIA_LENGTH * TIBIA_LENGTH) /
                       (2 * FEMUR_LENGTH * L));
    double gamma = atan2(local_z, local_x);

    // Calculate final angles
    double ankle_angle = -(M_PI - knee_angle); // Keep foot parallel to ground
    double femur_angle = gamma + beta;

    // Convert to degrees and store
    angles->hip = radians_to_degrees(hip_rad);
    angles->knee = radians_to_degrees(femur_angle);
    angles->ankle = radians_to_degrees(ankle_angle);

    printf("Debug - Target: (%.2f, %.2f, %.2f)\n", point->x, point->y, point->z);
    printf("Debug - Local coords: (%.2f, %.2f)\n", local_x, local_z);
    printf("Debug - L=%.2f, beta=%.3f, gamma=%.3f\n", L, beta, gamma);

    return 0;
}
