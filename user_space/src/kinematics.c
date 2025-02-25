#include <math.h>
#include <stdio.h>
#include "kinematics.h"

/* Robot dimensions (in mm) */
#define COXA_LENGTH 50.0
#define FEMUR_LENGTH 70.0
#define TIBIA_LENGTH 120.0
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

    // Calculate positions in leg plane
    double femur_x = FEMUR_LENGTH * cos(knee_rad);
    double femur_z = FEMUR_LENGTH * sin(knee_rad);

    double tibia_x = TIBIA_LENGTH * cos(knee_rad + ankle_rad);
    double tibia_z = TIBIA_LENGTH * sin(knee_rad + ankle_rad);

    // Total leg extension
    double leg_x = femur_x + tibia_x;
    double leg_z = -(femur_z + tibia_z); // Negative Z axis points downward

    // Project to 3D space
    point->x = (COXA_OFFSET + leg_x) * cos(hip_rad);
    point->y = (COXA_OFFSET + leg_x) * sin(hip_rad);
    point->z = leg_z;

    return 0;
}

int inverse_kinematics(const point3d_t *point, leg_position_t *angles)
{
    if (!point || !angles)
        return -1;

    // Calculate hip angle
    angles->hip = radians_to_degrees(atan2(point->y, point->x));

    // Transform to 2D problem in leg plane
    double x = sqrt(point->x * point->x + point->y * point->y) - COXA_OFFSET;
    double z = -point->z; // Convert to leg plane coordinates

    double L = sqrt(x * x + z * z);
    double max_reach = FEMUR_LENGTH + TIBIA_LENGTH;
    double min_reach = fabs(FEMUR_LENGTH - TIBIA_LENGTH);

    if (L > max_reach || L < min_reach)
    {
        printf("Point out of reach. L=%.2f, range=[%.2f, %.2f]\n",
               L, min_reach, max_reach);
        return -1;
    }

    // Calculate angles using law of cosines
    double cos_knee = (L * L - FEMUR_LENGTH * FEMUR_LENGTH - TIBIA_LENGTH * TIBIA_LENGTH) /
                      (2 * FEMUR_LENGTH * TIBIA_LENGTH);
    angles->knee = radians_to_degrees(acos(cos_knee));

    // Calculate ankle angle
    double gamma = atan2(z, x);
    double cos_alpha = (L * L + FEMUR_LENGTH * FEMUR_LENGTH - TIBIA_LENGTH * TIBIA_LENGTH) /
                       (2 * FEMUR_LENGTH * L);
    double alpha = acos(cos_alpha);
    angles->ankle = radians_to_degrees(gamma - alpha);

    return 0;
}
