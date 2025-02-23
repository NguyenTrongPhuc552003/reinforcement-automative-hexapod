#include <math.h>
#include <stdio.h>
#include "kinematics.h"

/* Robot dimensions (in mm) */
#define COXA_LENGTH 50.0
#define FEMUR_LENGTH 70.0
#define TIBIA_LENGTH 120.0

static double degrees_to_radians(double degrees)
{
    return degrees * M_PI / 180.0;
}

static double radians_to_degrees(double radians)
{
    return radians * 180.0 / M_PI;
}

int forward_kinematics(const leg_angles_t *angles, point3d_t *point)
{
    if (!angles || !point)
        return -1;

    // Convert angles to radians
    double hip_rad = degrees_to_radians(angles->hip);
    double knee_rad = degrees_to_radians(angles->knee);
    double ankle_rad = degrees_to_radians(angles->ankle);

    // Calculate leg plane extension and height
    double leg_projection = FEMUR_LENGTH * cos(knee_rad) +
                            TIBIA_LENGTH * cos(knee_rad + ankle_rad);
    double leg_height = FEMUR_LENGTH * sin(knee_rad) +
                        TIBIA_LENGTH * sin(knee_rad + ankle_rad);

    // Apply hip rotation to get final position
    double total_reach = COXA_LENGTH + leg_projection;
    point->x = total_reach * cos(hip_rad);
    point->y = total_reach * sin(hip_rad);
    point->z = -leg_height; // Negative because down is positive

    return 0;
}

int inverse_kinematics(const point3d_t *point, leg_angles_t *angles)
{
    if (!point || !angles)
        return -1;

    double x = point->x;
    double y = point->y;
    double z = -point->z; // Convert to internal coordinates

    // Calculate hip angle
    angles->hip = radians_to_degrees(atan2(y, x));

    // Transform to 2D problem in leg plane
    double leg_plane_reach = sqrt(x * x + y * y) - COXA_LENGTH;
    double target_distance = sqrt(leg_plane_reach * leg_plane_reach + z * z);

    // Check if point is reachable
    if (target_distance > (FEMUR_LENGTH + TIBIA_LENGTH))
    {
        printf("Point out of reach: distance=%.2f max=%.2f\n",
               target_distance, FEMUR_LENGTH + TIBIA_LENGTH);
        return -1;
    }

    // Calculate knee angle using law of cosines
    double knee_cos = (leg_plane_reach * leg_plane_reach + z * z +
                       FEMUR_LENGTH * FEMUR_LENGTH - TIBIA_LENGTH * TIBIA_LENGTH) /
                      (2.0 * FEMUR_LENGTH * sqrt(leg_plane_reach * leg_plane_reach + z * z));
    double knee_angle = acos(knee_cos);

    // Calculate leg plane angle
    double leg_angle = atan2(z, leg_plane_reach);
    angles->knee = radians_to_degrees(leg_angle + knee_angle);

    // Calculate ankle angle
    double ankle_cos = (FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH -
                        leg_plane_reach * leg_plane_reach - z * z) /
                       (2.0 * FEMUR_LENGTH * TIBIA_LENGTH);
    angles->ankle = radians_to_degrees(acos(ankle_cos));

    return 0;
}
