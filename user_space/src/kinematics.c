#include <math.h>
#include <stdio.h>
#include "kinematics.h"

/* Robot dimensions (in mm) - Actual measurements */
#define COXA_LENGTH 30.0
#define FEMUR_LENGTH 85.0
#define TIBIA_LENGTH 123.0
#define COXA_OFFSET COXA_LENGTH

/* Workspace limits */
#define MIN_REACH 45.0    /* Minimum reach distance */
#define MAX_REACH 180.0   /* Maximum reach distance */
#define MAX_HEIGHT 70.0   /* Maximum height */
#define MIN_HEIGHT -150.0 /* Minimum height (negative = below base) */

/* Convert degrees to radians */
static double deg_to_rad(double degrees)
{
    return degrees * M_PI / 180.0;
}

/* Convert radians to degrees */
static double rad_to_deg(double radians)
{
    return radians * 180.0 / M_PI;
}

int forward_kinematics(const leg_position_t *angles, point3d_t *position)
{
    if (!angles || !position)
        return -1;

    /* Convert angles to radians */
    double hip_rad = deg_to_rad(angles->hip);
    double knee_rad = deg_to_rad(angles->knee);
    double ankle_rad = deg_to_rad(angles->ankle);

    /* Calculate intermediate positions of each joint */
    double coxa_x = COXA_LENGTH * cos(hip_rad);
    double coxa_y = COXA_LENGTH * sin(hip_rad);

    double femur_angle = hip_rad + knee_rad;
    double femur_x = coxa_x + FEMUR_LENGTH * cos(femur_angle);
    double femur_y = coxa_y + FEMUR_LENGTH * sin(femur_angle);

    double tibia_angle = femur_angle + ankle_rad;

    /* Calculate end position */
    position->x = femur_x + TIBIA_LENGTH * cos(tibia_angle);
    position->y = femur_y + TIBIA_LENGTH * sin(tibia_angle);
    position->z = 0.0; /* Assuming Z is perpendicular to the X-Y plane */

    return 0;
}

int inverse_kinematics(const point3d_t *position, leg_position_t *angles)
{
    if (!position || !angles)
        return -1;

    /* Calculate distance from origin to target */
    double x = position->x;
    double y = position->y;
    double z = position->z;

    double distance_xy = sqrt(x * x + y * y);

    /* Check if point is within reachable workspace */
    if (distance_xy < MIN_REACH || distance_xy > MAX_REACH ||
        z > MAX_HEIGHT || z < MIN_HEIGHT)
    {
        return -1; /* Point is outside workspace */
    }

    /* Calculate hip angle */
    double hip_angle = atan2(y, x);

    /* Adjust distances for hip joint offset */
    double coxa_offset_x = COXA_LENGTH * cos(hip_angle);
    double coxa_offset_y = COXA_LENGTH * sin(hip_angle);

    /* Target for knee/ankle joints */
    double target_x = x - coxa_offset_x;
    double target_y = y - coxa_offset_y;
    double target_dist_xy = sqrt(target_x * target_x + target_y * target_y);
    double target_dist = sqrt(target_dist_xy * target_dist_xy + z * z);

    /* Check if target is reachable by femur+tibia */
    if (target_dist > FEMUR_LENGTH + TIBIA_LENGTH)
    {
        return -1; /* Target too far */
    }

    /* Law of cosines to find knee angle */
    double knee_angle = acos(
        (FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - target_dist * target_dist) /
        (2 * FEMUR_LENGTH * TIBIA_LENGTH));

    /* Angle of the leg, accounting for XY plane and Z height */
    double leg_angle = atan2(-z, target_dist_xy);

    /* Use law of sines to find angle between femur and target line */
    double inner_angle = asin(
        TIBIA_LENGTH * sin(knee_angle) / target_dist);

    /* Calculate femur angle */
    double femur_angle = leg_angle + inner_angle;

    /* Calculate final angles in degrees */
    angles->hip = rad_to_deg(hip_angle);
    angles->knee = rad_to_deg(femur_angle - hip_angle);
    angles->ankle = rad_to_deg(M_PI - knee_angle - femur_angle);

    return 0;
}
