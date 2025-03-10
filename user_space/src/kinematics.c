#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <errno.h>
#include "kinematics.h"

/* Constants */
#define RAD_TO_DEG 57.2957795131
#define DEG_TO_RAD 0.0174532925

/* Forward kinematics: Convert joint angles to position */
int forward_kinematics(const leg_position_t *angles, point3d_t *position)
{
    double hip_rad, knee_rad, ankle_rad;
    double s1, c1, s2, c2, s23, c23;

    if (!angles || !position)
        return -EINVAL;

    /* Convert angles to radians */
    hip_rad = angles->hip * DEG_TO_RAD;
    knee_rad = angles->knee * DEG_TO_RAD;
    ankle_rad = angles->ankle * DEG_TO_RAD;

    /* Compute sines and cosines */
    s1 = sin(hip_rad);
    c1 = cos(hip_rad);
    s2 = sin(knee_rad);
    c2 = cos(knee_rad);
    s23 = sin(knee_rad + ankle_rad);
    c23 = cos(knee_rad + ankle_rad);

    /* Compute foot position */
    position->x = c1 * (COXA_LENGTH + FEMUR_LENGTH * c2 + TIBIA_LENGTH * c23);
    position->y = s1 * (COXA_LENGTH + FEMUR_LENGTH * c2 + TIBIA_LENGTH * c23);
    position->z = FEMUR_LENGTH * s2 + TIBIA_LENGTH * s23;

    return 0;
}

/* Inverse kinematics: Convert position to joint angles */
int inverse_kinematics(const point3d_t *position, leg_position_t *angles)
{
    double x = position->x;
    double y = position->y;
    double z = position->z;
    double hip, knee, ankle;
    double L, L1, L2, alpha, beta;

    if (!position || !angles)
        return -EINVAL;

    /* Calculate hip angle (yaw) */
    hip = atan2(y, x) * RAD_TO_DEG;

    /* Calculate distance from hip to foot */
    L = sqrt(x * x + y * y) - COXA_LENGTH;
    L1 = sqrt(L * L + z * z);

    /* Check if position is reachable */
    if (L1 > (FEMUR_LENGTH + TIBIA_LENGTH))
    {
        /* Target is too far away - return the closest possible position */
        L1 = FEMUR_LENGTH + TIBIA_LENGTH - 1.0;
    }

    /* Calculate knee angle */
    L2 = (L1 * L1 - FEMUR_LENGTH * FEMUR_LENGTH - TIBIA_LENGTH * TIBIA_LENGTH) /
         (2.0 * FEMUR_LENGTH * TIBIA_LENGTH);
    if (L2 > 1.0)
        L2 = 1.0;
    if (L2 < -1.0)
        L2 = -1.0;

    knee = acos(L2) * RAD_TO_DEG;
    if (z < 0)
        knee = -knee;

    /* Calculate ankle angle */
    alpha = atan2(z, L) * RAD_TO_DEG;
    beta = acos((FEMUR_LENGTH * FEMUR_LENGTH + L1 * L1 - TIBIA_LENGTH * TIBIA_LENGTH) /
                (2.0 * FEMUR_LENGTH * L1)) *
           RAD_TO_DEG;
    if (knee >= 0)
        ankle = alpha - beta;
    else
        ankle = alpha + beta;

    /* Enforce joint limits */
    if (hip < HIP_MIN_ANGLE)
        hip = HIP_MIN_ANGLE;
    if (hip > HIP_MAX_ANGLE)
        hip = HIP_MAX_ANGLE;

    if (knee < KNEE_MIN_ANGLE)
        knee = KNEE_MIN_ANGLE;
    if (knee > KNEE_MAX_ANGLE)
        knee = KNEE_MAX_ANGLE;

    if (ankle < ANKLE_MIN_ANGLE)
        ankle = ANKLE_MIN_ANGLE;
    if (ankle > ANKLE_MAX_ANGLE)
        ankle = ANKLE_MAX_ANGLE;

    /* Store results */
    angles->hip = (int16_t)round(hip);
    angles->knee = (int16_t)round(knee);
    angles->ankle = (int16_t)round(ankle);

    return 0;
}

/* Calculate distance between two 3D points */
double point3d_distance(const point3d_t *p1, const point3d_t *p2)
{
    if (!p1 || !p2)
        return -1.0;

    return sqrt(pow(p2->x - p1->x, 2) +
                pow(p2->y - p1->y, 2) +
                pow(p2->z - p1->z, 2));
}

/* Normalize a 3D vector */
int point3d_normalize(point3d_t *vec)
{
    double length;

    if (!vec)
        return -EINVAL;

    length = sqrt(vec->x * vec->x + vec->y * vec->y + vec->z * vec->z);

    if (length < 1e-6) // Avoid division by near-zero
        return -EDOM;

    vec->x /= length;
    vec->y /= length;
    vec->z /= length;

    return 0;
}

/* Print point coordinates */
void point3d_print(const point3d_t *point)
{
    if (point)
        printf("Point: (%.2f, %.2f, %.2f)\n", point->x, point->y, point->z);
}

/* Check if a position is reachable */
int is_position_reachable(const point3d_t *position)
{
    double distance;

    if (!position)
        return 0;

    /* Calculate distance from origin */
    distance = sqrt(position->x * position->x + position->y * position->y);

    /* Check min/max reach */
    if (distance < (COXA_LENGTH - FEMUR_LENGTH))
        return 0; // Too close to body

    if (distance > (COXA_LENGTH + FEMUR_LENGTH + TIBIA_LENGTH))
        return 0; // Too far from body

    return 1; // Position is potentially reachable
}
