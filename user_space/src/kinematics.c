#include <math.h>
#include <string.h>
#include "kinematics.h"

/* Robot dimensions (in mm) */
#define COXA_LENGTH     50.0
#define FEMUR_LENGTH    70.0
#define TIBIA_LENGTH    120.0

/* Convert degrees to radians */
double degrees_to_radians(double degrees)
{
    return degrees * M_PI / 180.0;
}

/* Convert radians to degrees */
double radians_to_degrees(double radians)
{
    return radians * 180.0 / M_PI;
}

/* Normalize angles to [-180, 180] range */
void normalize_angles(leg_position_t *angles)
{
    double *angle_array[] = {&angles->hip, &angles->knee, &angles->ankle};
    
    for (int i = 0; i < 3; i++) {
        double *angle = angle_array[i];
        *angle = fmod(*angle + 180.0, 360.0);
        if (*angle < 0) {
            *angle += 360.0;
        }
        *angle -= 180.0;
    }
}

/* Forward kinematics: joint angles -> end effector position */
int forward_kinematics(const leg_position_t *angles, point3d_t *point)
{
    if (!angles || !point) {
        return -1;
    }

    /* Convert angles to radians */
    double hip = degrees_to_radians(angles->hip);
    double knee = degrees_to_radians(angles->knee);
    double ankle = degrees_to_radians(angles->ankle);

    /* Calculate intermediate values */
    double cos_hip = cos(hip);
    double sin_hip = sin(hip);
    double cos_knee = cos(knee);
    double sin_knee = sin(knee);
    double cos_ankle = cos(ankle);
    double sin_ankle = sin(ankle);

    /* Calculate end effector position */
    double r = COXA_LENGTH * cos_knee + 
               FEMUR_LENGTH * cos_knee * cos_ankle - 
               TIBIA_LENGTH * sin_knee * sin_ankle;
    
    point->x = r * cos_hip;
    point->y = r * sin_hip;
    point->z = COXA_LENGTH * sin_knee + 
               FEMUR_LENGTH * sin_knee * cos_ankle + 
               TIBIA_LENGTH * cos_knee * sin_ankle;

    return 0;
}

/* Inverse kinematics: end effector position -> joint angles */
int inverse_kinematics(const point3d_t *point, leg_position_t *angles)
{
    if (!point || !angles) {
        return -1;
    }

    /* Calculate hip angle */
    angles->hip = radians_to_degrees(atan2(point->y, point->x));

    /* Calculate distance to end effector in xz plane */
    double r = sqrt(point->x * point->x + point->y * point->y);
    double z = point->z;

    /* Calculate knee and ankle angles using cosine law */
    double d = sqrt(r * r + z * z);
    double a2 = FEMUR_LENGTH;
    double b2 = TIBIA_LENGTH;
    double c2 = d;

    /* Check if point is reachable */
    if (d > (FEMUR_LENGTH + TIBIA_LENGTH)) {
        return -1;  /* Point out of reach */
    }

    /* Calculate knee angle */
    double cos_knee = (a2*a2 + c2*c2 - b2*b2) / (2*a2*c2);
    if (cos_knee > 1.0) cos_knee = 1.0;
    if (cos_knee < -1.0) cos_knee = -1.0;
    angles->knee = radians_to_degrees(acos(cos_knee));

    /* Calculate ankle angle */
    double cos_ankle = (a2*a2 + b2*b2 - c2*c2) / (2*a2*b2);
    if (cos_ankle > 1.0) cos_ankle = 1.0;
    if (cos_ankle < -1.0) cos_ankle = -1.0;
    angles->ankle = radians_to_degrees(acos(cos_ankle));

    /* Normalize angles */
    normalize_angles(angles);

    return 0;
}
