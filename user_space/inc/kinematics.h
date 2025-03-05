#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <stdint.h>
#include "protocol.h"
#include "hexapod.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 3D Point Structure */
typedef struct point3d
{
    double x;
    double y;
    double z;
} point3d_t;

/* Kinematics API */
int forward_kinematics(const leg_position_t *angles, point3d_t *point);
int inverse_kinematics(const point3d_t *point, leg_position_t *angles);

/* Utility functions */
double degrees_to_radians(double degrees);
double radians_to_degrees(double radians);

#endif /* _KINEMATICS_H_ */
