#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <stdint.h>
#include "hexapod.h"

/* 3D Point Structure */
typedef struct {
    double x;
    double y;
    double z;
} point3d_t;

/* Forward Kinematics */
int forward_kinematics(const leg_position_t *angles, point3d_t *point);

/* Inverse Kinematics */
int inverse_kinematics(const point3d_t *point, leg_position_t *angles);

/* Utility Functions */
double degrees_to_radians(double degrees);
double radians_to_degrees(double radians);
void normalize_angles(leg_position_t *angles);

#endif /* _KINEMATICS_H_ */
