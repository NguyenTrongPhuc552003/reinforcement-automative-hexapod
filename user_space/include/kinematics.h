#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <stdint.h>
#include "protocol.h"

#define M_PI 3.14159265358979323846

/* 3D Point Structure */
typedef struct point3d
{
    double x;
    double y;
    double z;
} point3d_t;

/* Use the same leg_position struct from protocol.h */
typedef struct leg_position leg_position_t;

/* Forward Kinematics: converts joint angles to cartesian position */
int forward_kinematics(const leg_position_t *angles, point3d_t *point);

/* Inverse Kinematics: converts cartesian position to joint angles */
int inverse_kinematics(const point3d_t *point, leg_position_t *angles);

#endif /* _KINEMATICS_H_ */
