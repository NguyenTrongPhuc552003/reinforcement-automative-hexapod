#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <stdint.h>

/* 3D Point Structure */
typedef struct
{
    double x;
    double y;
    double z;
} point3d_t;

/* Leg angles structure - must match hexapod.h leg_position_t */
typedef struct
{
    double hip;
    double knee;
    double ankle;
} leg_angles_t;

/* Forward Kinematics: converts joint angles to cartesian position */
int forward_kinematics(const leg_angles_t *angles, point3d_t *point);

/* Inverse Kinematics: converts cartesian position to joint angles */
int inverse_kinematics(const point3d_t *point, leg_angles_t *angles);

#endif /* _KINEMATICS_H_ */
