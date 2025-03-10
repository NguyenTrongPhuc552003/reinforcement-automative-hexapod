#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "hexapod.h"

/* 3D point structure */
typedef struct point3d
{
    double x;
    double y;
    double z;
} point3d_t;

/* Kinematics functions */
int forward_kinematics(const leg_position_t *angles, point3d_t *position);
int inverse_kinematics(const point3d_t *position, leg_position_t *angles);

/* Vector operations */
double point3d_distance(const point3d_t *p1, const point3d_t *p2);
int point3d_normalize(point3d_t *vec);
void point3d_print(const point3d_t *point);

/* Utility functions */
int is_position_reachable(const point3d_t *position);

#endif /* KINEMATICS_H */
