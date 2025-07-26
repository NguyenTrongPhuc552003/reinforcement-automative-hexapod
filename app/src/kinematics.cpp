#include "kinematics.hpp"
#include <cmath>

const float Kinematics::COXA_LENGTH = 30.0f;   // mm
const float Kinematics::FEMUR_LENGTH = 85.0f;  // mm
const float Kinematics::TIBIA_LENGTH = 130.0f; // mm

Angles Kinematics::inverseKinematics(const Point3D &target)
{
    float x = target.x;
    float y = target.y;
    float z = target.z;

    // Coxa angle
    float coxa = atan2f(y, x);

    // Distance from coxa joint to target
    float distance = sqrtf(x * x + y * y) - COXA_LENGTH;
    float totalDist = sqrtf(distance * distance + z * z);

    // Check if target is reachable
    float maxReach = FEMUR_LENGTH + TIBIA_LENGTH;
    if (totalDist > maxReach)
    {
        totalDist = maxReach;
    }

    // Femur angle
    float a1 = atan2f(z, distance);
    float a2 = acosf((FEMUR_LENGTH * FEMUR_LENGTH + totalDist * totalDist - TIBIA_LENGTH * TIBIA_LENGTH) /
                     (2 * FEMUR_LENGTH * totalDist));
    float femur = a1 + a2;

    // Tibia angle
    float tibia = acosf((FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - totalDist * totalDist) /
                        (2 * FEMUR_LENGTH * TIBIA_LENGTH));
    tibia = M_PI - tibia; // Adjust for servo orientation

    return Angles(radToDeg(coxa), radToDeg(femur), radToDeg(tibia));
}

Point3D Kinematics::forwardKinematics(const Angles &angles)
{
    float coxa_rad = degToRad(angles.coxa);
    float femur_rad = degToRad(angles.femur);
    float tibia_rad = degToRad(angles.tibia);

    float x = cosf(coxa_rad) * (COXA_LENGTH + FEMUR_LENGTH * cosf(femur_rad) +
                                TIBIA_LENGTH * cosf(femur_rad + tibia_rad));
    float y = sinf(coxa_rad) * (COXA_LENGTH + FEMUR_LENGTH * cosf(femur_rad) +
                                TIBIA_LENGTH * cosf(femur_rad + tibia_rad));
    float z = FEMUR_LENGTH * sinf(femur_rad) + TIBIA_LENGTH * sinf(femur_rad + tibia_rad);

    return Point3D(x, y, z);
}

float Kinematics::radToDeg(float rad)
{
    return rad * 180.0f / M_PI;
}

float Kinematics::degToRad(float deg)
{
    return deg * M_PI / 180.0f;
}
