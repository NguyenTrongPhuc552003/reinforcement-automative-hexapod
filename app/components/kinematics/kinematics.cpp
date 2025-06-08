#include "kinematics.hpp"
#include <cmath>

Kinematics::Kinematics(float coxa_length, float femur_length, float tibia_length)
    : l1(coxa_length), l2(femur_length), l3(tibia_length) {}

std::array<float, 3> Kinematics::computeIK(const Vec3& foot_pos) const {
    float x = foot_pos.x;
    float y = foot_pos.y;
    float z = foot_pos.z;

    float theta1 = atan2(y, x);

    float d = sqrt(x*x + y*y) - l1;
    float r = sqrt(d*d + z*z);

    float alpha = acos((l2*l2 + r*r - l3*l3) / (2 * l2 * r));
    float beta = atan2(-z, d);

    float theta2 = alpha + beta;

    float gamma = acos((l2*l2 + l3*l3 - r*r) / (2 * l2 * l3));
    float theta3 = M_PI - gamma;

    return {theta1, theta2, theta3};
}

std::array<std::array<float, 3>, Kinematics::NUM_LEGS> Kinematics::computeAllIK(const std::array<Vec3, NUM_LEGS>& target_positions) const {
    std::array<std::array<float, 3>, NUM_LEGS> result;
    for (size_t i = 0; i < NUM_LEGS; ++i) {
        result[i] = computeIK(target_positions[i]);
    }
    return result;
}
