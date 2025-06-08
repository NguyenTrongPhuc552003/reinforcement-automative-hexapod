#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <array>
#include <cstdint>
#include <cstddef>  // Added for size_t
#include <cmath>

struct Vec3 {
    float x, y, z;
};

class Kinematics {
public:
    static constexpr size_t NUM_LEGS = 6;

    Kinematics(float coxa_length, float femur_length, float tibia_length);

    // Tính toán góc servo từ tọa độ điểm đích
    std::array<float, 3> computeIK(const Vec3& foot_pos) const;

    // Cập nhật vị trí mục tiêu cho từng chân
    std::array<std::array<float, 3>, NUM_LEGS> computeAllIK(const std::array<Vec3, NUM_LEGS>& target_positions) const;

private:
    float l1, l2, l3;  // độ dài các khớp: coxa, femur, tibia
};

#endif // KINEMATICS_HPP
