#pragma once

#include "leg.hpp"
#include "kinematics.hpp"
#include <array>
#include <memory>

class Hexapod {
public:
    static constexpr size_t NUM_LEGS = 6;
    
    Hexapod();
    void init();
    void update();
    void setLegPosition(size_t leg_index, float x, float y, float z);
    void setAllLegPositions(const std::array<Vec3, NUM_LEGS>& positions);
    void setGaitPattern(int pattern);
    void setDirection(float x, float y, float rotation);

private:
    Kinematics kinematics;
    std::array<std::unique_ptr<Leg>, NUM_LEGS> legs;  // Changed to use unique_ptr
    float dir_x{0}, dir_y{0}, dir_rot{0};
    int current_gait{0};

    static constexpr std::array<int, NUM_LEGS * 3> SERVO_CHANNELS = {
        0, 1, 2,    // Leg 0: coxa, femur, tibia
        3, 4, 5,    // Leg 1
        6, 7, 8,    // Leg 2
        9, 10, 11,  // Leg 3
        12, 13, 14, // Leg 4
        15, 16, 17  // Leg 5
    };
};
