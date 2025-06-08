#include "hexapod.hpp"

Hexapod::Hexapod()
    : kinematics(25.0f, 50.0f, 75.0f) // coxa, femur, tibia lengths
{
    // Initialize legs with their servo channels
    legs[0] = std::make_unique<Leg>(0, 0, 1, 2);    // Front Right
    legs[1] = std::make_unique<Leg>(1, 3, 4, 5);    // Middle Right
    legs[2] = std::make_unique<Leg>(2, 6, 7, 8);    // Back Right
    legs[3] = std::make_unique<Leg>(3, 9, 10, 11);  // Back Left
    legs[4] = std::make_unique<Leg>(4, 12, 13, 14); // Middle Left
    legs[5] = std::make_unique<Leg>(5, 15, 16, 17); // Front Left
}

void Hexapod::init() {
    // Set initial position for all legs
    for (auto& leg : legs) {
        leg->setTargetPosition(0, 0, -100); // Default standing height
    }
}

void Hexapod::update() {
    for (auto& leg : legs) {
        leg->update();
    }
}

void Hexapod::setLegPosition(size_t leg_index, float x, float y, float z) {
    if (leg_index < NUM_LEGS) {
        legs[leg_index]->setTargetPosition(x, y, z);
    }
}

void Hexapod::setAllLegPositions(const std::array<Vec3, NUM_LEGS>& positions) {
    for (size_t i = 0; i < NUM_LEGS; i++) {
        legs[i]->setTargetPosition(positions[i].x, positions[i].y, positions[i].z);
    }
}

void Hexapod::setGaitPattern(int pattern) {
    current_gait = pattern;
}

void Hexapod::setDirection(float x, float y, float rotation) {
    dir_x = x;
    dir_y = y;
    dir_rot = rotation;
}
