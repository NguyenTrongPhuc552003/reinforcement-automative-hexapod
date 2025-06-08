#ifndef GAIT_CONTROLLER_HPP
#define GAIT_CONTROLLER_HPP

#include "kinematics.hpp"
#include "hexapod.hpp"
#include <array>

class GaitController {
public:
    enum class Mode { STAND, WALK };
    enum class Direction { FORWARD, BACKWARD, LEFT, RIGHT };
    enum class GaitType { TRIPOD, WAVE };

    explicit GaitController(GaitType type = GaitType::TRIPOD, float step_height = 30.0f, float step_length = 50.0f);
    
    void attachHexapod(Hexapod* hexapod);
    void setMode(Mode mode);
    void setDirection(Direction dir);
    void update(float time_delta);
    std::array<Vec3, Kinematics::NUM_LEGS> getTargetPositions() const;

private:
    GaitType gait_type;
    Mode current_mode;
    Direction current_direction;
    Hexapod* hexapod;
    float time;
    float step_height;
    float step_length;
    std::array<Vec3, Kinematics::NUM_LEGS> positions;

    void updateTripod(float t);
    void updateWave(float t);
};

#endif // GAIT_CONTROLLER_HPP
