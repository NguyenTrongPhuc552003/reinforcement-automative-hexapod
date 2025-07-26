#ifndef GAIT_HPP
#define GAIT_HPP

#include "kinematics.hpp"

class Gait
{
public:
    static const int NUM_LEGS = 6;

    Gait();

    void update(float dt);
    void setSpeed(float speed);
    void setDirection(float direction); // radians

    Point3D getLegPosition(int leg);

private:
    static const float STEP_HEIGHT;
    static const float STEP_LENGTH;
    static const float LEG_RADIUS;

    float speed;
    float direction;
    float phase;

    Point3D defaultPositions[NUM_LEGS];

    void initDefaultPositions();
    Point3D calculateStepPosition(int leg, float legPhase);
};

#endif
