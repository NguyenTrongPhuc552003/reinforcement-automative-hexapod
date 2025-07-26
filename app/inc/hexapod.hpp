#ifndef HEXAPOD_HPP
#define HEXAPOD_HPP

#include "pca9685.hpp"
#include "ultrasonic.hpp"
#include "gait.hpp"
#include "kinematics.hpp"

class Hexapod
{
public:
    Hexapod();
    ~Hexapod();

    bool init();
    void run(); // Main autonomous loop

private:
    PCA9685 pca1;
    PCA9685 pca2;
    Ultrasonic ultrasonic;
    Gait gait;

    static const float OBSTACLE_THRESHOLD; // cm
    static const float TURN_SPEED;
    static const float WALK_SPEED;

    void updateServos();
    void setLegAngles(int leg, const Angles &angles);
    void obstacleAvoidance();

    // Servo mapping: leg * 3 + joint (0=coxa, 1=femur, 2=tibia)
    int getServoChannel(int leg, int joint);
};

#endif
