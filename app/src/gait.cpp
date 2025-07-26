#include "gait.hpp"
#include <cmath>

const float Gait::STEP_HEIGHT = 30.0f; // mm
const float Gait::STEP_LENGTH = 60.0f; // mm
const float Gait::LEG_RADIUS = 120.0f; // mm from center

Gait::Gait() : speed(0.0f), direction(0.0f), phase(0.0f)
{
    initDefaultPositions();
}

void Gait::initDefaultPositions()
{
    // Leg positions around hexapod (front=0°, right=90°)
    float angles[] = {-30, 30, 90, 150, 210, 270}; // degrees

    for (int i = 0; i < NUM_LEGS; i++)
    {
        float angle = angles[i] * M_PI / 180.0f;
        defaultPositions[i] = Point3D(
            LEG_RADIUS * cosf(angle),
            LEG_RADIUS * sinf(angle),
            -80.0f // Default height
        );
    }
}

void Gait::update(float dt)
{
    if (speed > 0.01f)
    {
        phase += speed * dt;
        if (phase > 2 * M_PI)
        {
            phase -= 2 * M_PI;
        }
    }
}

void Gait::setSpeed(float newSpeed)
{
    speed = newSpeed;
}

void Gait::setDirection(float newDirection)
{
    direction = newDirection;
}

Point3D Gait::getLegPosition(int leg)
{
    if (speed < 0.01f)
    {
        return defaultPositions[leg];
    }

    // Tripod gait: legs 0,2,4 vs legs 1,3,5
    float legPhase = phase;
    if (leg % 2 == 1)
    {
        legPhase += M_PI; // 180° phase shift
    }

    return calculateStepPosition(leg, legPhase);
}

Point3D Gait::calculateStepPosition(int leg, float legPhase)
{
    Point3D base = defaultPositions[leg];

    // Normalize phase to [0, 2π]
    while (legPhase < 0)
        legPhase += 2 * M_PI;
    while (legPhase >= 2 * M_PI)
        legPhase -= 2 * M_PI;

    float stepX = STEP_LENGTH * cosf(direction);
    float stepY = STEP_LENGTH * sinf(direction);

    if (legPhase < M_PI)
    {
        // Stance phase (on ground)
        float t = legPhase / M_PI;
        return Point3D(
            base.x + stepX * (0.5f - t),
            base.y + stepY * (0.5f - t),
            base.z);
    }
    else
    {
        // Swing phase (in air)
        float t = (legPhase - M_PI) / M_PI;
        float height = STEP_HEIGHT * sinf(t * M_PI);

        return Point3D(
            base.x + stepX * (t - 0.5f),
            base.y + stepY * (t - 0.5f),
            base.z + height);
    }
}
