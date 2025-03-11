#ifndef GAIT_HPP
#define GAIT_HPP

#include <memory>
#include <vector>
#include "kinematics.hpp"

// Forward declaration for implementation
class GaitImpl;

// Gait types
enum class GaitType
{
    TRIPOD, // Alternating tripod (3 legs at a time)
    WAVE,   // Wave gait (one leg at a time)
    RIPPLE  // Ripple gait (overlapping movement)
};

// Gait parameters
class GaitParameters
{
public:
    GaitType type = GaitType::TRIPOD; // Type of gait pattern
    double stepHeight = 30.0;         // Maximum step height (mm)
    double stepLength = 60.0;         // Maximum stride length (mm)
    double cycleTime = 2.0;           // Time for complete gait cycle (seconds)
    double dutyFactor = 0.5;          // Portion of cycle in stance phase (0.0-1.0)
};

// Gait control class
class Gait
{
public:
    Gait();
    ~Gait();

    // Non-copyable
    Gait(const Gait &) = delete;
    Gait &operator=(const Gait &) = delete;

    // Move constructor and assignment
    Gait(Gait &&) noexcept;
    Gait &operator=(Gait &&) noexcept;

    // Initialize gait controller
    bool init(Hexapod &hexapod, const GaitParameters &params);

    // Update gait for all legs
    bool update(double time, double direction, double speed);

    // Center all legs
    bool centerLegs();

    // Get current parameters
    GaitParameters getParameters() const;

    // Update parameters
    bool setParameters(const GaitParameters &params);

private:
    std::unique_ptr<GaitImpl> pImpl;
};

#endif /* GAIT_HPP */
