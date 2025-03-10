#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <unistd.h>
#include "gait.hpp"

// Implementation class (PIMPL idiom)
class GaitImpl
{
public:
    GaitImpl(Hexapod &hexapod, const GaitParameters &params) : hexapod(hexapod), params(params), initialized(false), lastTime(0.0)
    {
        // Set default leg positions (standing stance)
        defaultPositions = {
            {100.0, 100.0, -100.0},  // Front-right
            {0.0, 120.0, -100.0},    // Middle-right
            {-100.0, 100.0, -100.0}, // Back-right
            {100.0, -100.0, -100.0}, // Front-left
            {0.0, -120.0, -100.0},   // Middle-left
            {-100.0, -100.0, -100.0} // Back-left
        };

        // Initialize phase offsets based on gait type
        configureGaitPattern(params.type);
    }

    ~GaitImpl() = default;

    // Configure phases for different gait patterns
    void configureGaitPattern(GaitType type)
    {
        phaseOffsets.resize(NUM_LEGS);

        switch (type)
        {
        case GaitType::TRIPOD:
            // Legs 0, 2, 4 move together, and 1, 3, 5 move together
            phaseOffsets = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};
            params.dutyFactor = 0.5; // 50% stance phase
            break;

        case GaitType::WAVE:
            // Each leg is 1/6 cycle out of phase with the next
            phaseOffsets = {0.0, 0.5, 0.2, 0.7, 0.4, 0.9};
            params.dutyFactor = 0.85; // 85% stance phase
            params.stepHeight = 40.0; // Higher steps
            break;

        case GaitType::RIPPLE:
            // More complex overlapping sequence
            phaseOffsets = {0.0, 0.5, 0.33, 0.83, 0.67, 0.17};
            params.dutyFactor = 0.65; // 65% stance phase
            break;

        default:
            throw std::invalid_argument("Invalid gait type");
        }

        // Create storage for leg states
        legStates.resize(NUM_LEGS, 0); // 0 = stance, 1 = swing

        initialized = true;
    }

    // Update all leg positions for the current time
    bool update(double time, double direction, double speed)
    {
        if (!initialized)
        {
            return false;
        }

        // Calculate global phase (0.0 - 1.0)
        double phase = std::fmod(time / params.cycleTime, 1.0);

        // Store current time
        lastTime = time;

        // Update each leg
        for (int i = 0; i < NUM_LEGS; ++i)
        {
            // Calculate leg phase (with offset)
            double legPhase = std::fmod(phase + phaseOffsets[i], 1.0);

            // Get default position for this leg
            Point3D footPos(
                defaultPositions[i].x,
                defaultPositions[i].y,
                defaultPositions[i].z);

            // Apply stride pattern
            computeLegTrajectory(footPos, legPhase, direction, speed);

            // Convert position to joint angles
            LegPosition angles;
            if (!Kinematics::getInstance().inverseKinematics(footPos, angles))
            {
                fprintf(stderr, "Inverse kinematics failed for leg %d\n", i);
                continue;
            }

            // Send command to hexapod
            if (!hexapod.setLegPosition(i, angles))
            {
                fprintf(stderr, "Failed to set position for leg %d: %s\n",
                        i, hexapod.getLastErrorMessage().c_str());
                return false;
            }
        }

        return true;
    }

    // Center all legs in default position
    bool centerLegs()
    {
        for (int i = 0; i < NUM_LEGS; ++i)
        {
            Point3D pos(
                defaultPositions[i].x,
                defaultPositions[i].y,
                defaultPositions[i].z);

            LegPosition angles;
            if (!Kinematics::getInstance().inverseKinematics(pos, angles))
            {
                fprintf(stderr, "Inverse kinematics failed during centering for leg %d\n", i);
                continue;
            }

            if (!hexapod.setLegPosition(i, angles))
            {
                fprintf(stderr, "Failed to center leg %d: %s\n",
                        i, hexapod.getLastErrorMessage().c_str());
                return false;
            }

            // Small delay between legs for smoother movement
            usleep(50000);
        }

        return true;
    }

    // Get current parameters
    GaitParameters getParameters() const
    {
        return params;
    }

    // Update parameters
    bool setParameters(const GaitParameters &newParams)
    {
        if (newParams.type != params.type)
        {
            // Need to reconfigure gait pattern for new type
            configureGaitPattern(newParams.type);
        }

        params = newParams;
        return true;
    }

private:
    // Compute leg trajectory for a given phase
    void computeLegTrajectory(Point3D &position, double phase, double direction, double speed)
    {
        // Convert direction from degrees to radians
        double angleRad = direction * M_PI / 180.0;
        double strideLength = params.stepLength * speed;

        // Calculate stride vector components
        double strideX = strideLength * cos(angleRad);
        double strideY = strideLength * sin(angleRad);

        // Determine if stance or swing phase
        if (phase < params.dutyFactor)
        {
            // Stance phase - foot is on ground, moving backward
            double stancePhase = phase / params.dutyFactor;
            position.x += strideX * (0.5 - stancePhase);
            position.y += strideY * (0.5 - stancePhase);
            // Height is constant during stance phase
        }
        else
        {
            // Swing phase - foot is in air, moving forward
            double swingPhase = (phase - params.dutyFactor) / (1.0 - params.dutyFactor);
            position.x += strideX * (swingPhase - 0.5);
            position.y += strideY * (swingPhase - 0.5);

            // Add vertical component - parabolic trajectory for swing
            position.z += params.stepHeight * (1.0 - 4.0 * pow(swingPhase - 0.5, 2.0));
        }
    }

    // Member variables
    Hexapod &hexapod;
    GaitParameters params;
    bool initialized;
    double lastTime;

    struct Point3DSimple
    {
        double x, y, z;
    };

    std::vector<Point3DSimple> defaultPositions;
    std::vector<double> phaseOffsets;
    std::vector<int> legStates;
};

// Gait class implementation
Gait::Gait() : pImpl(nullptr) {}

Gait::~Gait() = default;

Gait::Gait(Gait &&other) noexcept = default;

Gait &Gait::operator=(Gait &&other) noexcept = default;

bool Gait::init(Hexapod &hexapod, const GaitParameters &params)
{
    try
    {
        pImpl = std::make_unique<GaitImpl>(hexapod, params);
        return true;
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "Gait initialization failed: %s\n", e.what());
        return false;
    }
}

bool Gait::update(double time, double direction, double speed)
{
    if (!pImpl)
        return false;
    return pImpl->update(time, direction, speed);
}

bool Gait::centerLegs()
{
    if (!pImpl)
        return false;
    return pImpl->centerLegs();
}

GaitParameters Gait::getParameters() const
{
    if (!pImpl)
        return GaitParameters();
    return pImpl->getParameters();
}

bool Gait::setParameters(const GaitParameters &params)
{
    if (!pImpl)
        return false;
    return pImpl->setParameters(params);
}
