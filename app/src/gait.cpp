#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <unistd.h>
#include <cstdio>
#include <iostream>
#include "gait.hpp"

namespace gait
{

    //==============================================================================
    // Implementation Class (PIMPL idiom)
    //==============================================================================

    class GaitImpl
    {
    public:
        /**
         * @brief Construct a new Gait Implementation object
         *
         * @param hexapod Reference to hexapod controller
         * @param params Initial gait parameters
         */
        GaitImpl(hexapod::Hexapod &hexapod, const GaitParameters &params)
            : hexapod(hexapod),
              params(params),
              initialized(false),
              lastTime(0.0)
        {
            if (!params.validate())
            {
                throw std::invalid_argument("Invalid gait parameters");
            }

            // Initialize leg positions, phase offsets, and states vectors
            initDefaultPositions();
            configureGaitPattern(params.type);

            initialized = true;
        }

        ~GaitImpl() = default;

        /**
         * @brief Initialize default standing positions for each leg
         */
        void initDefaultPositions()
        {
            // Use struct to avoid confusion with Point3D which has different member order
            struct Position
            {
                double x, y, z;
            };

            // Clear and preallocate vectors
            defaultPositions.clear();
            defaultPositions.reserve(hexapod::Config::NUM_LEGS);

            // Assign default positions for standing stance
            defaultPositions = {
                {100.0, 100.0, -100.0},  // Front-right (0)
                {0.0, 120.0, -100.0},    // Middle-right (1)
                {-100.0, 100.0, -100.0}, // Back-right (2)
                {100.0, -100.0, -100.0}, // Front-left (3)
                {0.0, -120.0, -100.0},   // Middle-left (4)
                {-100.0, -100.0, -100.0} // Back-left (5)
            };
        }

        /**
         * @brief Configure phases for different gait patterns
         *
         * @param type The gait type to configure
         */
        void configureGaitPattern(GaitType type)
        {
            // Resize and initialize phase offsets array
            phaseOffsets.resize(hexapod::Config::NUM_LEGS);

            // Configure based on gait type
            switch (type)
            {
            case GaitType::TRIPOD:
                // Legs 0, 2, 4 move together, and 1, 3, 5 move together
                phaseOffsets = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};
                params.dutyFactor = 0.5; // 50% stance phase
                break;

            case GaitType::WAVE:
                // Each leg has a specific phase offset for wave-like motion
                phaseOffsets = {0.0, 0.5, 0.2, 0.7, 0.4, 0.9};
                params.dutyFactor = 0.85; // 85% stance phase
                break;

            case GaitType::RIPPLE:
                // More complex overlapping sequence
                phaseOffsets = {0.0, 0.5, 0.33, 0.83, 0.67, 0.17};
                params.dutyFactor = 0.65; // 65% stance phase
                break;

            default:
                throw std::invalid_argument("Unsupported gait type");
            }

            // Initialize leg states vector (0 = stance, 1 = swing)
            legStates.resize(hexapod::Config::NUM_LEGS, 0);
        }

        /**
         * @brief Update all leg positions for the current time
         *
         * @param time Current time in seconds
         * @param direction Direction of travel in degrees
         * @param speed Movement speed factor (0.0-1.0)
         * @return bool True if update successful
         */
        bool update(double time, double direction, double speed)
        {
            if (!initialized)
            {
                std::cerr << "Gait not initialized" << std::endl;
                return false;
            }

            // Clamp speed to valid range
            speed = std::max(0.0, std::min(1.0, speed));

            // Calculate global phase (0.0 - 1.0) based on time and cycle time
            double phase = std::fmod(time / params.cycleTime, 1.0);

// Debug logging (reduced frequency)
#ifdef DEBUG_GAIT
            static int debugCounter = 0;
            if (++debugCounter % 200 == 0)
            {
                std::cerr << "Gait update: time=" << time
                          << ", phase=" << phase
                          << ", dir=" << direction
                          << ", speed=" << speed << std::endl;
            }
#endif

            // Store current time for future calculations
            lastTime = time;

            // Update each leg position
            bool success = true;
            for (int i = 0; i < hexapod::Config::NUM_LEGS; ++i)
            {
                // Skip update if unsuccessful - prevents partial updates
                if (!updateLegPosition(i, phase, direction, speed))
                {
                    success = false;
                    break;
                }
            }

            return success;
        }

        /**
         * @brief Update position for a specific leg
         *
         * @param legIndex Leg index (0-5)
         * @param globalPhase Global phase of the gait cycle
         * @param direction Direction of travel in degrees
         * @param speed Movement speed factor
         * @return bool True if update successful
         */
        bool updateLegPosition(int legIndex, double globalPhase, double direction, double speed)
        {
            // Calculate leg-specific phase with offset
            double legPhase = std::fmod(globalPhase + phaseOffsets[legIndex], 1.0);

            // Get default position for this leg
            kinematics::Point3D footPos(
                defaultPositions[legIndex].x,
                defaultPositions[legIndex].y,
                defaultPositions[legIndex].z);

            // Apply gait trajectory to modify the position
            computeLegTrajectory(footPos, legPhase, direction, speed);

            // Convert position to joint angles using inverse kinematics
            hexapod::LegPosition angles;
            angles.leg_num = legIndex; // Set leg number for IK to use correct parameters

            if (!kinematics::Kinematics::getInstance().inverseKinematics(footPos, angles))
            {
                std::cerr << "Inverse kinematics failed for leg " << legIndex
                          << ": " << hexapod.getLastErrorMessage() << std::endl;
                return false;
            }

            // Send command to hexapod
            if (!hexapod.setLegPosition(legIndex, angles))
            {
                std::cerr << "Failed to set position for leg " << legIndex
                          << ": " << hexapod.getLastErrorMessage() << std::endl;
                return false;
            }

            return true;
        }

        /**
         * @brief Compute trajectory for a leg at a given phase
         *
         * @param position Position to be modified
         * @param phase Phase within gait cycle (0.0-1.0)
         * @param direction Direction of travel in degrees
         * @param speed Movement speed factor
         */
        void computeLegTrajectory(kinematics::Point3D &position, double phase, double direction, double speed)
        {
            // Cache direction calculations for efficiency
            static double lastDirection = -999.0;
            static double sinAngle = 0.0, cosAngle = 1.0;

            // Recalculate only when direction changes
            if (direction != lastDirection)
            {
                double angleRad = direction * M_PI / 180.0;
                sinAngle = sin(angleRad);
                cosAngle = cos(angleRad);
                lastDirection = direction;
            }

            // Calculate stride length based on speed
            double strideLength = params.stepLength * speed;

            // Calculate stride vector components
            double strideX = strideLength * cosAngle;
            double strideY = strideLength * sinAngle;

// Store original position for debug
#ifdef DEBUG_TRAJECTORY
            double origX = position.x;
            double origY = position.y;
            double origZ = position.z;
#endif

            // Precompute constants for efficiency
            const double dutyFactor = params.dutyFactor;
            const double invDutyFactor = 1.0 / dutyFactor;
            const double invSwingFactor = 1.0 / (1.0 - dutyFactor);

            // Determine if stance or swing phase
            if (phase < dutyFactor)
            {
                // Stance phase - foot is on ground, moving backward
                double stancePhase = phase * invDutyFactor;
                position.x += strideX * (0.5 - stancePhase);
                position.y += strideY * (0.5 - stancePhase);
                // Height remains constant during stance
            }
            else
            {
                // Swing phase - foot is in air, moving forward
                double swingPhase = (phase - dutyFactor) * invSwingFactor;

                // Move foot horizontally
                position.x += strideX * (swingPhase - 0.5);
                position.y += strideY * (swingPhase - 0.5);

                // Add vertical component (parabolic trajectory)
                double swingOffset = swingPhase - 0.5;
                position.z += params.stepHeight * (1.0 - 4.0 * (swingOffset * swingOffset));
            }

// Debug trajectory calculation
#ifdef DEBUG_TRAJECTORY
            static int debugCount = 0;
            if (++debugCount % 500 == 0)
            {
                std::cerr << "Trajectory for phase=" << phase
                          << ": delta=(" << position.x - origX
                          << "," << position.y - origY
                          << "," << position.z - origZ << ")" << std::endl;
            }
#endif
        }

        /**
         * @brief Center all legs to their default positions
         *
         * @return bool True if all legs were successfully centered
         */
        bool centerLegs()
        {
            if (!initialized)
            {
                std::cerr << "Gait not initialized" << std::endl;
                return false;
            }

            for (int i = 0; i < hexapod::Config::NUM_LEGS; ++i)
            {
                // Create Point3D from default position
                kinematics::Point3D pos(
                    defaultPositions[i].x,
                    defaultPositions[i].y,
                    defaultPositions[i].z);

                // Calculate joint angles using inverse kinematics
                hexapod::LegPosition angles;
                angles.leg_num = i;

                if (!kinematics::Kinematics::getInstance().inverseKinematics(pos, angles))
                {
                    std::cerr << "Inverse kinematics failed for leg " << i
                              << ": " << hexapod.getLastErrorMessage() << std::endl;
                    continue; // Continue with other legs even if one fails
                }

                // Send command to hexapod
                if (!hexapod.setLegPosition(i, angles))
                {
                    std::cerr << "Failed to center leg " << i
                              << ": " << hexapod.getLastErrorMessage() << std::endl;
                    return false;
                }

                // Small delay between leg movements for smoother centering
                usleep(50000); // 50ms delay
            }

            return true;
        }

        /**
         * @brief Get current parameters
         *
         * @return GaitParameters Current parameters
         */
        GaitParameters getParameters() const
        {
            return params;
        }

        /**
         * @brief Update parameters
         *
         * @param newParams New parameters to use
         * @return bool True if parameters were updated successfully
         */
        bool setParameters(const GaitParameters &newParams)
        {
            if (!newParams.validate())
            {
                return false;
            }

            // Reconfigure gait pattern if type changed
            if (newParams.type != params.type)
            {
                try
                {
                    configureGaitPattern(newParams.type);
                }
                catch (const std::exception &e)
                {
                    std::cerr << "Failed to configure gait pattern: " << e.what() << std::endl;
                    return false;
                }
            }

            // Update parameters
            params = newParams;
            return true;
        }

        /**
         * @brief Calculate the phase for a specific leg
         *
         * @param legIndex The leg index (0-5)
         * @param time Current time in seconds
         * @return double Phase value (0.0 to 1.0)
         */
        double calculateLegPhase(int legIndex, double time) const
        {
            if (legIndex < 0 || legIndex >= hexapod::Config::NUM_LEGS)
            {
                return 0.0;
            }

            // Calculate global phase
            double globalPhase = std::fmod(time / params.cycleTime, 1.0);

            // Apply leg-specific offset
            return std::fmod(globalPhase + phaseOffsets[legIndex], 1.0);
        }

        /**
         * @brief Check if initialized
         *
         * @return bool True if initialized
         */
        bool isInitialized() const
        {
            return initialized;
        }

        // Member variables
        hexapod::Hexapod &hexapod;
        GaitParameters params;
        bool initialized;
        double lastTime;

        // Simple 3D position structure for leg positions
        struct Position
        {
            double x, y, z;
        };

        std::vector<Position> defaultPositions;
        std::vector<double> phaseOffsets;
        std::vector<int> legStates;
    };

    //==============================================================================
    // Gait Class Implementation
    //==============================================================================

    // Constructor and destructor
    Gait::Gait() : pImpl(nullptr) {}

    Gait::~Gait() = default;

    // Move semantics
    Gait::Gait(Gait &&other) noexcept = default;

    Gait &Gait::operator=(Gait &&other) noexcept = default;

    // Initialization
    bool Gait::init(hexapod::Hexapod &hexapod, const GaitParameters &params)
    {
        try
        {
            pImpl = std::make_unique<GaitImpl>(hexapod, params);
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Gait initialization failed: " << e.what() << std::endl;
            return false;
        }
    }

    // Parameter access
    GaitParameters Gait::getParameters() const
    {
        if (!pImpl)
        {
            return GaitParameters();
        }
        return pImpl->getParameters();
    }

    bool Gait::setParameters(const GaitParameters &params)
    {
        if (!pImpl)
        {
            return false;
        }
        return pImpl->setParameters(params);
    }

    // Movement control
    bool Gait::update(double time, double direction, double speed)
    {
        if (!pImpl)
        {
            return false;
        }
        return pImpl->update(time, direction, speed);
    }

    bool Gait::centerLegs()
    {
        if (!pImpl)
        {
            return false;
        }
        return pImpl->centerLegs();
    }

    // Utility methods
    double Gait::calculateLegPhase(int legIndex, double time) const
    {
        if (!pImpl)
        {
            return 0.0;
        }
        return pImpl->calculateLegPhase(legIndex, time);
    }

    bool Gait::isInitialized() const
    {
        return (pImpl && pImpl->isInitialized());
    }

} // namespace gait
