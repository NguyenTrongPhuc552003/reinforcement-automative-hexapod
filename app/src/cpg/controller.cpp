/*
 * Hexapod Project - A Reinforcement Learning-based Autonomous Hexapod
 * Copyright (C) 2025  Nguyen Trong Phuc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include "cpg/controller.hpp"

namespace cpg
{

    //==============================================================================
    // Constants and Utilities
    //==============================================================================

    namespace
    {
        // Control constants
        constexpr double PI = M_PI;
        constexpr double TWO_PI = 2.0 * M_PI;
        constexpr size_t HEXAPOD_LEGS = 6;
        constexpr size_t JOINTS_PER_LEG = 3;
        constexpr size_t TOTAL_JOINTS = HEXAPOD_LEGS * JOINTS_PER_LEG;

        // Physical constraints
        constexpr double MAX_JOINT_ANGLE = PI / 2.0; // ±90 degrees
        constexpr double MAX_JOINT_VELOCITY = 5.0;   // 5 rad/s
        constexpr double DEFAULT_STEP_LENGTH = 0.08; // 8 cm
        constexpr double DEFAULT_BODY_HEIGHT = 0.15; // 15 cm

        // Control limits
        constexpr double MIN_FREQUENCY_SCALE = 0.1;
        constexpr double MAX_FREQUENCY_SCALE = 3.0;
        constexpr double EMERGENCY_STOP_TIME = 0.5;    // 500ms emergency stop

        // Performance thresholds
        constexpr double MIN_STABILITY_MARGIN = 0.05;       // 5% minimum stability
        constexpr double ENERGY_EFFICIENCY_THRESHOLD = 0.7; // 70% efficiency threshold
    }

    //==============================================================================
    // Controller State Machine
    //==============================================================================

    enum class ControllerMode
    {
        IDLE,          ///< Controller inactive
        INITIALIZING,  ///< Starting up
        WALKING,       ///< Active locomotion
        TRANSITIONING, ///< Gait transition in progress
        STOPPING,      ///< Graceful stop in progress
        EMERGENCY,     ///< Emergency stop
        ERROR          ///< Error state
    };

    //==============================================================================
    // Implementation Class (PIMPL idiom)
    //==============================================================================

    class ControllerImpl
    {
    public:
        /**
         * @brief Construct a new Controller Implementation object
         */
        ControllerImpl(const ControllerConfig &config)
            : m_config(config),
              m_mode(ControllerMode::IDLE),
              m_cpgNetwork(nullptr),
              m_currentCommand(),
              m_targetCommand(),
              m_lastUpdateTime(std::chrono::high_resolution_clock::now()),
              m_totalEnergy(0.0),
              m_stepCount(0),
              m_debugMode(false),
              m_lastErrorMessage("")
        {
            // Initialize joint arrays
            m_jointAngles.resize(TOTAL_JOINTS, 0.0);
            m_jointVelocities.resize(TOTAL_JOINTS, 0.0);
            m_legContacts.resize(HEXAPOD_LEGS, false);
            m_gaitProgress.resize(HEXAPOD_LEGS, 0.0);
        }

        /**
         * @brief Initialize with CPG network
         */
        bool initialize(const NetworkParams &network_params, const GaitParams &gait_params)
        {
            try
            {
                // Create CPG network
                m_cpgNetwork = std::make_unique<Network>(network_params, gait_params);

                // Configure for hexapod locomotion
                if (!m_cpgNetwork->configureForHexapod("tripod"))
                {
                    setLastError("Failed to configure CPG network for hexapod");
                    return false;
                }

                // Set initial state
                m_mode = ControllerMode::IDLE;
                m_currentCommand = LocomotionCommand(); // Default stationary command
                m_targetCommand = m_currentCommand;

                clearLastError();
                return true;
            }
            catch (const std::exception &e)
            {
                setLastError("Exception during initialization: " + std::string(e.what()));
                return false;
            }
        }

        /**
         * @brief Start locomotion with given command
         */
        bool startLocomotion(const LocomotionCommand &command)
        {
            if (m_mode == ControllerMode::ERROR)
            {
                setLastError("Controller in error state");
                return false;
            }

            if (!controller_utils::validateCommand(command, m_config))
            {
                setLastError("Invalid locomotion command");
                return false;
            }

            // Handle emergency stop
            if (command.emergency_stop)
            {
                return emergencyStop();
            }

            // Store command and start transition
            m_targetCommand = command;

            if (m_mode == ControllerMode::IDLE)
            {
                m_mode = ControllerMode::INITIALIZING;

                // Switch to requested gait if needed
                if (command.gait_type != "tripod")
                {
                    if (!m_cpgNetwork->switchGait(command.gait_type))
                    {
                        setLastError("Failed to switch to gait: " + command.gait_type);
                        return false;
                    }
                }

                // Update network parameters
                updateNetworkFromCommand(command);

                m_mode = ControllerMode::WALKING;
            }
            else
            {
                // Smooth transition to new command
                m_mode = ControllerMode::TRANSITIONING;
            }

            // Trigger callbacks
            triggerStateChangeCallback();

            return true;
        }

        /**
         * @brief Update controller by one time step
         */
        bool update(double dt)
        {
            if (!m_cpgNetwork)
            {
                setLastError("CPG network not initialized");
                return false;
            }

            auto currentTime = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration<double>(currentTime - m_lastUpdateTime).count();
            m_lastUpdateTime = currentTime;

            // Handle state machine
            switch (m_mode)
            {
            case ControllerMode::IDLE:
                handleIdleMode();
                break;

            case ControllerMode::INITIALIZING:
                handleInitializingMode();
                break;

            case ControllerMode::WALKING:
                handleWalkingMode(dt);
                break;

            case ControllerMode::TRANSITIONING:
                handleTransitioningMode(dt);
                break;

            case ControllerMode::STOPPING:
                handleStoppingMode(dt);
                break;

            case ControllerMode::EMERGENCY:
                handleEmergencyMode(dt);
                break;

            case ControllerMode::ERROR:
                return false;
            }

            // Update CPG network
            if (!m_cpgNetwork->update(dt))
            {
                setLastError("CPG network update failed");
                m_mode = ControllerMode::ERROR;
                return false;
            }

            // Generate joint commands
            generateJointCommands();

            // Update performance metrics
            updatePerformanceMetrics(elapsed);

            return true;
        }

        /**
         * @brief Handle idle mode
         */
        void handleIdleMode()
        {
            // Keep joints at neutral positions
            std::fill(m_jointAngles.begin(), m_jointAngles.end(), 0.0);
            std::fill(m_jointVelocities.begin(), m_jointVelocities.end(), 0.0);
            std::fill(m_legContacts.begin(), m_legContacts.end(), true); // All legs on ground
        }

        /**
         * @brief Handle initializing mode
         */
        void handleInitializingMode()
        {
            // Wait for network to stabilize
            if (m_cpgNetwork->isStable())
            {
                m_mode = ControllerMode::WALKING;
                triggerStateChangeCallback();
            }
        }

        /**
         * @brief Handle walking mode
         */
        void handleWalkingMode(double dt)
        {
            // Smooth interpolation towards target command
            interpolateToTargetCommand(dt);

            // Check if we need to stop
            if (isStationaryCommand(m_targetCommand))
            {
                m_mode = ControllerMode::STOPPING;
                triggerStateChangeCallback();
            }
        }

        /**
         * @brief Handle transitioning mode
         */
        void handleTransitioningMode(double dt)
        {
            // Continue interpolating to target
            interpolateToTargetCommand(dt);

            // Check if transition complete
            if (commandsEqual(m_currentCommand, m_targetCommand, 0.01))
            {
                m_mode = ControllerMode::WALKING;
                triggerStateChangeCallback();
            }
        }

        /**
         * @brief Handle stopping mode
         */
        void handleStoppingMode(double dt)
        {
            // Gradually reduce velocities
            double stop_rate = 2.0; // Reduce by 2.0 m/s per second

            m_currentCommand.linear_velocity = approachZero(m_currentCommand.linear_velocity, stop_rate * dt);
            m_currentCommand.angular_velocity = approachZero(m_currentCommand.angular_velocity, stop_rate * dt);
            m_currentCommand.lateral_velocity = approachZero(m_currentCommand.lateral_velocity, stop_rate * dt);

            // Update network
            updateNetworkFromCommand(m_currentCommand);

            // Check if stopped
            if (std::abs(m_currentCommand.linear_velocity) < 0.01 &&
                std::abs(m_currentCommand.angular_velocity) < 0.01 &&
                std::abs(m_currentCommand.lateral_velocity) < 0.01)
            {
                m_mode = ControllerMode::IDLE;
                triggerStateChangeCallback();
            }
        }

        /**
         * @brief Handle emergency mode
         */
        void handleEmergencyMode(double dt)
        {
            // Force all velocities to zero immediately
            m_currentCommand.linear_velocity = 0.0;
            m_currentCommand.angular_velocity = 0.0;
            m_currentCommand.lateral_velocity = 0.0;

            // Set emergency frequency (stop pattern)
            m_cpgNetwork->setGlobalFrequency(0.0);

            // Transition to idle after emergency stop time
            static double emergency_timer = 0.0;
            emergency_timer += dt;

            if (emergency_timer >= EMERGENCY_STOP_TIME)
            {
                emergency_timer = 0.0;
                m_mode = ControllerMode::IDLE;
                triggerStateChangeCallback();
            }
        }

        /**
         * @brief Generate joint commands from CPG output
         */
        void generateJointCommands()
        {
            if (!m_cpgNetwork)
                return;

            auto network_output = m_cpgNetwork->getNetworkOutput();

            // Map CPG outputs to joint angles
            for (size_t leg = 0; leg < HEXAPOD_LEGS; ++leg)
            {
                for (size_t joint = 0; joint < JOINTS_PER_LEG; ++joint)
                {
                    size_t joint_index = leg * JOINTS_PER_LEG + joint;

                    if (joint_index < network_output.joint_positions.size())
                    {
                        // Apply joint-specific scaling and offsets
                        double cpg_output = network_output.joint_positions[joint_index];
                        double joint_angle = mapCpgToJoint(cpg_output, joint);

                        // Apply limits
                        joint_angle = std::clamp(joint_angle, -MAX_JOINT_ANGLE, MAX_JOINT_ANGLE);

                        m_jointAngles[joint_index] = joint_angle;
                    }

                    if (joint_index < network_output.joint_velocities.size())
                    {
                        double joint_velocity = network_output.joint_velocities[joint_index];
                        joint_velocity = std::clamp(joint_velocity, -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY);

                        m_jointVelocities[joint_index] = joint_velocity;
                    }
                }

                // Update leg contact and gait progress
                if (leg < network_output.stance_phases.size())
                {
                    m_legContacts[leg] = network_output.stance_phases[leg];
                }

                if (leg < network_output.gait_progress.size())
                {
                    m_gaitProgress[leg] = network_output.gait_progress[leg];
                }
            }
        }

        /**
         * @brief Map CPG output to specific joint angle
         */
        double mapCpgToJoint(double cpg_output, size_t joint_id) const
        {
            // Joint-specific mapping
            switch (joint_id)
            {
            case 0:                      // Coxa (hip rotation)
                return cpg_output * 0.5; // ±30 degrees

            case 1: // Femur (hip pitch)
            {
                double base_angle = -0.3; // -17 degrees baseline
                double range = 0.8;       // ±46 degrees range
                return base_angle + cpg_output * range;
            }

            case 2: // Tibia (knee)
            {
                double base_angle = 0.6; // 34 degrees baseline
                double range = 1.0;      // ±57 degrees range
                return base_angle + cpg_output * range;
            }

            default:
                return 0.0;
            }
        }

        /**
         * @brief Update network parameters from command
         */
        void updateNetworkFromCommand(const LocomotionCommand &command)
        {
            if (!m_cpgNetwork)
                return;

            // Calculate frequency from velocity
            double frequency = controller_utils::velocityToFrequency(
                std::abs(command.linear_velocity), DEFAULT_STEP_LENGTH);

            // Apply constraints
            frequency = std::clamp(frequency, m_config.min_step_frequency, m_config.max_step_frequency);

            // Set global frequency
            m_cpgNetwork->setGlobalFrequency(frequency);

            // Update gait parameters
            GaitParams gait_params = m_cpgNetwork->getGaitParams();
            gait_params.step_frequency = frequency;
            gait_params.stance_duration = command.duty_factor;
            gait_params.swing_duration = 1.0 - command.duty_factor;
            gait_params.step_height = DEFAULT_BODY_HEIGHT * command.step_height;

            m_cpgNetwork->setGaitParams(gait_params);
        }

        /**
         * @brief Interpolate current command towards target
         */
        void interpolateToTargetCommand(double dt)
        {
            double interpolation_rate = 2.0; // Commands per second
            double alpha = std::min(1.0, interpolation_rate * dt);

            m_currentCommand.linear_velocity = lerp(m_currentCommand.linear_velocity,
                                                    m_targetCommand.linear_velocity, alpha);
            m_currentCommand.angular_velocity = lerp(m_currentCommand.angular_velocity,
                                                     m_targetCommand.angular_velocity, alpha);
            m_currentCommand.lateral_velocity = lerp(m_currentCommand.lateral_velocity,
                                                     m_targetCommand.lateral_velocity, alpha);
            m_currentCommand.step_height = lerp(m_currentCommand.step_height,
                                                m_targetCommand.step_height, alpha);
            m_currentCommand.duty_factor = lerp(m_currentCommand.duty_factor,
                                                m_targetCommand.duty_factor, alpha);

            // Update network with interpolated command
            updateNetworkFromCommand(m_currentCommand);
        }

        /**
         * @brief Emergency stop
         */
        bool emergencyStop()
        {
            m_mode = ControllerMode::EMERGENCY;

            // Clear all velocities immediately
            m_currentCommand.linear_velocity = 0.0;
            m_currentCommand.angular_velocity = 0.0;
            m_currentCommand.lateral_velocity = 0.0;
            m_targetCommand = m_currentCommand;

            // Trigger emergency callback
            if (m_emergencyCallback)
            {
                m_emergencyCallback("Emergency stop initiated");
            }

            triggerStateChangeCallback();
            return true;
        }

        /**
         * @brief Update performance metrics
         */
        void updatePerformanceMetrics(double dt)
        {
            // Estimate energy consumption
            double power = controller_utils::estimateEnergyConsumption(m_jointVelocities,
                                                                       std::vector<double>(TOTAL_JOINTS, 1.0));
            m_totalEnergy += power * dt;

            // Count steps
            static bool last_contact_state = false;
            bool any_leg_contact = std::any_of(m_legContacts.begin(), m_legContacts.end(),
                                               [](bool contact)
                                               { return contact; });

            if (!last_contact_state && any_leg_contact)
            {
                m_stepCount++;
            }
            last_contact_state = any_leg_contact;
        }

        /**
         * @brief Generate controller state
         */
        ControllerState generateControllerState() const
        {
            ControllerState state;

            state.is_active = (m_mode != ControllerMode::IDLE && m_mode != ControllerMode::ERROR);
            state.is_walking = (m_mode == ControllerMode::WALKING || m_mode == ControllerMode::TRANSITIONING);
            state.current_gait = m_cpgNetwork ? m_currentCommand.gait_type : "unknown";
            state.current_frequency = m_cpgNetwork ? m_cpgNetwork->getGlobalFrequency() : 0.0;

            state.actual_linear_velocity = m_currentCommand.linear_velocity;
            state.actual_angular_velocity = m_currentCommand.angular_velocity;

            state.joint_angles = m_jointAngles;
            state.leg_contacts = m_legContacts;

            // Calculate stability margin (simplified)
            state.stability_margin = calculateStabilityMargin();

            state.energy_consumption = m_totalEnergy;

            return state;
        }

        /**
         * @brief Calculate stability margin
         */
        double calculateStabilityMargin() const
        {
            // Count legs in stance
            size_t stance_legs = std::count(m_legContacts.begin(), m_legContacts.end(), true);

            // Simple stability metric based on number of supporting legs
            if (stance_legs >= 3)
            {
                return 1.0; // Statically stable
            }
            else if (stance_legs == 2)
            {
                return 0.5; // Dynamically stable
            }
            else
            {
                return 0.1; // Unstable
            }
        }

        /**
         * @brief Linear interpolation
         */
        double lerp(double a, double b, double t) const
        {
            return a + t * (b - a);
        }

        /**
         * @brief Approach zero with given rate
         */
        double approachZero(double value, double rate) const
        {
            if (std::abs(value) <= rate)
            {
                return 0.0;
            }

            return value > 0.0 ? value - rate : value + rate;
        }

        /**
         * @brief Check if command is stationary
         */
        bool isStationaryCommand(const LocomotionCommand &command) const
        {
            return std::abs(command.linear_velocity) < 0.01 &&
                   std::abs(command.angular_velocity) < 0.01 &&
                   std::abs(command.lateral_velocity) < 0.01;
        }

        /**
         * @brief Check if two commands are equal within tolerance
         */
        bool commandsEqual(const LocomotionCommand &a, const LocomotionCommand &b, double tolerance) const
        {
            return std::abs(a.linear_velocity - b.linear_velocity) < tolerance &&
                   std::abs(a.angular_velocity - b.angular_velocity) < tolerance &&
                   std::abs(a.lateral_velocity - b.lateral_velocity) < tolerance &&
                   std::abs(a.step_height - b.step_height) < tolerance &&
                   std::abs(a.duty_factor - b.duty_factor) < tolerance;
        }

        /**
         * @brief Trigger state change callback
         */
        void triggerStateChangeCallback()
        {
            if (m_stateChangeCallback)
            {
                ControllerState state = generateControllerState();
                m_stateChangeCallback(state);
            }
        }

        /**
         * @brief Set error message
         */
        void setLastError(const std::string &message)
        {
            m_lastErrorMessage = message;
            if (m_debugMode)
            {
                std::cerr << "Controller Error: " << message << std::endl;
            }
        }

        /**
         * @brief Clear error message
         */
        void clearLastError()
        {
            m_lastErrorMessage.clear();
        }

        // Member variables
        ControllerConfig m_config;
        ControllerMode m_mode;
        std::unique_ptr<Network> m_cpgNetwork;
        LocomotionCommand m_currentCommand;
        LocomotionCommand m_targetCommand;

        // State vectors
        std::vector<double> m_jointAngles;
        std::vector<double> m_jointVelocities;
        std::vector<bool> m_legContacts;
        std::vector<double> m_gaitProgress;

        // Timing and performance
        std::chrono::high_resolution_clock::time_point m_lastUpdateTime;
        double m_totalEnergy;
        size_t m_stepCount;

        // Debug and callbacks
        bool m_debugMode;
        std::string m_lastErrorMessage;
        std::function<void(const ControllerState &)> m_stateChangeCallback;
        std::function<void(const std::string &, const std::string &)> m_gaitTransitionCallback;
        std::function<void(const std::string &)> m_emergencyCallback;
    };

    //==============================================================================
    // Main Controller Class Implementation
    //==============================================================================

    Controller::Controller(const ControllerConfig &config)
        : pImpl(std::make_unique<ControllerImpl>(config))
    {
    }

    Controller::~Controller() = default;

    Controller::Controller(Controller &&other) noexcept
        : pImpl(std::move(other.pImpl))
    {
    }

    Controller &Controller::operator=(Controller &&other) noexcept
    {
        if (this != &other)
        {
            pImpl = std::move(other.pImpl);
        }
        return *this;
    }

    //--------------------------------------------------------------------------
    // Initialization and Configuration
    //--------------------------------------------------------------------------

    bool Controller::initialize(const NetworkParams &network_params, const GaitParams &gait_params)
    {
        return pImpl->initialize(network_params, gait_params);
    }

    bool Controller::configure(const ControllerConfig &config)
    {
        pImpl->m_config = config;
        return true;
    }

    ControllerConfig Controller::getConfiguration() const
    {
        return pImpl->m_config;
    }

    void Controller::reset(bool stop_motion)
    {
        if (stop_motion)
        {
            pImpl->emergencyStop();
        }

        pImpl->m_mode = ControllerMode::IDLE;
        pImpl->m_currentCommand = LocomotionCommand();
        pImpl->m_targetCommand = LocomotionCommand();
        pImpl->m_totalEnergy = 0.0;
        pImpl->m_stepCount = 0;

        if (pImpl->m_cpgNetwork)
        {
            pImpl->m_cpgNetwork->reset(false);
        }

        pImpl->clearLastError();
    }

    //--------------------------------------------------------------------------
    // Control Interface
    //--------------------------------------------------------------------------

    bool Controller::startLocomotion(const LocomotionCommand &command)
    {
        return pImpl->startLocomotion(command);
    }

    bool Controller::updateCommand(const LocomotionCommand &command)
    {
        if (!controller_utils::validateCommand(command, pImpl->m_config))
        {
            pImpl->setLastError("Invalid locomotion command for update");
            return false;
        }

        pImpl->m_targetCommand = command;

        if (pImpl->m_mode == ControllerMode::IDLE)
        {
            return startLocomotion(command);
        }
        else if (pImpl->m_mode == ControllerMode::WALKING)
        {
            pImpl->m_mode = ControllerMode::TRANSITIONING;
            pImpl->triggerStateChangeCallback();
        }

        return true;
    }

    bool Controller::stopLocomotion(bool emergency)
    {
        if (emergency)
        {
            return pImpl->emergencyStop();
        }
        else
        {
            pImpl->m_targetCommand = LocomotionCommand(); // Stationary command
            if (pImpl->m_mode == ControllerMode::WALKING || pImpl->m_mode == ControllerMode::TRANSITIONING)
            {
                pImpl->m_mode = ControllerMode::STOPPING;
                pImpl->triggerStateChangeCallback();
            }
            return true;
        }
    }

    //--------------------------------------------------------------------------
    // Gait Control
    //--------------------------------------------------------------------------

    bool Controller::switchGait(const std::string &gait_type)
    {
        if (!pImpl->m_cpgNetwork)
        {
            pImpl->setLastError("CPG network not initialized");
            return false;
        }

        bool success = pImpl->m_cpgNetwork->switchGait(gait_type);

        if (success && pImpl->m_gaitTransitionCallback)
        {
            std::string current_gait = getCurrentGait();
            pImpl->m_gaitTransitionCallback(current_gait, gait_type);
        }

        return success;
    }

    bool Controller::setStepFrequency(double frequency)
    {
        if (frequency < pImpl->m_config.min_step_frequency || frequency > pImpl->m_config.max_step_frequency)
        {
            pImpl->setLastError("Step frequency out of range");
            return false;
        }

        if (pImpl->m_cpgNetwork)
        {
            return pImpl->m_cpgNetwork->setGlobalFrequency(frequency);
        }

        return false;
    }

    double Controller::getStepFrequency() const
    {
        if (pImpl->m_cpgNetwork)
        {
            return pImpl->m_cpgNetwork->getGlobalFrequency();
        }
        return 0.0;
    }

    bool Controller::setDutyFactor(double duty_factor)
    {
        if (duty_factor < 0.3 || duty_factor > 0.8)
        {
            pImpl->setLastError("Duty factor out of range");
            return false;
        }

        pImpl->m_currentCommand.duty_factor = duty_factor;
        pImpl->m_targetCommand.duty_factor = duty_factor;

        return true;
    }

    double Controller::getDutyFactor() const
    {
        return pImpl->m_currentCommand.duty_factor;
    }

    //--------------------------------------------------------------------------
    // Velocity Control
    //--------------------------------------------------------------------------

    bool Controller::setLinearVelocity(double velocity)
    {
        if (std::abs(velocity) > pImpl->m_config.max_linear_velocity)
        {
            pImpl->setLastError("Linear velocity out of range");
            return false;
        }

        pImpl->m_targetCommand.linear_velocity = velocity;
        return true;
    }

    bool Controller::setAngularVelocity(double velocity)
    {
        if (std::abs(velocity) > pImpl->m_config.max_angular_velocity)
        {
            pImpl->setLastError("Angular velocity out of range");
            return false;
        }

        pImpl->m_targetCommand.angular_velocity = velocity;
        return true;
    }

    bool Controller::setLateralVelocity(double velocity)
    {
        if (std::abs(velocity) > pImpl->m_config.max_linear_velocity)
        {
            pImpl->setLastError("Lateral velocity out of range");
            return false;
        }

        pImpl->m_targetCommand.lateral_velocity = velocity;
        return true;
    }

    double Controller::getLinearVelocity() const
    {
        return pImpl->m_currentCommand.linear_velocity;
    }

    double Controller::getAngularVelocity() const
    {
        return pImpl->m_currentCommand.angular_velocity;
    }

    double Controller::getLateralVelocity() const
    {
        return pImpl->m_currentCommand.lateral_velocity;
    }

    //--------------------------------------------------------------------------
    // Update and Output
    //--------------------------------------------------------------------------

    bool Controller::update(double dt)
    {
        return pImpl->update(dt);
    }

    std::vector<double> Controller::getJointCommands() const
    {
        return pImpl->m_jointAngles;
    }

    std::vector<double> Controller::getJointVelocities() const
    {
        return pImpl->m_jointVelocities;
    }

    std::vector<bool> Controller::getStancePhases() const
    {
        return pImpl->m_legContacts;
    }

    std::vector<double> Controller::getGaitProgress() const
    {
        return pImpl->m_gaitProgress;
    }

    //--------------------------------------------------------------------------
    // State and Monitoring
    //--------------------------------------------------------------------------

    ControllerState Controller::getControllerState() const
    {
        return pImpl->generateControllerState();
    }

    bool Controller::isActive() const
    {
        return pImpl->m_mode != ControllerMode::IDLE && pImpl->m_mode != ControllerMode::ERROR;
    }

    bool Controller::isWalking() const
    {
        return pImpl->m_mode == ControllerMode::WALKING || pImpl->m_mode == ControllerMode::TRANSITIONING;
    }

    bool Controller::isTransitioning() const
    {
        return pImpl->m_mode == ControllerMode::TRANSITIONING;
    }

    std::string Controller::getCurrentGait() const
    {
        if (pImpl->m_cpgNetwork)
        {
            return pImpl->m_currentCommand.gait_type;
        }
        return "unknown";
    }

    //--------------------------------------------------------------------------
    // Adaptive Control (Simplified implementations)
    //--------------------------------------------------------------------------

    void Controller::setAdaptiveGait(bool enable)
    {
        pImpl->m_config.adaptive_gait = enable;
    }

    bool Controller::isAdaptiveGaitEnabled() const
    {
        return pImpl->m_config.adaptive_gait;
    }

    void Controller::updateTerrainFeedback(double roughness)
    {
        if (!pImpl)
            return;

        // Store terrain parameters for adaptive gait selection
        // Can be used to modify gait parameters based on terrain conditions
        if (pImpl->m_config.adaptive_gait && roughness > 0.5)
        {
            // Switch to more stable gait on rough terrain
            if (pImpl->m_currentCommand.gait_type == "tripod")
            {
                pImpl->m_currentCommand.gait_type = "wave"; // More stable
                pImpl->m_currentCommand.duty_factor = 0.7;  // Longer stance phase
            }
        }
    }

    void Controller::updateBalanceFeedback(double roll, double pitch, const std::vector<double> &angular_velocity)
    {
        if (!pImpl || !pImpl->m_config.balance_control)
            return;

        // Convert roll/pitch from radians to degrees for easier threshold checking
        double roll_deg = roll * 180.0 / M_PI;
        double pitch_deg = pitch * 180.0 / M_PI;

        // Balance thresholds
        const double tilt_threshold = 15.0; // degrees
        const double critical_tilt = 30.0;  // degrees

        // Check for critical tilt - emergency stop
        if (std::abs(roll_deg) > critical_tilt || std::abs(pitch_deg) > critical_tilt)
        {
            pImpl->emergencyStop();
            pImpl->setLastError("Critical tilt detected - emergency stop activated");
            return;
        }

        // Apply balance corrections for moderate tilt
        if (std::abs(roll_deg) > tilt_threshold || std::abs(pitch_deg) > tilt_threshold)
        {
            // Reduce velocity when tilted
            double tilt_factor = std::max(0.3, 1.0 - (std::max(std::abs(roll_deg), std::abs(pitch_deg)) / critical_tilt));

            if (isWalking())
            {
                pImpl->m_currentCommand.linear_velocity *= tilt_factor;
                pImpl->m_currentCommand.angular_velocity *= tilt_factor;

                // Adjust step height and duty factor for stability
                pImpl->m_currentCommand.step_height = std::max(0.5, pImpl->m_currentCommand.step_height * 0.8);
                pImpl->m_currentCommand.duty_factor = std::min(0.8, pImpl->m_currentCommand.duty_factor + 0.1);
            }
        }

        // Update stability margin based on IMU data
        double angular_vel_magnitude = 0.0;
        for (double vel : angular_velocity)
        {
            angular_vel_magnitude += vel * vel;
        }
        angular_vel_magnitude = std::sqrt(angular_vel_magnitude);

        // Update performance metrics
        pImpl->updatePerformanceMetrics(0.02); // Assume 50Hz update rate
    }

    //--------------------------------------------------------------------------
    // Performance Analysis
    //--------------------------------------------------------------------------

    std::string Controller::getPerformanceMetrics() const
    {
        std::ostringstream oss;
        oss << "=== CPG Controller Performance ===\n";
        oss << "Mode: " << static_cast<int>(pImpl->m_mode) << "\n";
        oss << "Total Energy: " << std::fixed << std::setprecision(2) << pImpl->m_totalEnergy << " J\n";
        oss << "Step Count: " << pImpl->m_stepCount << "\n";
        oss << "Current Frequency: " << getStepFrequency() << " Hz\n";
        oss << "Linear Velocity: " << getLinearVelocity() << " m/s\n";
        oss << "Angular Velocity: " << getAngularVelocity() << " rad/s\n";
        oss << "Stability Margin: " << getStabilityMargin();
        return oss.str();
    }

    double Controller::getEnergyConsumption() const
    {
        return pImpl->m_totalEnergy;
    }

    double Controller::getStabilityMargin() const
    {
        return pImpl->calculateStabilityMargin();
    }

    double Controller::getLocomotionEfficiency() const
    {
        // Simplified efficiency calculation
        double velocity = std::abs(getLinearVelocity());
        double frequency = getStepFrequency();

        if (frequency > 0.0)
        {
            return std::min(1.0, velocity / (frequency * DEFAULT_STEP_LENGTH));
        }

        return 0.0;
    }

    //--------------------------------------------------------------------------
    // Error Handling and Diagnostics
    //--------------------------------------------------------------------------

    bool Controller::validateState() const
    {
        if (!pImpl->m_cpgNetwork)
        {
            return false;
        }

        return pImpl->m_cpgNetwork->validateConfiguration();
    }

    std::string Controller::getLastErrorMessage() const
    {
        return pImpl->m_lastErrorMessage;
    }

    void Controller::clearLastError()
    {
        pImpl->clearLastError();
    }

    std::string Controller::getDiagnostics() const
    {
        std::ostringstream oss;
        oss << getPerformanceMetrics() << "\n\n";

        if (pImpl->m_cpgNetwork)
        {
            oss << pImpl->m_cpgNetwork->getNetworkStatistics();
        }

        return oss.str();
    }

    //--------------------------------------------------------------------------
    // Callbacks and Events
    //--------------------------------------------------------------------------

    void Controller::setStateChangeCallback(std::function<void(const ControllerState &)> callback)
    {
        pImpl->m_stateChangeCallback = callback;
    }

    void Controller::setGaitTransitionCallback(std::function<void(const std::string &, const std::string &)> callback)
    {
        pImpl->m_gaitTransitionCallback = callback;
    }

    void Controller::setEmergencyCallback(std::function<void(const std::string &)> callback)
    {
        pImpl->m_emergencyCallback = callback;
    }

    void Controller::clearCallbacks()
    {
        pImpl->m_stateChangeCallback = nullptr;
        pImpl->m_gaitTransitionCallback = nullptr;
        pImpl->m_emergencyCallback = nullptr;
    }

    //--------------------------------------------------------------------------
    // Debugging and Visualization
    //--------------------------------------------------------------------------

    void Controller::setDebugMode(bool enable)
    {
        pImpl->m_debugMode = enable;
        if (pImpl->m_cpgNetwork)
        {
            pImpl->m_cpgNetwork->setDebugMode(enable);
        }
    }

    bool Controller::isDebugModeEnabled() const
    {
        return pImpl->m_debugMode;
    }

    //==============================================================================
    // Utility Functions Implementation
    //==============================================================================

    namespace controller_utils
    {
        LocomotionCommand createWalkCommand(double linear_vel, double angular_vel, const std::string &gait_type)
        {
            LocomotionCommand command;
            command.linear_velocity = linear_vel;
            command.angular_velocity = angular_vel;
            command.gait_type = gait_type;
            return command;
        }

        LocomotionCommand createStopCommand()
        {
            return LocomotionCommand(); // Default constructor creates stationary command
        }

        bool validateCommand(const LocomotionCommand &command, const ControllerConfig &config)
        {
            if (std::abs(command.linear_velocity) > config.max_linear_velocity)
                return false;
            if (std::abs(command.angular_velocity) > config.max_angular_velocity)
                return false;
            if (command.step_height < 0.5 || command.step_height > 2.0)
                return false;
            if (command.duty_factor < 0.3 || command.duty_factor > 0.8)
                return false;

            return true;
        }

        double velocityToFrequency(double velocity, double step_length)
        {
            if (step_length <= 0.0)
                return 0.0;
            return std::abs(velocity) / step_length;
        }

        double estimateEnergyConsumption(const std::vector<double> &joint_velocities,
                                         const std::vector<double> &joint_torques)
        {
            double power = 0.0;

            for (size_t i = 0; i < std::min(joint_velocities.size(), joint_torques.size()); ++i)
            {
                power += std::abs(joint_velocities[i] * joint_torques[i]);
            }

            return power;
        }

        double computeStabilityMargin(const std::vector<bool> &stance_legs)
        {
            // Simplified stability calculation
            size_t supporting_legs = std::count(stance_legs.begin(), stance_legs.end(), true);

            if (supporting_legs >= 3)
                return 1.0; // Statically stable
            else if (supporting_legs == 2)
                return 0.5; // Dynamically stable
            else
                return 0.1; // Unstable
        }

        std::string selectOptimalGait(double velocity, double terrain_roughness, double stability_requirement)
        {
            if (stability_requirement > 0.8 || terrain_roughness > 0.6)
            {
                return "wave"; // Most stable
            }
            else if (velocity > 0.2)
            {
                return "tripod"; // Fastest
            }
            else
            {
                return "ripple"; // Balanced
            }
        }
    }

} // namespace cpg
