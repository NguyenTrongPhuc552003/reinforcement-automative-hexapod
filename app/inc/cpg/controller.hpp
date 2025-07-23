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

#ifndef CPG_CONTROLLER_HPP
#define CPG_CONTROLLER_HPP

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include "cpg/network.hpp"
#include "cpg/parameters.hpp"

/**
 * @brief Central Pattern Generator (CPG) controller implementation
 *
 * This namespace contains classes and functions for high-level control of the
 * CPG system, providing interfaces for locomotion control and gait transitions.
 */
namespace cpg
{

    //==============================================================================
    // Control Commands and State
    //==============================================================================

    /**
     * @brief Locomotion command structure
     *
     * Represents high-level locomotion commands that the controller can execute.
     */
    struct LocomotionCommand
    {
        double linear_velocity;  ///< Forward/backward velocity (m/s)
        double angular_velocity; ///< Rotational velocity (rad/s)
        double lateral_velocity; ///< Left/right velocity (m/s)
        std::string gait_type;   ///< Desired gait pattern
        double step_height;      ///< Step height scaling [0.5, 2.0]
        double duty_factor;      ///< Stance phase ratio [0.3, 0.8]
        bool emergency_stop;     ///< Emergency stop flag

        /**
         * @brief Default constructor (stationary)
         */
        LocomotionCommand()
            : linear_velocity(0.0),
              angular_velocity(0.0),
              lateral_velocity(0.0),
              gait_type("tripod"),
              step_height(1.0),
              duty_factor(0.5),
              emergency_stop(false)
        {
        }

        /**
         * @brief Parameterized constructor
         */
        LocomotionCommand(double linear_vel, double angular_vel,
                          const std::string &gait = "tripod")
            : linear_velocity(linear_vel),
              angular_velocity(angular_vel),
              lateral_velocity(0.0),
              gait_type(gait),
              step_height(1.0),
              duty_factor(0.5),
              emergency_stop(false)
        {
        }
    };

    /**
     * @brief Controller state information
     *
     * Contains current state and status of the CPG controller.
     */
    struct ControllerState
    {
        bool is_active;                   ///< Whether controller is active
        bool is_walking;                  ///< Whether currently walking
        std::string current_gait;         ///< Current gait pattern
        double current_frequency;         ///< Current step frequency (Hz)
        double actual_linear_velocity;    ///< Actual linear velocity (m/s)
        double actual_angular_velocity;   ///< Actual angular velocity (rad/s)
        std::vector<double> joint_angles; ///< Current joint angles (radians)
        std::vector<bool> leg_contacts;   ///< Ground contact for each leg
        double stability_margin;          ///< Current stability margin
        double energy_consumption;        ///< Estimated energy consumption

        /**
         * @brief Default constructor
         */
        ControllerState()
            : is_active(false),
              is_walking(false),
              current_gait("tripod"),
              current_frequency(0.0),
              actual_linear_velocity(0.0),
              actual_angular_velocity(0.0),
              stability_margin(0.0),
              energy_consumption(0.0)
        {
        }
    };

    /**
     * @brief Controller configuration parameters
     */
    struct ControllerConfig
    {
        double max_linear_velocity;  ///< Maximum linear velocity (m/s)
        double max_angular_velocity; ///< Maximum angular velocity (rad/s)
        double max_step_frequency;   ///< Maximum step frequency (Hz)
        double min_step_frequency;   ///< Minimum step frequency (Hz)
        double transition_time;      ///< Gait transition time (seconds)
        double stability_threshold;  ///< Minimum stability margin
        bool adaptive_gait;          ///< Enable adaptive gait selection
        bool balance_control;        ///< Enable balance feedback control

        /**
         * @brief Default constructor with conservative values
         */
        ControllerConfig()
            : max_linear_velocity(0.3),  // 30 cm/s
              max_angular_velocity(1.0), // 1 rad/s
              max_step_frequency(2.0),   // 2 Hz
              min_step_frequency(0.5),   // 0.5 Hz
              transition_time(2.0),      // 2 seconds
              stability_threshold(0.1),  // 10% margin
              adaptive_gait(true),
              balance_control(true)
        {
        }
    };

    //==============================================================================
    // Forward declarations
    //==============================================================================

    // Implementation class (PIMPL idiom)
    class ControllerImpl;

    //==============================================================================
    // Main Controller Class
    //==============================================================================

    /**
     * @brief CPG-based hexapod locomotion controller
     *
     * High-level controller that manages CPG networks to generate coordinated
     * locomotion patterns for the hexapod robot, including gait transitions
     * and velocity control.
     */
    class Controller
    {
    public:
        /**
         * @brief Construct a new Controller object
         *
         * @param config Controller configuration parameters
         */
        Controller(const ControllerConfig &config = ControllerConfig());

        /**
         * @brief Destroy the Controller object
         */
        ~Controller();

        // Non-copyable
        Controller(const Controller &) = delete;
        Controller &operator=(const Controller &) = delete;

        // Move semantics
        Controller(Controller &&other) noexcept;
        Controller &operator=(Controller &&other) noexcept;

        //--------------------------------------------------------------------------
        // Initialization and Configuration
        //--------------------------------------------------------------------------

        /**
         * @brief Initialize controller with CPG network
         *
         * @param network_params Parameters for the CPG network
         * @param gait_params Initial gait parameters
         * @return true if initialization successful
         * @return false if initialization failed
         */
        bool initialize(const NetworkParams &network_params = NetworkParams(),
                        const GaitParams &gait_params = GaitParams());

        /**
         * @brief Configure controller parameters
         *
         * @param config New controller configuration
         * @return true if configuration successful
         * @return false if configuration invalid
         */
        bool configure(const ControllerConfig &config);

        /**
         * @brief Get current controller configuration
         *
         * @return ControllerConfig Current configuration
         */
        ControllerConfig getConfiguration() const;

        /**
         * @brief Reset controller to initial state
         *
         * @param stop_motion If true, stop all motion immediately
         */
        void reset(bool stop_motion = true);

        //--------------------------------------------------------------------------
        // Control Interface
        //--------------------------------------------------------------------------

        /**
         * @brief Start locomotion with given command
         *
         * @param command Locomotion command to execute
         * @return true if command accepted and started
         * @return false if command invalid or controller not ready
         */
        bool startLocomotion(const LocomotionCommand &command);

        /**
         * @brief Update locomotion command
         *
         * @param command New locomotion command
         * @return true if command updated successfully
         * @return false if command invalid
         */
        bool updateCommand(const LocomotionCommand &command);

        /**
         * @brief Stop all locomotion
         *
         * @param emergency If true, stop immediately; if false, gradual stop
         * @return true if stop initiated successfully
         * @return false if stop failed
         */
        bool stopLocomotion(bool emergency = false);

        //--------------------------------------------------------------------------
        // Gait Control
        //--------------------------------------------------------------------------

        /**
         * @brief Switch to different gait pattern
         *
         * @param gait_type Target gait pattern ("tripod", "wave", "ripple")
         * @param transition_time Time for smooth transition (seconds)
         * @return true if gait switch initiated
         * @return false if gait type invalid
         */
        bool switchGait(const std::string &gait_type);

        /**
         * @brief Set step frequency
         *
         * @param frequency Step frequency in Hz
         * @return true if frequency set successfully
         * @return false if frequency out of range
         */
        bool setStepFrequency(double frequency);

        /**
         * @brief Get current step frequency
         *
         * @return double Current step frequency in Hz
         */
        double getStepFrequency() const;

        /**
         * @brief Set duty factor (stance phase ratio)
         *
         * @param duty_factor Stance phase ratio [0.3, 0.8]
         * @return true if duty factor set successfully
         * @return false if duty factor out of range
         */
        bool setDutyFactor(double duty_factor);

        /**
         * @brief Get current duty factor
         *
         * @return double Current duty factor
         */
        double getDutyFactor() const;

        //--------------------------------------------------------------------------
        // Velocity Control
        //--------------------------------------------------------------------------

        /**
         * @brief Set linear velocity
         *
         * @param velocity Linear velocity in m/s (positive = forward)
         * @return true if velocity set successfully
         * @return false if velocity out of range
         */
        bool setLinearVelocity(double velocity);

        /**
         * @brief Set angular velocity
         *
         * @param velocity Angular velocity in rad/s (positive = CCW)
         * @return true if velocity set successfully
         * @return false if velocity out of range
         */
        bool setAngularVelocity(double velocity);

        /**
         * @brief Set lateral velocity
         *
         * @param velocity Lateral velocity in m/s (positive = right)
         * @return true if velocity set successfully
         * @return false if velocity out of range
         */
        bool setLateralVelocity(double velocity);

        /**
         * @brief Get current linear velocity
         *
         * @return double Current linear velocity in m/s
         */
        double getLinearVelocity() const;

        /**
         * @brief Get current angular velocity
         *
         * @return double Current angular velocity in rad/s
         */
        double getAngularVelocity() const;

        /**
         * @brief Get current lateral velocity
         *
         * @return double Current lateral velocity in m/s
         */
        double getLateralVelocity() const;

        //--------------------------------------------------------------------------
        // Update and Output
        //--------------------------------------------------------------------------

        /**
         * @brief Update controller by one time step
         *
         * Performs CPG network integration and generates joint commands.
         *
         * @param dt Time step size (seconds)
         * @return true if update successful
         * @return false if update failed
         */
        bool update(double dt);

        /**
         * @brief Get joint angle commands
         *
         * @return std::vector<double> Joint angles in radians (18 values for hexapod)
         */
        std::vector<double> getJointCommands() const;

        /**
         * @brief Get joint velocity commands
         *
         * @return std::vector<double> Joint velocities in rad/s
         */
        std::vector<double> getJointVelocities() const;

        /**
         * @brief Get stance phase status for all legs
         *
         * @return std::vector<bool> True if leg is in stance phase
         */
        std::vector<bool> getStancePhases() const;

        /**
         * @brief Get gait progress for all legs
         *
         * @return std::vector<double> Gait progress [0,1] for each leg
         */
        std::vector<double> getGaitProgress() const;

        //--------------------------------------------------------------------------
        // State and Monitoring
        //--------------------------------------------------------------------------

        /**
         * @brief Get current controller state
         *
         * @return ControllerState Complete controller state information
         */
        ControllerState getControllerState() const;

        /**
         * @brief Check if controller is active
         *
         * @return true if controller is running
         * @return false if controller is stopped
         */
        bool isActive() const;

        /**
         * @brief Check if robot is currently walking
         *
         * @return true if locomotion is active
         * @return false if stationary
         */
        bool isWalking() const;

        /**
         * @brief Check if gait transition is in progress
         *
         * @return true if transitioning between gaits
         * @return false if in stable gait
         */
        bool isTransitioning() const;

        /**
         * @brief Get current gait type
         *
         * @return std::string Current gait pattern name
         */
        std::string getCurrentGait() const;

        //--------------------------------------------------------------------------
        // Adaptive Control
        //--------------------------------------------------------------------------

        /**
         * @brief Enable/disable adaptive gait selection
         *
         * @param enable True to enable adaptive behavior
         */
        void setAdaptiveGait(bool enable);

        /**
         * @brief Check if adaptive gait is enabled
         *
         * @return true if adaptive gait is enabled
         */
        bool isAdaptiveGaitEnabled() const;

        /**
         * @brief Provide terrain feedback for adaptive control
         *
         * @param roughness Terrain roughness [0,1]
         * @param slope Terrain slope in radians
         * @param stability Current stability margin
         */
        void updateTerrainFeedback(double roughness);

        /**
         * @brief Provide balance feedback for control adjustment
         *
         * @param roll Roll angle in radians
         * @param pitch Pitch angle in radians
         * @param angular_velocity Body angular velocity
         */
        void updateBalanceFeedback(double roll, double pitch,
                                   const std::vector<double> &angular_velocity);

        //--------------------------------------------------------------------------
        // Performance Analysis
        //--------------------------------------------------------------------------

        /**
         * @brief Get controller performance metrics
         *
         * @return std::string Formatted performance information
         */
        std::string getPerformanceMetrics() const;

        /**
         * @brief Get energy consumption estimate
         *
         * @return double Estimated energy consumption (Watts)
         */
        double getEnergyConsumption() const;

        /**
         * @brief Get stability margin
         *
         * @return double Current stability margin [0,1]
         */
        double getStabilityMargin() const;

        /**
         * @brief Get locomotion efficiency
         *
         * @return double Locomotion efficiency [0,1]
         */
        double getLocomotionEfficiency() const;

        //--------------------------------------------------------------------------
        // Error Handling and Diagnostics
        //--------------------------------------------------------------------------

        /**
         * @brief Validate current controller state
         *
         * @return true if controller state is valid
         * @return false if errors detected
         */
        bool validateState() const;

        /**
         * @brief Get last error message
         *
         * @return std::string Error message
         */
        std::string getLastErrorMessage() const;

        /**
         * @brief Clear last error
         */
        void clearLastError();

        /**
         * @brief Get detailed diagnostic information
         *
         * @return std::string Diagnostic information
         */
        std::string getDiagnostics() const;

        //--------------------------------------------------------------------------
        // Callbacks and Events
        //--------------------------------------------------------------------------

        /**
         * @brief Set callback for state changes
         *
         * @param callback Function to call when controller state changes
         */
        void setStateChangeCallback(std::function<void(const ControllerState &)> callback);

        /**
         * @brief Set callback for gait transitions
         *
         * @param callback Function to call during gait transitions
         */
        void setGaitTransitionCallback(std::function<void(const std::string &, const std::string &)> callback);

        /**
         * @brief Set callback for emergency events
         *
         * @param callback Function to call on emergency stops
         */
        void setEmergencyCallback(std::function<void(const std::string &)> callback);

        /**
         * @brief Clear all callbacks
         */
        void clearCallbacks();

        //--------------------------------------------------------------------------
        // Debugging and Visualization
        //--------------------------------------------------------------------------

        /**
         * @brief Enable/disable debug mode
         *
         * @param enable True to enable debug output
         */
        void setDebugMode(bool enable);

        /**
         * @brief Check if debug mode is enabled
         *
         * @return true if debug mode is on
         */
        bool isDebugModeEnabled() const;

    private:
        /**
         * @brief Implementation pointer (PIMPL idiom)
         */
        std::unique_ptr<ControllerImpl> pImpl;
    };

    //==============================================================================
    // Utility Functions
    //==============================================================================

    /**
     * @brief Controller utility functions
     */
    namespace controller_utils
    {
        /**
         * @brief Create default locomotion command
         *
         * @param linear_vel Linear velocity (m/s)
         * @param angular_vel Angular velocity (rad/s)
         * @param gait_type Gait pattern name
         * @return LocomotionCommand Default command structure
         */
        LocomotionCommand createWalkCommand(double linear_vel, double angular_vel = 0.0,
                                            const std::string &gait_type = "tripod");

        /**
         * @brief Create emergency stop command
         *
         * @return LocomotionCommand Emergency stop command
         */
        LocomotionCommand createStopCommand();

        /**
         * @brief Validate locomotion command
         *
         * @param command Command to validate
         * @param config Controller configuration for limits
         * @return true if command is valid
         */
        bool validateCommand(const LocomotionCommand &command, const ControllerConfig &config);

        /**
         * @brief Convert velocity to step frequency
         *
         * @param velocity Linear velocity (m/s)
         * @param step_length Step length (m)
         * @return double Corresponding step frequency (Hz)
         */
        double velocityToFrequency(double velocity, double step_length);

        /**
         * @brief Estimate energy consumption
         *
         * @param joint_velocities Joint velocities (rad/s)
         * @param joint_torques Joint torques (N⋅m)
         * @return double Estimated power consumption (Watts)
         */
        double estimateEnergyConsumption(const std::vector<double> &joint_velocities,
                                         const std::vector<double> &joint_torques);

        /**
         * @brief Compute stability margin
         *
         * @param stance_legs Legs currently in stance
         * @param center_of_mass Center of mass position
         * @param support_polygon Support polygon vertices
         * @return double Stability margin [0,1]
         */
        double computeStabilityMargin(const std::vector<bool> &stance_legs);

        /**
         * @brief Select optimal gait for conditions
         *
         * @param velocity Desired velocity (m/s)
         * @param terrain_roughness Terrain roughness [0,1]
         * @param stability_requirement Required stability [0,1]
         * @return std::string Recommended gait type
         */
        std::string selectOptimalGait(double velocity, double terrain_roughness,
                                      double stability_requirement);
    }

} // namespace cpg

#endif // CPG_CONTROLLER_HPP
