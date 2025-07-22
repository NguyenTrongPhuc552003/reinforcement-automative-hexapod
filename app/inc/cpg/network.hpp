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

#ifndef CPG_NETWORK_HPP
#define CPG_NETWORK_HPP

#include <memory>
#include <vector>
#include <string>
#include <functional>
#include "cpg/oscillator.hpp"
#include "cpg/parameters.hpp"

/**
 * @brief Central Pattern Generator (CPG) network implementation
 *
 * This namespace contains classes and functions for implementing CPG networks
 * that coordinate multiple oscillators for hexapod locomotion.
 */
namespace cpg
{

    //==============================================================================
    // Network State and Output
    //==============================================================================

    /**
     * @brief Complete network state snapshot
     *
     * Contains the state of all oscillators in the network at a given time.
     */
    struct NetworkState
    {
        std::vector<OscillatorState> oscillator_states; ///< State of each oscillator
        double network_time;                            ///< Current network time
        double global_frequency;                        ///< Current global frequency
        bool is_synchronized;                           ///< Whether network is synchronized

        /**
         * @brief Default constructor
         */
        NetworkState()
            : network_time(0.0), global_frequency(1.0), is_synchronized(false)
        {
        }
    };

    /**
     * @brief Network output for robot control
     *
     * Contains processed outputs suitable for controlling hexapod joints.
     */
    struct NetworkOutput
    {
        std::vector<double> joint_positions;  ///< Target joint positions (radians)
        std::vector<double> joint_velocities; ///< Target joint velocities (rad/s)
        std::vector<bool> stance_phases;      ///< Stance phase for each leg
        std::vector<double> gait_progress;    ///< Gait progress for each leg [0,1]
        double step_frequency;                ///< Current step frequency
        bool is_stable;                       ///< Whether output is stable

        /**
         * @brief Default constructor
         */
        NetworkOutput()
            : step_frequency(0.0), is_stable(false)
        {
        }
    };

    /**
     * @brief Connection topology for network coupling
     */
    struct ConnectionTopology
    {
        size_t from_oscillator;  ///< Source oscillator ID
        size_t to_oscillator;    ///< Target oscillator ID
        CouplingParams coupling; ///< Coupling parameters
        bool is_active;          ///< Whether connection is active

        /**
         * @brief Default constructor
         */
        ConnectionTopology()
            : from_oscillator(0), to_oscillator(0), is_active(true)
        {
        }

        /**
         * @brief Parameterized constructor
         */
        ConnectionTopology(size_t from, size_t to, const CouplingParams &params)
            : from_oscillator(from), to_oscillator(to), coupling(params), is_active(true)
        {
        }
    };

    //==============================================================================
    // Forward declarations
    //==============================================================================

    // Implementation class (PIMPL idiom)
    class NetworkImpl;

    //==============================================================================
    // Main Network Class
    //==============================================================================

    /**
     * @brief CPG Network coordinator
     *
     * Manages a collection of coupled oscillators to generate coordinated
     * locomotion patterns for the hexapod robot.
     */
    class Network
    {
    public:
        /**
         * @brief Construct a new Network object
         *
         * @param network_params Network configuration parameters
         * @param gait_params Initial gait parameters
         */
        Network(const NetworkParams &network_params = NetworkParams(),
                const GaitParams &gait_params = GaitParams());

        /**
         * @brief Destroy the Network object
         */
        ~Network();

        // Non-copyable
        Network(const Network &) = delete;
        Network &operator=(const Network &) = delete;

        // Move semantics
        Network(Network &&other) noexcept;
        Network &operator=(Network &&other) noexcept;

        //--------------------------------------------------------------------------
        // Network Configuration
        //--------------------------------------------------------------------------

        /**
         * @brief Initialize network with specific configuration
         *
         * @param num_oscillators Number of oscillators in network
         * @param gait_type Type of gait pattern ("tripod", "wave", "ripple")
         * @return true if initialization successful
         * @return false if initialization failed
         */
        bool initialize(size_t num_oscillators, const std::string &gait_type = "tripod");

        /**
         * @brief Configure network for hexapod locomotion
         *
         * Sets up 6 oscillators with proper coupling for hexapod gaits.
         *
         * @param gait_type Type of gait pattern
         * @return true if configuration successful
         * @return false if configuration failed
         */
        bool configureForHexapod(const std::string &gait_type = "tripod");

        /**
         * @brief Set network parameters
         *
         * @param params Network parameters to set
         * @return true if parameters were set successfully
         * @return false if parameters are invalid
         */
        bool setNetworkParams(const NetworkParams &params);

        /**
         * @brief Get current network parameters
         *
         * @return NetworkParams Current network parameters
         */
        NetworkParams getNetworkParams() const;

        //--------------------------------------------------------------------------
        // Oscillator Management
        //--------------------------------------------------------------------------

        /**
         * @brief Get number of oscillators in network
         *
         * @return size_t Number of oscillators
         */
        size_t getNumOscillators() const;

        /**
         * @brief Add oscillator to network
         *
         * @param params Parameters for the new oscillator
         * @return size_t ID of the added oscillator
         */
        size_t addOscillator(const OscillatorParams &params);

        /**
         * @brief Remove oscillator from network
         *
         * @param oscillator_id ID of oscillator to remove
         * @return true if removal successful
         * @return false if oscillator_id is invalid
         */
        bool removeOscillator(size_t oscillator_id);

        /**
         * @brief Set parameters for specific oscillator
         *
         * @param oscillator_id ID of oscillator to configure
         * @param params New parameters for the oscillator
         * @return true if parameters were set successfully
         * @return false if oscillator_id is invalid or parameters are invalid
         */
        bool setOscillatorParams(size_t oscillator_id, const OscillatorParams &params);

        /**
         * @brief Get parameters for specific oscillator
         *
         * @param oscillator_id ID of oscillator
         * @return OscillatorParams Parameters of the oscillator
         */
        OscillatorParams getOscillatorParams(size_t oscillator_id) const;

        //--------------------------------------------------------------------------
        // Coupling Management
        //--------------------------------------------------------------------------

        /**
         * @brief Add coupling between two oscillators
         *
         * @param from_id Source oscillator ID
         * @param to_id Target oscillator ID
         * @param coupling_params Coupling parameters
         * @return true if coupling was added successfully
         * @return false if IDs are invalid or coupling already exists
         */
        bool addCoupling(size_t from_id, size_t to_id, const CouplingParams &coupling_params);

        /**
         * @brief Remove coupling between two oscillators
         *
         * @param from_id Source oscillator ID
         * @param to_id Target oscillator ID
         * @return true if coupling was removed successfully
         * @return false if coupling doesn't exist or IDs are invalid
         */
        bool removeCoupling(size_t from_id, size_t to_id);

        /**
         * @brief Update coupling parameters
         *
         * @param from_id Source oscillator ID
         * @param to_id Target oscillator ID
         * @param coupling_params New coupling parameters
         * @return true if update successful
         * @return false if coupling doesn't exist or parameters are invalid
         */
        bool updateCoupling(size_t from_id, size_t to_id, const CouplingParams &coupling_params);

        /**
         * @brief Clear all couplings
         */
        void clearAllCouplings();

        /**
         * @brief Get all active connections
         *
         * @return std::vector<ConnectionTopology> List of all connections
         */
        std::vector<ConnectionTopology> getAllConnections() const;

        //--------------------------------------------------------------------------
        // Gait Control
        //--------------------------------------------------------------------------

        /**
         * @brief Set gait parameters
         *
         * @param gait_params Gait parameters to set
         * @return true if parameters were set successfully
         * @return false if parameters are invalid
         */
        bool setGaitParams(const GaitParams &gait_params);

        /**
         * @brief Get current gait parameters
         *
         * @return GaitParams Current gait parameters
         */
        GaitParams getGaitParams() const;

        /**
         * @brief Switch to different gait pattern
         *
         * @param gait_type Type of gait ("tripod", "wave", "ripple")
         * @param transition_time Time to transition between gaits (seconds)
         * @return true if gait switch initiated successfully
         * @return false if gait_type is invalid
         */
        bool switchGait(const std::string &gait_type);

        /**
         * @brief Set global frequency scaling
         *
         * @param frequency_scale Scaling factor for all oscillator frequencies
         * @return true if frequency was set successfully
         * @return false if frequency_scale is invalid
         */
        bool setGlobalFrequency(double frequency_scale);

        /**
         * @brief Get current global frequency
         *
         * @return double Current global frequency scaling
         */
        double getGlobalFrequency() const;

        //--------------------------------------------------------------------------
        // Network Simulation
        //--------------------------------------------------------------------------

        /**
         * @brief Update network by one time step
         *
         * Performs numerical integration for all oscillators and coupling.
         *
         * @param dt Time step size (seconds)
         * @return true if update successful
         * @return false if update failed
         */
        bool update(double dt);

        /**
         * @brief Reset network to initial state
         *
         * @param random_phases If true, use random initial phases
         */
        void reset(bool random_phases = false);

        /**
         * @brief Get current network state
         *
         * @return NetworkState Complete state of all oscillators
         */
        NetworkState getNetworkState() const;

        /**
         * @brief Set network state
         *
         * @param state New state to set
         * @return true if state was set successfully
         * @return false if state is invalid
         */
        bool setNetworkState(const NetworkState &state);

        //--------------------------------------------------------------------------
        // Output Generation
        //--------------------------------------------------------------------------

        /**
         * @brief Get network output for robot control
         *
         * @return NetworkOutput Processed output for joint control
         */
        NetworkOutput getNetworkOutput() const;

        /**
         * @brief Get raw oscillator outputs
         *
         * @return std::vector<double> Raw output from each oscillator
         */
        std::vector<double> getRawOutputs() const;

        /**
         * @brief Get phase outputs for all oscillators
         *
         * @return std::vector<double> Phase output [0,1] for each oscillator
         */
        std::vector<double> getPhaseOutputs() const;

        /**
         * @brief Get stance phase status for all legs
         *
         * @return std::vector<bool> True if leg is in stance phase
         */
        std::vector<bool> getStancePhases() const;

        //--------------------------------------------------------------------------
        // Analysis and Monitoring
        //--------------------------------------------------------------------------

        /**
         * @brief Check if network is synchronized
         *
         * @param tolerance Phase tolerance for synchronization check
         * @return true if network is synchronized within tolerance
         * @return false if network is not synchronized
         */
        bool isSynchronized(double tolerance = 0.1) const;

        /**
         * @brief Check if network is stable
         *
         * @return true if all oscillators are in stable limit cycles
         * @return false if any oscillator is unstable
         */
        bool isStable() const;

        /**
         * @brief Get network performance metrics
         *
         * @return std::string Formatted performance metrics
         */
        std::string getPerformanceMetrics() const;

        /**
         * @brief Get network statistics
         *
         * @return std::string Formatted statistics for all oscillators
         */
        std::string getNetworkStatistics() const;

        //--------------------------------------------------------------------------
        // Validation and Error Handling
        //--------------------------------------------------------------------------

        /**
         * @brief Validate current network configuration
         *
         * @return true if configuration is valid
         * @return false if configuration has errors
         */
        bool validateConfiguration() const;

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

        //--------------------------------------------------------------------------
        // File I/O and Configuration
        //--------------------------------------------------------------------------

        /**
         * @brief Save network configuration to file
         *
         * @param filename Path to save configuration
         * @return true if saving successful
         * @return false if saving failed
         */
        bool saveConfiguration(const std::string &filename) const;

        /**
         * @brief Load network configuration from file
         *
         * @param filename Path to configuration file
         * @return true if loading successful
         * @return false if loading failed
         */
        bool loadConfiguration(const std::string &filename);

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
         * @brief Set callback for state monitoring
         *
         * @param callback Function to call with network state updates
         */
        void setStateCallback(std::function<void(const NetworkState &)> callback);

        /**
         * @brief Remove state monitoring callback
         */
        void clearStateCallback();

    private:
        /**
         * @brief Implementation pointer (PIMPL idiom)
         */
        std::unique_ptr<NetworkImpl> pImpl;
    };

    //==============================================================================
    // Utility Functions
    //==============================================================================

    /**
     * @brief Network utility functions
     */
    namespace network_utils
    {
        /**
         * @brief Create predefined hexapod network topology
         *
         * @param gait_type Type of gait ("tripod", "wave", "ripple")
         * @return std::vector<ConnectionTopology> Network connections
         */
        std::vector<ConnectionTopology> createHexapodTopology(const std::string &gait_type);

        /**
         * @brief Calculate phase differences for gait pattern
         *
         * @param gait_type Type of gait
         * @param num_legs Number of legs (typically 6)
         * @return std::vector<double> Phase differences for each leg
         */
        std::vector<double> calculateGaitPhases(const std::string &gait_type, size_t num_legs);

        /**
         * @brief Validate network parameters
         *
         * @param params Network parameters to validate
         * @return true if parameters are valid
         */
        bool validateNetworkParams(const NetworkParams &params);

        /**
         * @brief Convert CPG output to joint angles
         *
         * @param cpg_outputs Raw CPG outputs
         * @param leg_id Leg identifier (0-5)
         * @param joint_id Joint identifier (0-2 for coxa-femur-tibia)
         * @return double Joint angle in radians
         */
        double cpgToJointAngle(const std::vector<double> &cpg_outputs,
                               size_t leg_id, size_t joint_id);

        /**
         * @brief Compute network synchronization index
         *
         * @param phases Vector of oscillator phases
         * @return double Synchronization index [0,1] where 1 is fully synchronized
         */
        double computeSynchronizationIndex(const std::vector<double> &phases);

        /**
         * @brief Generate default coupling matrix for hexapod
         *
         * @param coupling_strength Base coupling strength
         * @return std::vector<std::vector<double>> 6x6 coupling matrix
         */
        std::vector<std::vector<double>> generateHexapodCouplingMatrix(double coupling_strength = 0.1);
    }

} // namespace cpg

#endif // CPG_NETWORK_HPP
