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
#include <map>
#include <iomanip>
#include "cpg/network.hpp"

namespace cpg
{

    //==============================================================================
    // Constants and Utilities
    //==============================================================================

    namespace
    {
        // Mathematical constants
        constexpr double TWO_PI = 2.0 * M_PI;
        constexpr double PI = M_PI;

        // Network constraints
        constexpr size_t MAX_OSCILLATORS = 18;
        constexpr size_t HEXAPOD_LEGS = 6;
        constexpr double MIN_UPDATE_FREQUENCY = 0.001; // 1ms minimum
        constexpr double MAX_UPDATE_FREQUENCY = 1.0;   // 1s maximum

        // Synchronization thresholds
        constexpr double SYNC_THRESHOLD = 0.9;       // Synchronization threshold
        constexpr double STABILITY_THRESHOLD = 0.95; // Stability threshold
        constexpr size_t MIN_UPDATES_FOR_SYNC = 100; // Minimum updates before checking sync
    }

    //==============================================================================
    // Implementation Class (PIMPL idiom)
    //==============================================================================

    class NetworkImpl
    {
    public:
        /**
         * @brief Construct a new Network Implementation object
         */
        NetworkImpl(const NetworkParams &network_params, const GaitParams &gait_params)
            : m_networkParams(network_params),
              m_gaitParams(gait_params),
              m_networkTime(0.0),
              m_updateCount(0),
              m_debugMode(false),
              m_lastErrorMessage(""),
              m_globalFrequencyScale(1.0),
              m_stateCallback(nullptr)
        {
            // Initialize with empty oscillator vector
            m_oscillators.clear();
            m_connections.clear();
        }

        /**
         * @brief Initialize network with specified number of oscillators
         */
        bool initialize(size_t num_oscillators, const std::string &gait_type)
        {
            if (num_oscillators == 0 || num_oscillators > MAX_OSCILLATORS)
            {
                setLastError("Invalid number of oscillators: " + std::to_string(num_oscillators));
                return false;
            }

            // Clear existing oscillators
            m_oscillators.clear();
            m_connections.clear();

            // Create oscillators with gait-specific parameters
            for (size_t i = 0; i < num_oscillators; ++i)
            {
                OscillatorParams params = oscillator_utils::createGaitParameters(gait_type, i);
                auto oscillator = std::make_unique<Oscillator>(i, params);
                m_oscillators.push_back(std::move(oscillator));
            }

            // Update network parameters
            m_networkParams.num_oscillators = num_oscillators;

            // Create topology based on gait type
            if (num_oscillators == HEXAPOD_LEGS)
            {
                createHexapodTopology(gait_type);
            }

            clearLastError();
            return true;
        }

        /**
         * @brief Configure network specifically for hexapod locomotion
         */
        bool configureForHexapod(const std::string &gait_type)
        {
            return initialize(HEXAPOD_LEGS, gait_type);
        }

        /**
         * @brief Create hexapod-specific topology
         */
        void createHexapodTopology(const std::string &gait_type)
        {
            m_connections.clear();

            if (gait_type == "tripod")
            {
                // Tripod gait: strong coupling between opposite legs
                addConnection(0, 3, CouplingParams(0.8, PI)); // Left front - Right rear
                addConnection(1, 4, CouplingParams(0.8, PI)); // Right front - Left middle
                addConnection(2, 5, CouplingParams(0.8, PI)); // Left rear - Right middle
                addConnection(3, 0, CouplingParams(0.8, PI)); // Right rear - Left front
                addConnection(4, 1, CouplingParams(0.8, PI)); // Left middle - Right front
                addConnection(5, 2, CouplingParams(0.8, PI)); // Right middle - Left rear

                // Weaker coupling for coordination
                addConnection(0, 2, CouplingParams(0.3, 0.0)); // Left front - Left rear
                addConnection(1, 3, CouplingParams(0.3, 0.0)); // Right front - Right rear
                addConnection(2, 4, CouplingParams(0.3, 0.0)); // Left rear - Left middle
                addConnection(3, 5, CouplingParams(0.3, 0.0)); // Right rear - Right middle
            }
            else if (gait_type == "wave")
            {
                // Wave gait: sequential activation
                for (size_t i = 0; i < HEXAPOD_LEGS; ++i)
                {
                    size_t next = (i + 1) % HEXAPOD_LEGS;
                    addConnection(i, next, CouplingParams(0.6, PI / 3.0));
                }
            }
            else if (gait_type == "ripple")
            {
                // Ripple gait: two legs at a time
                for (size_t i = 0; i < HEXAPOD_LEGS; ++i)
                {
                    size_t opposite = (i + 3) % HEXAPOD_LEGS;
                    addConnection(i, opposite, CouplingParams(0.7, 2 * PI / 3.0));
                }
            }
        }

        /**
         * @brief Add connection between oscillators
         */
        bool addConnection(size_t from_id, size_t to_id, const CouplingParams &coupling_params)
        {
            if (from_id >= m_oscillators.size() || to_id >= m_oscillators.size())
            {
                setLastError("Invalid oscillator IDs for connection");
                return false;
            }

            if (from_id == to_id)
            {
                setLastError("Cannot connect oscillator to itself");
                return false;
            }

            // Check if connection already exists
            for (const auto &conn : m_connections)
            {
                if (conn.from_oscillator == from_id && conn.to_oscillator == to_id)
                {
                    setLastError("Connection already exists");
                    return false;
                }
            }

            // Create new connection
            ConnectionTopology connection(from_id, to_id, coupling_params);
            m_connections.push_back(connection);

            return true;
        }

        /**
         * @brief Update all oscillators and compute coupling
         */
        bool update(double dt)
        {
            if (dt <= 0.0 || dt > MAX_UPDATE_FREQUENCY)
            {
                setLastError("Invalid time step: " + std::to_string(dt));
                return false;
            }

            if (m_oscillators.empty())
            {
                setLastError("No oscillators in network");
                return false;
            }

            // Clear coupling inputs for all oscillators
            for (auto &oscillator : m_oscillators)
            {
                oscillator->clearCouplingInputs();
            }

            // Compute coupling inputs
            computeCouplingInputs();

            // Update all oscillators
            bool success = true;
            for (auto &oscillator : m_oscillators)
            {
                double coupling_input = oscillator->getTotalCouplingInput();
                if (!oscillator->update(dt, coupling_input))
                {
                    success = false;
                    setLastError("Failed to update oscillator " + std::to_string(oscillator->getId()));
                }
            }

            // Update network time
            m_networkTime += dt;
            m_updateCount++;

            // Call state callback if registered
            if (m_stateCallback && (m_updateCount % 10 == 0)) // Every 10 updates
            {
                NetworkState state = generateNetworkState();
                m_stateCallback(state);
            }

            return success;
        }

        /**
         * @brief Compute coupling inputs for all oscillators
         */
        void computeCouplingInputs()
        {
            for (const auto &connection : m_connections)
            {
                if (!connection.is_active)
                    continue;

                size_t from_id = connection.from_oscillator;
                size_t to_id = connection.to_oscillator;

                if (from_id >= m_oscillators.size() || to_id >= m_oscillators.size())
                {
                    continue; // Skip invalid connections
                }

                // Get phase difference
                double from_phase = m_oscillators[from_id]->getPhase();
                double to_phase = m_oscillators[to_id]->getPhase();
                double phase_diff = oscillator_utils::phaseDifference(from_phase, to_phase + connection.coupling.phase_bias);

                // Add coupling input to target oscillator
                m_oscillators[to_id]->addCouplingInput(
                    connection.coupling.strength,
                    phase_diff,
                    connection.coupling.bidirectional ? 1.0 : 0.5);

                // If bidirectional, add reverse coupling
                if (connection.coupling.bidirectional)
                {
                    double reverse_phase_diff = oscillator_utils::phaseDifference(to_phase, from_phase - connection.coupling.phase_bias);
                    m_oscillators[from_id]->addCouplingInput(
                        connection.coupling.strength,
                        reverse_phase_diff,
                        0.5 // Reduced strength for reverse coupling
                    );
                }
            }
        }

        /**
         * @brief Generate current network state
         */
        NetworkState generateNetworkState() const
        {
            NetworkState state;
            state.network_time = m_networkTime;
            state.global_frequency = m_globalFrequencyScale;
            state.is_synchronized = checkSynchronization();

            state.oscillator_states.clear();
            state.oscillator_states.reserve(m_oscillators.size());

            for (const auto &oscillator : m_oscillators)
            {
                state.oscillator_states.push_back(oscillator->getState());
            }

            return state;
        }

        /**
         * @brief Generate network output for robot control
         */
        NetworkOutput generateNetworkOutput() const
        {
            NetworkOutput output;

            output.joint_positions.clear();
            output.joint_velocities.clear();
            output.stance_phases.clear();
            output.gait_progress.clear();

            output.joint_positions.reserve(m_oscillators.size());
            output.joint_velocities.reserve(m_oscillators.size());
            output.stance_phases.reserve(m_oscillators.size());
            output.gait_progress.reserve(m_oscillators.size());

            for (const auto &oscillator : m_oscillators)
            {
                // Get oscillator output
                auto osc_output = oscillator->getOutput();

                // Convert to joint position (scaled for servo range)
                double joint_pos = osc_output.scaled_output * (PI / 4.0); // ±45 degrees
                output.joint_positions.push_back(joint_pos);

                // Estimate velocity (simple finite difference)
                double velocity = 0.0; // TODO: Implement proper velocity calculation
                output.joint_velocities.push_back(velocity);

                // Stance phase and gait progress
                output.stance_phases.push_back(osc_output.is_stance_phase);
                output.gait_progress.push_back(osc_output.phase_output);
            }

            // Set global properties
            output.step_frequency = m_gaitParams.step_frequency * m_globalFrequencyScale;
            output.is_stable = checkStability();

            return output;
        }

        /**
         * @brief Check if network is synchronized
         */
        bool checkSynchronization(double tolerance = 0.1) const
        {
            if (m_oscillators.size() < 2 || m_updateCount < MIN_UPDATES_FOR_SYNC)
            {
                return false;
            }

            // Calculate synchronization index
            std::vector<double> phases;
            phases.reserve(m_oscillators.size());

            for (const auto &oscillator : m_oscillators)
            {
                phases.push_back(oscillator->getPhase());
            }

            double sync_index = network_utils::computeSynchronizationIndex(phases);
            return sync_index >= (SYNC_THRESHOLD - tolerance);
        }

        /**
         * @brief Check if network is stable
         */
        bool checkStability() const
        {
            if (m_updateCount < MIN_UPDATES_FOR_SYNC)
                return false;

            for (const auto &oscillator : m_oscillators)
            {
                if (!oscillator->isStable())
                {
                    return false;
                }
            }
            return true;
        }

        /**
         * @brief Generate performance metrics string
         */
        std::string generatePerformanceMetrics() const
        {
            std::ostringstream oss;
            oss << "Network Performance Metrics:\n";
            oss << "  Network Time: " << std::fixed << std::setprecision(3) << m_networkTime << " s\n";
            oss << "  Update Count: " << m_updateCount << "\n";
            oss << "  Oscillators: " << m_oscillators.size() << "\n";
            oss << "  Connections: " << m_connections.size() << "\n";
            oss << "  Synchronized: " << (checkSynchronization() ? "Yes" : "No") << "\n";
            oss << "  Stable: " << (checkStability() ? "Yes" : "No") << "\n";
            oss << "  Global Frequency: " << m_globalFrequencyScale << "x\n";

            if (checkSynchronization())
            {
                std::vector<double> phases;
                for (const auto &oscillator : m_oscillators)
                {
                    phases.push_back(oscillator->getPhase());
                }
                double sync_index = network_utils::computeSynchronizationIndex(phases);
                oss << "  Synchronization Index: " << sync_index;
            }

            return oss.str();
        }

        /**
         * @brief Set error message
         */
        void setLastError(const std::string &message)
        {
            m_lastErrorMessage = message;
            if (m_debugMode)
            {
                std::cerr << "Network Error: " << message << std::endl;
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
        NetworkParams m_networkParams;
        GaitParams m_gaitParams;
        std::vector<std::unique_ptr<Oscillator>> m_oscillators;
        std::vector<ConnectionTopology> m_connections;
        double m_networkTime;
        size_t m_updateCount;
        bool m_debugMode;
        std::string m_lastErrorMessage;
        double m_globalFrequencyScale;
        std::function<void(const NetworkState &)> m_stateCallback;
    };

    //==============================================================================
    // Main Network Class Implementation
    //==============================================================================

    Network::Network(const NetworkParams &network_params, const GaitParams &gait_params)
        : pImpl(std::make_unique<NetworkImpl>(network_params, gait_params))
    {
    }

    Network::~Network() = default;

    Network::Network(Network &&other) noexcept
        : pImpl(std::move(other.pImpl))
    {
    }

    Network &Network::operator=(Network &&other) noexcept
    {
        if (this != &other)
        {
            pImpl = std::move(other.pImpl);
        }
        return *this;
    }

    //--------------------------------------------------------------------------
    // Network Configuration
    //--------------------------------------------------------------------------

    bool Network::initialize(size_t num_oscillators, const std::string &gait_type)
    {
        return pImpl->initialize(num_oscillators, gait_type);
    }

    bool Network::configureForHexapod(const std::string &gait_type)
    {
        return pImpl->configureForHexapod(gait_type);
    }

    bool Network::setNetworkParams(const NetworkParams &params)
    {
        if (!network_utils::validateNetworkParams(params))
        {
            pImpl->setLastError("Invalid network parameters");
            return false;
        }

        pImpl->m_networkParams = params;
        return true;
    }

    NetworkParams Network::getNetworkParams() const
    {
        return pImpl->m_networkParams;
    }

    //--------------------------------------------------------------------------
    // Oscillator Management
    //--------------------------------------------------------------------------

    size_t Network::getNumOscillators() const
    {
        return pImpl->m_oscillators.size();
    }

    size_t Network::addOscillator(const OscillatorParams &params)
    {
        size_t new_id = pImpl->m_oscillators.size();
        auto oscillator = std::make_unique<Oscillator>(new_id, params);
        pImpl->m_oscillators.push_back(std::move(oscillator));
        return new_id;
    }

    bool Network::removeOscillator(size_t oscillator_id)
    {
        if (oscillator_id >= pImpl->m_oscillators.size())
        {
            pImpl->setLastError("Invalid oscillator ID");
            return false;
        }

        // Remove oscillator and associated connections
        pImpl->m_oscillators.erase(pImpl->m_oscillators.begin() + oscillator_id);

        // Remove connections involving this oscillator
        auto &connections = pImpl->m_connections;
        connections.erase(
            std::remove_if(connections.begin(), connections.end(),
                           [oscillator_id](const ConnectionTopology &conn)
                           {
                               return conn.from_oscillator == oscillator_id || conn.to_oscillator == oscillator_id;
                           }),
            connections.end());

        return true;
    }

    bool Network::setOscillatorParams(size_t oscillator_id, const OscillatorParams &params)
    {
        if (oscillator_id >= pImpl->m_oscillators.size())
        {
            pImpl->setLastError("Invalid oscillator ID");
            return false;
        }

        return pImpl->m_oscillators[oscillator_id]->setParameters(params);
    }

    OscillatorParams Network::getOscillatorParams(size_t oscillator_id) const
    {
        if (oscillator_id >= pImpl->m_oscillators.size())
        {
            return OscillatorParams(); // Return default parameters
        }

        return pImpl->m_oscillators[oscillator_id]->getParameters();
    }

    //--------------------------------------------------------------------------
    // Coupling Management
    //--------------------------------------------------------------------------

    bool Network::addCoupling(size_t from_id, size_t to_id, const CouplingParams &coupling_params)
    {
        return pImpl->addConnection(from_id, to_id, coupling_params);
    }

    bool Network::removeCoupling(size_t from_id, size_t to_id)
    {
        auto &connections = pImpl->m_connections;
        auto it = std::find_if(connections.begin(), connections.end(),
                               [from_id, to_id](const ConnectionTopology &conn)
                               {
                                   return conn.from_oscillator == from_id && conn.to_oscillator == to_id;
                               });

        if (it != connections.end())
        {
            connections.erase(it);
            return true;
        }

        pImpl->setLastError("Coupling not found");
        return false;
    }

    bool Network::updateCoupling(size_t from_id, size_t to_id, const CouplingParams &coupling_params)
    {
        auto &connections = pImpl->m_connections;
        auto it = std::find_if(connections.begin(), connections.end(),
                               [from_id, to_id](const ConnectionTopology &conn)
                               {
                                   return conn.from_oscillator == from_id && conn.to_oscillator == to_id;
                               });

        if (it != connections.end())
        {
            it->coupling = coupling_params;
            return true;
        }

        pImpl->setLastError("Coupling not found");
        return false;
    }

    void Network::clearAllCouplings()
    {
        pImpl->m_connections.clear();
    }

    std::vector<ConnectionTopology> Network::getAllConnections() const
    {
        return pImpl->m_connections;
    }

    //--------------------------------------------------------------------------
    // Gait Control
    //--------------------------------------------------------------------------

    bool Network::setGaitParams(const GaitParams &gait_params)
    {
        // Basic validation
        if (gait_params.step_frequency <= 0.0 || gait_params.step_frequency > 5.0)
        {
            pImpl->setLastError("Invalid step frequency");
            return false;
        }

        pImpl->m_gaitParams = gait_params;
        return true;
    }

    GaitParams Network::getGaitParams() const
    {
        return pImpl->m_gaitParams;
    }

    bool Network::switchGait(const std::string &gait_type)
    {
        // TODO: Implement smooth gait transition
        return pImpl->configureForHexapod(gait_type);
    }

    bool Network::setGlobalFrequency(double frequency_scale)
    {
        if (frequency_scale <= 0.0 || frequency_scale > 5.0)
        {
            pImpl->setLastError("Invalid frequency scale");
            return false;
        }

        pImpl->m_globalFrequencyScale = frequency_scale;

        // Update all oscillator frequencies
        for (auto &oscillator : pImpl->m_oscillators)
        {
            auto params = oscillator->getParameters();
            oscillator->setFrequency(params.frequency * frequency_scale);
        }

        return true;
    }

    double Network::getGlobalFrequency() const
    {
        return pImpl->m_globalFrequencyScale;
    }

    //--------------------------------------------------------------------------
    // Network Simulation
    //--------------------------------------------------------------------------

    bool Network::update(double dt)
    {
        return pImpl->update(dt);
    }

    void Network::reset(bool random_phases)
    {
        for (auto &oscillator : pImpl->m_oscillators)
        {
            oscillator->reset(random_phases);
        }

        pImpl->m_networkTime = 0.0;
        pImpl->m_updateCount = 0;
        pImpl->clearLastError();
    }

    NetworkState Network::getNetworkState() const
    {
        return pImpl->generateNetworkState();
    }

    bool Network::setNetworkState(const NetworkState &state)
    {
        if (state.oscillator_states.size() != pImpl->m_oscillators.size())
        {
            pImpl->setLastError("State size mismatch");
            return false;
        }

        for (size_t i = 0; i < pImpl->m_oscillators.size(); ++i)
        {
            if (!pImpl->m_oscillators[i]->setState(state.oscillator_states[i]))
            {
                pImpl->setLastError("Failed to set oscillator state " + std::to_string(i));
                return false;
            }
        }

        pImpl->m_networkTime = state.network_time;
        return true;
    }

    //--------------------------------------------------------------------------
    // Output Generation
    //--------------------------------------------------------------------------

    NetworkOutput Network::getNetworkOutput() const
    {
        return pImpl->generateNetworkOutput();
    }

    std::vector<double> Network::getRawOutputs() const
    {
        std::vector<double> outputs;
        outputs.reserve(pImpl->m_oscillators.size());

        for (const auto &oscillator : pImpl->m_oscillators)
        {
            outputs.push_back(oscillator->getRawOutput());
        }

        return outputs;
    }

    std::vector<double> Network::getPhaseOutputs() const
    {
        std::vector<double> phases;
        phases.reserve(pImpl->m_oscillators.size());

        for (const auto &oscillator : pImpl->m_oscillators)
        {
            phases.push_back(oscillator->getPhaseOutput());
        }

        return phases;
    }

    std::vector<bool> Network::getStancePhases() const
    {
        std::vector<bool> stance_phases;
        stance_phases.reserve(pImpl->m_oscillators.size());

        for (const auto &oscillator : pImpl->m_oscillators)
        {
            stance_phases.push_back(oscillator->isStancePhase());
        }

        return stance_phases;
    }

    //--------------------------------------------------------------------------
    // Analysis and Monitoring
    //--------------------------------------------------------------------------

    bool Network::isSynchronized(double tolerance) const
    {
        return pImpl->checkSynchronization(tolerance);
    }

    bool Network::isStable() const
    {
        return pImpl->checkStability();
    }

    std::string Network::getPerformanceMetrics() const
    {
        return pImpl->generatePerformanceMetrics();
    }

    std::string Network::getNetworkStatistics() const
    {
        std::ostringstream oss;
        oss << "=== CPG Network Statistics ===\n";
        oss << pImpl->generatePerformanceMetrics() << "\n";

        oss << "\nIndividual Oscillator Statistics:\n";
        for (size_t i = 0; i < pImpl->m_oscillators.size(); ++i)
        {
            oss << "Oscillator " << i << ":\n";
            oss << "  Phase: " << pImpl->m_oscillators[i]->getPhase() << " rad\n";
            oss << "  Amplitude: " << pImpl->m_oscillators[i]->getAmplitude() << "\n";
            oss << "  Frequency: " << pImpl->m_oscillators[i]->getFrequency() << " Hz\n";
            oss << "  Stance: " << (pImpl->m_oscillators[i]->isStancePhase() ? "Yes" : "No") << "\n";
        }

        return oss.str();
    }

    //--------------------------------------------------------------------------
    // Validation and Error Handling
    //--------------------------------------------------------------------------

    bool Network::validateConfiguration() const
    {
        // Check basic configuration
        if (pImpl->m_oscillators.empty())
        {
            return false;
        }

        // Validate all oscillators
        for (const auto &oscillator : pImpl->m_oscillators)
        {
            if (!oscillator->validateState())
            {
                return false;
            }
        }

        // Validate connections
        for (const auto &connection : pImpl->m_connections)
        {
            if (connection.from_oscillator >= pImpl->m_oscillators.size() ||
                connection.to_oscillator >= pImpl->m_oscillators.size())
            {
                return false;
            }
        }

        return true;
    }

    std::string Network::getLastErrorMessage() const
    {
        return pImpl->m_lastErrorMessage;
    }

    void Network::clearLastError()
    {
        pImpl->clearLastError();
    }

    //--------------------------------------------------------------------------
    // Debugging and Visualization
    //--------------------------------------------------------------------------

    void Network::setDebugMode(bool enable)
    {
        pImpl->m_debugMode = enable;
        for (auto &oscillator : pImpl->m_oscillators)
        {
            oscillator->setDebugMode(enable);
        }
    }

    void Network::setStateCallback(std::function<void(const NetworkState &)> callback)
    {
        pImpl->m_stateCallback = callback;
    }

    void Network::clearStateCallback()
    {
        pImpl->m_stateCallback = nullptr;
    }

    //==============================================================================
    // Utility Functions Implementation
    //==============================================================================

    namespace network_utils
    {
        std::vector<ConnectionTopology> createHexapodTopology(const std::string &gait_type)
        {
            std::vector<ConnectionTopology> connections;

            if (gait_type == "tripod")
            {
                // Tripod gait connections
                connections.emplace_back(0, 3, CouplingParams(0.8, PI));
                connections.emplace_back(1, 4, CouplingParams(0.8, PI));
                connections.emplace_back(2, 5, CouplingParams(0.8, PI));
                connections.emplace_back(3, 0, CouplingParams(0.8, PI));
                connections.emplace_back(4, 1, CouplingParams(0.8, PI));
                connections.emplace_back(5, 2, CouplingParams(0.8, PI));
            }
            else if (gait_type == "wave")
            {
                // Wave gait connections
                for (size_t i = 0; i < HEXAPOD_LEGS; ++i)
                {
                    size_t next = (i + 1) % HEXAPOD_LEGS;
                    connections.emplace_back(i, next, CouplingParams(0.6, PI / 3.0));
                }
            }
            else if (gait_type == "ripple")
            {
                // Ripple gait connections
                for (size_t i = 0; i < HEXAPOD_LEGS; ++i)
                {
                    size_t opposite = (i + 3) % HEXAPOD_LEGS;
                    connections.emplace_back(i, opposite, CouplingParams(0.7, 2 * PI / 3.0));
                }
            }

            return connections;
        }

        std::vector<double> calculateGaitPhases(const std::string &gait_type, size_t num_legs)
        {
            std::vector<double> phases(num_legs, 0.0);

            if (gait_type == "tripod" && num_legs == 6)
            {
                phases = {0.0, PI, 0.0, PI, 0.0, PI}; // Alternating pattern
            }
            else if (gait_type == "wave" && num_legs == 6)
            {
                for (size_t i = 0; i < num_legs; ++i)
                {
                    phases[i] = (i * TWO_PI) / num_legs; // Sequential
                }
            }
            else if (gait_type == "ripple" && num_legs == 6)
            {
                phases = {0.0, 2 * PI / 3, 4 * PI / 3, PI, PI + 2 * PI / 3, PI + 4 * PI / 3};
            }

            return phases;
        }

        bool validateNetworkParams(const NetworkParams &params)
        {
            if (params.num_oscillators == 0 || params.num_oscillators > MAX_OSCILLATORS)
                return false;
            if (params.global_frequency <= 0.0 || params.global_frequency > 10.0)
                return false;
            if (params.noise_level < 0.0 || params.noise_level > 1.0)
                return false;

            return true;
        }

        double cpgToJointAngle(const std::vector<double> &cpg_outputs, size_t leg_id, size_t joint_id)
        {
            if (leg_id >= 6 || joint_id >= 3)
                return 0.0;

            size_t index = leg_id; // Simplified mapping for now
            if (index >= cpg_outputs.size())
                return 0.0;

            // Scale CPG output to joint range
            double scaled_output = cpg_outputs[index];

            // Apply joint-specific scaling
            switch (joint_id)
            {
            case 0:                                // Coxa
                return scaled_output * (PI / 4.0); // ±45 degrees
            case 1:                                // Femur
                return scaled_output * (PI / 3.0); // ±60 degrees
            case 2:                                // Tibia
                return scaled_output * (PI / 2.0); // ±90 degrees
            default:
                return 0.0;
            }
        }

        double computeSynchronizationIndex(const std::vector<double> &phases)
        {
            if (phases.size() < 2)
                return 1.0;

            // Compute order parameter
            double sum_cos = 0.0, sum_sin = 0.0;
            for (double phase : phases)
            {
                sum_cos += std::cos(phase);
                sum_sin += std::sin(phase);
            }

            double order_parameter = std::sqrt(sum_cos * sum_cos + sum_sin * sum_sin) / phases.size();
            return order_parameter;
        }

        std::vector<std::vector<double>> generateHexapodCouplingMatrix(double coupling_strength)
        {
            std::vector<std::vector<double>> matrix(6, std::vector<double>(6, 0.0));

            // Tripod gait coupling pattern
            matrix[0][3] = matrix[3][0] = coupling_strength; // Left front - Right rear
            matrix[1][4] = matrix[4][1] = coupling_strength; // Right front - Left middle
            matrix[2][5] = matrix[5][2] = coupling_strength; // Left rear - Right middle

            // Weaker ipsilateral coupling
            double weak_coupling = coupling_strength * 0.3;
            matrix[0][2] = matrix[2][0] = weak_coupling; // Left side
            matrix[1][3] = matrix[3][1] = weak_coupling; // Right side
            matrix[2][4] = matrix[4][2] = weak_coupling; // Left-middle
            matrix[3][5] = matrix[5][3] = weak_coupling; // Right-middle

            return matrix;
        }
    }

} // namespace cpg
