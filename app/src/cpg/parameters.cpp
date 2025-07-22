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

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <map>
#include <iostream>
#include "cpg/parameters.hpp"

namespace cpg
{

    //==============================================================================
    // Parameter Validation Constants
    //==============================================================================

    namespace ParameterLimits
    {
        // Oscillator parameter limits
        constexpr double MIN_FREQUENCY = 0.01; // 0.01 Hz minimum
        constexpr double MAX_FREQUENCY = 10.0; // 10 Hz maximum
        constexpr double MIN_AMPLITUDE = 0.0;  // 0.0 minimum amplitude
        constexpr double MAX_AMPLITUDE = 10.0; // 10.0 maximum amplitude
        constexpr double MIN_DUTY_CYCLE = 0.1; // 10% minimum duty cycle
        constexpr double MAX_DUTY_CYCLE = 0.9; // 90% maximum duty cycle

        // Coupling parameter limits
        constexpr double MIN_COUPLING_STRENGTH = 0.0;
        constexpr double MAX_COUPLING_STRENGTH = 1.0;

        // Gait parameter limits
        constexpr double MIN_STEP_FREQUENCY = 0.1;
        constexpr double MAX_STEP_FREQUENCY = 5.0;
        constexpr double MIN_PHASE_DURATION = 0.1;
        constexpr double MAX_PHASE_DURATION = 0.9;
        constexpr double MIN_STEP_HEIGHT = 10.0;  // mm
        constexpr double MAX_STEP_HEIGHT = 100.0; // mm
        constexpr double MIN_STEP_LENGTH = 20.0;  // mm
        constexpr double MAX_STEP_LENGTH = 150.0; // mm

        // Network parameter limits
        constexpr size_t MIN_OSCILLATORS = 1;
        constexpr size_t MAX_OSCILLATORS = 12;
        constexpr double MIN_NOISE_LEVEL = 0.0;
        constexpr double MAX_NOISE_LEVEL = 0.5;
    }

    //==============================================================================
    // Implementation Class (PIMPL idiom)
    //==============================================================================

    class ParametersImpl
    {
    public:
        /**
         * @brief Construct a new Parameters Implementation object
         */
        ParametersImpl()
        {
            resetToDefaults();
        }

        /**
         * @brief Reset all parameters to safe defaults
         */
        void resetToDefaults()
        {
            // Set default network parameters
            networkParams.num_oscillators = 6; // 6 legs
            networkParams.global_frequency = 1.0;
            networkParams.noise_level = 0.0;
            networkParams.adaptive_coupling = false;

            // Initialize oscillator parameters for each leg
            oscillatorParams.clear();
            oscillatorParams.resize(networkParams.num_oscillators);

            for (size_t i = 0; i < networkParams.num_oscillators; ++i)
            {
                oscillatorParams[i] = OscillatorParams(1.0, 1.0, 0.0, 0.0, 0.5);
            }

            // Set default gait parameters
            gaitParams.step_frequency = 1.0;
            gaitParams.stance_duration = 0.6;
            gaitParams.swing_duration = 0.4;
            gaitParams.step_height = 30.0;
            gaitParams.step_length = 60.0;

            // Clear coupling matrix
            couplingMatrix.clear();

            // Clear any error
            lastError.clear();
        }

        /**
         * @brief Validate oscillator parameters
         */
        bool validateOscillatorParams(const OscillatorParams &params)
        {
            if (params.frequency < ParameterLimits::MIN_FREQUENCY ||
                params.frequency > ParameterLimits::MAX_FREQUENCY)
            {
                lastError = "Frequency out of range [" +
                            std::to_string(ParameterLimits::MIN_FREQUENCY) + ", " +
                            std::to_string(ParameterLimits::MAX_FREQUENCY) + "]";
                return false;
            }

            if (params.amplitude < ParameterLimits::MIN_AMPLITUDE ||
                params.amplitude > ParameterLimits::MAX_AMPLITUDE)
            {
                lastError = "Amplitude out of range [" +
                            std::to_string(ParameterLimits::MIN_AMPLITUDE) + ", " +
                            std::to_string(ParameterLimits::MAX_AMPLITUDE) + "]";
                return false;
            }

            if (params.duty_cycle < ParameterLimits::MIN_DUTY_CYCLE ||
                params.duty_cycle > ParameterLimits::MAX_DUTY_CYCLE)
            {
                lastError = "Duty cycle out of range [" +
                            std::to_string(ParameterLimits::MIN_DUTY_CYCLE) + ", " +
                            std::to_string(ParameterLimits::MAX_DUTY_CYCLE) + "]";
                return false;
            }

            return true;
        }

        /**
         * @brief Validate coupling parameters
         */
        bool validateCouplingParams(const CouplingParams &params)
        {
            if (params.strength < ParameterLimits::MIN_COUPLING_STRENGTH ||
                params.strength > ParameterLimits::MAX_COUPLING_STRENGTH)
            {
                lastError = "Coupling strength out of range [" +
                            std::to_string(ParameterLimits::MIN_COUPLING_STRENGTH) + ", " +
                            std::to_string(ParameterLimits::MAX_COUPLING_STRENGTH) + "]";
                return false;
            }

            return true;
        }

        /**
         * @brief Validate gait parameters
         */
        bool validateGaitParams(const GaitParams &params)
        {
            if (params.step_frequency < ParameterLimits::MIN_STEP_FREQUENCY ||
                params.step_frequency > ParameterLimits::MAX_STEP_FREQUENCY)
            {
                lastError = "Step frequency out of range";
                return false;
            }

            if (params.stance_duration < ParameterLimits::MIN_PHASE_DURATION ||
                params.stance_duration > ParameterLimits::MAX_PHASE_DURATION)
            {
                lastError = "Stance duration out of range";
                return false;
            }

            if (params.swing_duration < ParameterLimits::MIN_PHASE_DURATION ||
                params.swing_duration > ParameterLimits::MAX_PHASE_DURATION)
            {
                lastError = "Swing duration out of range";
                return false;
            }

            if (std::abs((params.stance_duration + params.swing_duration) - 1.0) > 0.01)
            {
                lastError = "Stance and swing durations must sum to 1.0";
                return false;
            }

            if (params.step_height < ParameterLimits::MIN_STEP_HEIGHT ||
                params.step_height > ParameterLimits::MAX_STEP_HEIGHT)
            {
                lastError = "Step height out of range";
                return false;
            }

            if (params.step_length < ParameterLimits::MIN_STEP_LENGTH ||
                params.step_length > ParameterLimits::MAX_STEP_LENGTH)
            {
                lastError = "Step length out of range";
                return false;
            }

            return true;
        }

        /**
         * @brief Validate network parameters
         */
        bool validateNetworkParams(const NetworkParams &params)
        {
            if (params.num_oscillators < ParameterLimits::MIN_OSCILLATORS ||
                params.num_oscillators > ParameterLimits::MAX_OSCILLATORS)
            {
                lastError = "Number of oscillators out of range";
                return false;
            }

            if (params.global_frequency < ParameterLimits::MIN_FREQUENCY ||
                params.global_frequency > ParameterLimits::MAX_FREQUENCY)
            {
                lastError = "Global frequency out of range";
                return false;
            }

            if (params.noise_level < ParameterLimits::MIN_NOISE_LEVEL ||
                params.noise_level > ParameterLimits::MAX_NOISE_LEVEL)
            {
                lastError = "Noise level out of range";
                return false;
            }

            return true;
        }

        /**
         * @brief Generate coupling key for the coupling matrix
         */
        std::string makeCouplingKey(size_t from_id, size_t to_id) const
        {
            return std::to_string(from_id) + "->" + std::to_string(to_id);
        }

        // Member variables
        std::vector<OscillatorParams> oscillatorParams;
        std::map<std::string, CouplingParams> couplingMatrix;
        GaitParams gaitParams;
        NetworkParams networkParams;
        std::string lastError;
    };

    //==============================================================================
    // Parameters Class Implementation
    //==============================================================================

    // Constructor and Destructor
    Parameters::Parameters() : pImpl(std::make_unique<ParametersImpl>()) {}

    Parameters::~Parameters() = default;

    // Move semantics
    Parameters::Parameters(Parameters &&other) noexcept = default;
    Parameters &Parameters::operator=(Parameters &&other) noexcept = default;

    // Oscillator Parameter Management
    bool Parameters::setOscillatorParams(size_t oscillator_id, const OscillatorParams &params)
    {
        if (oscillator_id >= pImpl->networkParams.num_oscillators)
        {
            pImpl->lastError = "Invalid oscillator ID: " + std::to_string(oscillator_id);
            return false;
        }

        if (!pImpl->validateOscillatorParams(params))
        {
            return false;
        }

        pImpl->oscillatorParams[oscillator_id] = params;
        pImpl->lastError.clear();
        return true;
    }

    bool Parameters::getOscillatorParams(size_t oscillator_id, OscillatorParams &params) const
    {
        if (oscillator_id >= pImpl->networkParams.num_oscillators)
        {
            pImpl->lastError = "Invalid oscillator ID: " + std::to_string(oscillator_id);
            return false;
        }

        params = pImpl->oscillatorParams[oscillator_id];
        return true;
    }

    bool Parameters::setAllOscillatorParams(const std::vector<OscillatorParams> &params_list)
    {
        if (params_list.size() != pImpl->networkParams.num_oscillators)
        {
            pImpl->lastError = "Parameter list size (" + std::to_string(params_list.size()) +
                               ") doesn't match number of oscillators (" +
                               std::to_string(pImpl->networkParams.num_oscillators) + ")";
            return false;
        }

        // Validate all parameters first
        for (size_t i = 0; i < params_list.size(); ++i)
        {
            if (!pImpl->validateOscillatorParams(params_list[i]))
            {
                pImpl->lastError = "Invalid parameters for oscillator " + std::to_string(i) + ": " + pImpl->lastError;
                return false;
            }
        }

        // If all valid, apply them
        pImpl->oscillatorParams = params_list;
        pImpl->lastError.clear();
        return true;
    }

    std::vector<OscillatorParams> Parameters::getAllOscillatorParams() const
    {
        return pImpl->oscillatorParams;
    }

    // Coupling Parameter Management
    bool Parameters::setCouplingParams(size_t from_id, size_t to_id, const CouplingParams &params)
    {
        if (from_id >= pImpl->networkParams.num_oscillators ||
            to_id >= pImpl->networkParams.num_oscillators)
        {
            pImpl->lastError = "Invalid oscillator IDs for coupling";
            return false;
        }

        if (from_id == to_id)
        {
            pImpl->lastError = "Cannot couple oscillator to itself";
            return false;
        }

        if (!pImpl->validateCouplingParams(params))
        {
            return false;
        }

        std::string key = pImpl->makeCouplingKey(from_id, to_id);
        pImpl->couplingMatrix[key] = params;
        pImpl->lastError.clear();
        return true;
    }

    bool Parameters::getCouplingParams(size_t from_id, size_t to_id, CouplingParams &params) const
    {
        if (from_id >= pImpl->networkParams.num_oscillators ||
            to_id >= pImpl->networkParams.num_oscillators)
        {
            pImpl->lastError = "Invalid oscillator IDs for coupling";
            return false;
        }

        std::string key = pImpl->makeCouplingKey(from_id, to_id);
        auto it = pImpl->couplingMatrix.find(key);
        if (it == pImpl->couplingMatrix.end())
        {
            pImpl->lastError = "Coupling not found between oscillators " +
                               std::to_string(from_id) + " and " + std::to_string(to_id);
            return false;
        }

        params = it->second;
        return true;
    }

    bool Parameters::removeCoupling(size_t from_id, size_t to_id)
    {
        if (from_id >= pImpl->networkParams.num_oscillators ||
            to_id >= pImpl->networkParams.num_oscillators)
        {
            pImpl->lastError = "Invalid oscillator IDs for coupling";
            return false;
        }

        std::string key = pImpl->makeCouplingKey(from_id, to_id);
        auto it = pImpl->couplingMatrix.find(key);
        if (it == pImpl->couplingMatrix.end())
        {
            pImpl->lastError = "Coupling not found";
            return false;
        }

        pImpl->couplingMatrix.erase(it);
        pImpl->lastError.clear();
        return true;
    }

    void Parameters::clearAllCouplings()
    {
        pImpl->couplingMatrix.clear();
        pImpl->lastError.clear();
    }

    // Gait Parameter Management
    bool Parameters::setGaitParams(const GaitParams &params)
    {
        if (!pImpl->validateGaitParams(params))
        {
            return false;
        }

        pImpl->gaitParams = params;
        pImpl->lastError.clear();
        return true;
    }

    GaitParams Parameters::getGaitParams() const
    {
        return pImpl->gaitParams;
    }

    // Network Parameter Management
    bool Parameters::setNetworkParams(const NetworkParams &params)
    {
        if (!pImpl->validateNetworkParams(params))
        {
            return false;
        }

        // If number of oscillators changed, resize the oscillator parameters vector
        if (params.num_oscillators != pImpl->networkParams.num_oscillators)
        {
            size_t old_size = pImpl->oscillatorParams.size();
            pImpl->oscillatorParams.resize(params.num_oscillators);

            // Initialize new oscillators with default parameters
            for (size_t i = old_size; i < params.num_oscillators; ++i)
            {
                pImpl->oscillatorParams[i] = OscillatorParams();
            }

            // Clear couplings that reference non-existent oscillators
            auto it = pImpl->couplingMatrix.begin();
            while (it != pImpl->couplingMatrix.end())
            {
                // Parse the key to get oscillator IDs
                std::istringstream ss(it->first);
                std::string from_str, to_str;
                std::getline(ss, from_str, '-');
                std::getline(ss, to_str, '>');
                std::getline(ss, to_str);

                size_t from_id = std::stoul(from_str);
                size_t to_id = std::stoul(to_str);

                if (from_id >= params.num_oscillators || to_id >= params.num_oscillators)
                {
                    it = pImpl->couplingMatrix.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }

        pImpl->networkParams = params;
        pImpl->lastError.clear();
        return true;
    }

    NetworkParams Parameters::getNetworkParams() const
    {
        return pImpl->networkParams;
    }

    // Validation and Utility
    bool Parameters::validateAll() const
    {
        // Validate network parameters
        if (!pImpl->validateNetworkParams(pImpl->networkParams))
        {
            return false;
        }

        // Validate gait parameters
        if (!pImpl->validateGaitParams(pImpl->gaitParams))
        {
            return false;
        }

        // Validate all oscillator parameters
        for (size_t i = 0; i < pImpl->oscillatorParams.size(); ++i)
        {
            if (!pImpl->validateOscillatorParams(pImpl->oscillatorParams[i]))
            {
                return false;
            }
        }

        // Validate all coupling parameters
        for (const auto &coupling : pImpl->couplingMatrix)
        {
            if (!pImpl->validateCouplingParams(coupling.second))
            {
                return false;
            }
        }

        return true;
    }

    void Parameters::resetToDefaults()
    {
        pImpl->resetToDefaults();
    }

    size_t Parameters::getNumOscillators() const
    {
        return pImpl->networkParams.num_oscillators;
    }

    // File I/O
    bool Parameters::loadFromFile(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            pImpl->lastError = "Failed to open file: " + filename;
            return false;
        }

        try
        {
            // Simple text-based format for now
            // In a production system, you might use JSON or XML
            std::string line;
            while (std::getline(file, line))
            {
                // Skip comments and empty lines
                if (line.empty() || line[0] == '#')
                    continue;

                // Parse parameter lines (simplified parser)
                std::istringstream iss(line);
                std::string key;
                if (!(iss >> key))
                    continue;

                // Example parsing - extend as needed
                if (key == "global_frequency")
                {
                    double value;
                    if (iss >> value)
                    {
                        pImpl->networkParams.global_frequency = value;
                    }
                }
                // Add more parameter parsing as needed
            }

            pImpl->lastError.clear();
            return true;
        }
        catch (const std::exception &e)
        {
            pImpl->lastError = "Error parsing file: " + std::string(e.what());
            return false;
        }
    }

    bool Parameters::saveToFile(const std::string &filename) const
    {
        std::ofstream file(filename);
        if (!file.is_open())
        {
            pImpl->lastError = "Failed to create file: " + filename;
            return false;
        }

        try
        {
            file << "# CPG Parameters Configuration File\n";
            file << "# Generated automatically\n\n";

            // Save network parameters
            file << "# Network Parameters\n";
            file << "num_oscillators " << pImpl->networkParams.num_oscillators << "\n";
            file << "global_frequency " << pImpl->networkParams.global_frequency << "\n";
            file << "noise_level " << pImpl->networkParams.noise_level << "\n";
            file << "adaptive_coupling " << (pImpl->networkParams.adaptive_coupling ? 1 : 0) << "\n\n";

            // Save gait parameters
            file << "# Gait Parameters\n";
            file << "step_frequency " << pImpl->gaitParams.step_frequency << "\n";
            file << "stance_duration " << pImpl->gaitParams.stance_duration << "\n";
            file << "swing_duration " << pImpl->gaitParams.swing_duration << "\n";
            file << "step_height " << pImpl->gaitParams.step_height << "\n";
            file << "step_length " << pImpl->gaitParams.step_length << "\n\n";

            // Save oscillator parameters
            file << "# Oscillator Parameters\n";
            for (size_t i = 0; i < pImpl->oscillatorParams.size(); ++i)
            {
                const auto &params = pImpl->oscillatorParams[i];
                file << "oscillator " << i << " "
                     << params.frequency << " "
                     << params.amplitude << " "
                     << params.phase << " "
                     << params.offset << " "
                     << params.duty_cycle << "\n";
            }

            return true;
        }
        catch (const std::exception &e)
        {
            pImpl->lastError = "Error writing file: " + std::string(e.what());
            return false;
        }
    }

    // Error Handling
    std::string Parameters::getLastErrorMessage() const
    {
        return pImpl->lastError;
    }

    void Parameters::clearLastError()
    {
        pImpl->lastError.clear();
    }

} // namespace cpg
