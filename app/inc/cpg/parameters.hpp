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

#ifndef CPG_PARAMETERS_HPP
#define CPG_PARAMETERS_HPP

#include <memory>
#include <string>
#include <vector>

/**
 * @brief Central Pattern Generator (CPG) parameter management system
 *
 * This namespace contains classes and functions for managing
 * CPG parameters and configuration.
 */
namespace cpg
{

    //==============================================================================
    // Parameter Structures
    //==============================================================================

    /**
     * @brief Basic oscillator parameters
     */
    struct OscillatorParams
    {
        double frequency = 1.0;  ///< Oscillation frequency in Hz
        double amplitude = 1.0;  ///< Oscillation amplitude
        double phase = 0.0;      ///< Phase offset in radians
        double offset = 0.0;     ///< DC offset
        double duty_cycle = 0.5; ///< Duty cycle (0.0-1.0)

        /**
         * @brief Default constructor with safe defaults
         */
        OscillatorParams() = default;

        /**
         * @brief Parameterized constructor
         */
        OscillatorParams(double freq, double amp, double ph = 0.0, double off = 0.0, double duty = 0.5)
            : frequency(freq), amplitude(amp), phase(ph), offset(off), duty_cycle(duty) {}
    };

    /**
     * @brief Coupling parameters between oscillators
     */
    struct CouplingParams
    {
        double strength = 0.1;     ///< Coupling strength (0.0-1.0)
        double phase_bias = 0.0;   ///< Phase bias in radians
        bool bidirectional = true; ///< Whether coupling is bidirectional

        /**
         * @brief Default constructor
         */
        CouplingParams() = default;

        /**
         * @brief Parameterized constructor
         */
        CouplingParams(double str, double bias = 0.0, bool bidir = true)
            : strength(str), phase_bias(bias), bidirectional(bidir) {}
    };

    /**
     * @brief Gait-specific parameters
     */
    struct GaitParams
    {
        double step_frequency = 1.0;  ///< Step frequency in Hz
        double stance_duration = 0.6; ///< Stance phase duration (0.0-1.0)
        double swing_duration = 0.4;  ///< Swing phase duration (0.0-1.0)
        double step_height = 30.0;    ///< Maximum step height in mm
        double step_length = 60.0;    ///< Step length in mm

        /**
         * @brief Default constructor
         */
        GaitParams() = default;
    };

    /**
     * @brief Network topology parameters
     */
    struct NetworkParams
    {
        size_t num_oscillators = 6;     ///< Number of oscillators in network
        double global_frequency = 1.0;  ///< Global frequency scaling factor
        double noise_level = 0.0;       ///< Noise level (0.0-1.0)
        bool adaptive_coupling = false; ///< Enable adaptive coupling

        /**
         * @brief Default constructor
         */
        NetworkParams() = default;
    };

    //==============================================================================
    // Forward declarations
    //==============================================================================

    // Implementation class (PIMPL idiom)
    class ParametersImpl;

    //==============================================================================
    // Main Parameters Class
    //==============================================================================

    /**
     * @brief CPG parameter manager
     *
     * Manages all parameters for the CPG system including oscillator parameters,
     * coupling parameters, gait parameters, and network topology.
     */
    class Parameters
    {
    public:
        /**
         * @brief Construct a new Parameters object
         */
        Parameters();

        /**
         * @brief Destroy the Parameters object
         */
        ~Parameters();

        // Non-copyable
        Parameters(const Parameters &) = delete;
        Parameters &operator=(const Parameters &) = delete;

        // Move semantics
        Parameters(Parameters &&other) noexcept;
        Parameters &operator=(Parameters &&other) noexcept;

        //--------------------------------------------------------------------------
        // Oscillator Parameter Management
        //--------------------------------------------------------------------------

        /**
         * @brief Set parameters for a specific oscillator
         *
         * @param oscillator_id Oscillator identifier (0-based index)
         * @param params Oscillator parameters to set
         * @return true if parameters were set successfully
         * @return false if oscillator_id is invalid or parameters are invalid
         */
        bool setOscillatorParams(size_t oscillator_id, const OscillatorParams &params);

        /**
         * @brief Get parameters for a specific oscillator
         *
         * @param oscillator_id Oscillator identifier
         * @param[out] params Output parameters
         * @return true if parameters were retrieved successfully
         * @return false if oscillator_id is invalid
         */
        bool getOscillatorParams(size_t oscillator_id, OscillatorParams &params) const;

        /**
         * @brief Set parameters for all oscillators
         *
         * @param params_list Vector of parameters for each oscillator
         * @return true if all parameters were set successfully
         * @return false if vector size doesn't match number of oscillators
         */
        bool setAllOscillatorParams(const std::vector<OscillatorParams> &params_list);

        /**
         * @brief Get parameters for all oscillators
         *
         * @return std::vector<OscillatorParams> Vector of all oscillator parameters
         */
        std::vector<OscillatorParams> getAllOscillatorParams() const;

        //--------------------------------------------------------------------------
        // Coupling Parameter Management
        //--------------------------------------------------------------------------

        /**
         * @brief Set coupling parameters between two oscillators
         *
         * @param from_id Source oscillator ID
         * @param to_id Target oscillator ID
         * @param params Coupling parameters
         * @return true if coupling was set successfully
         * @return false if IDs are invalid or parameters are invalid
         */
        bool setCouplingParams(size_t from_id, size_t to_id, const CouplingParams &params);

        /**
         * @brief Get coupling parameters between two oscillators
         *
         * @param from_id Source oscillator ID
         * @param to_id Target oscillator ID
         * @param[out] params Output coupling parameters
         * @return true if coupling parameters were retrieved successfully
         * @return false if coupling doesn't exist or IDs are invalid
         */
        bool getCouplingParams(size_t from_id, size_t to_id, CouplingParams &params) const;

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
         * @brief Clear all couplings
         */
        void clearAllCouplings();

        //--------------------------------------------------------------------------
        // Gait Parameter Management
        //--------------------------------------------------------------------------

        /**
         * @brief Set gait parameters
         *
         * @param params Gait parameters to set
         * @return true if parameters were set successfully
         * @return false if parameters are invalid
         */
        bool setGaitParams(const GaitParams &params);

        /**
         * @brief Get current gait parameters
         *
         * @return GaitParams Current gait parameters
         */
        GaitParams getGaitParams() const;

        //--------------------------------------------------------------------------
        // Network Parameter Management
        //--------------------------------------------------------------------------

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
        // Validation and Utility
        //--------------------------------------------------------------------------

        /**
         * @brief Validate all current parameters
         *
         * @return true if all parameters are valid
         * @return false if any parameters are invalid
         */
        bool validateAll() const;

        /**
         * @brief Reset all parameters to default values
         */
        void resetToDefaults();

        /**
         * @brief Get the number of oscillators in the network
         *
         * @return size_t Number of oscillators
         */
        size_t getNumOscillators() const;

        //--------------------------------------------------------------------------
        // File I/O
        //--------------------------------------------------------------------------

        /**
         * @brief Load parameters from file
         *
         * @param filename Path to parameter file
         * @return true if loading was successful
         * @return false if loading failed
         */
        bool loadFromFile(const std::string &filename);

        /**
         * @brief Save parameters to file
         *
         * @param filename Path to save parameters
         * @return true if saving was successful
         * @return false if saving failed
         */
        bool saveToFile(const std::string &filename) const;

        //--------------------------------------------------------------------------
        // Error Handling
        //--------------------------------------------------------------------------

        /**
         * @brief Get the last error message
         *
         * @return std::string Error message
         */
        std::string getLastErrorMessage() const;

        /**
         * @brief Clear the last error
         */
        void clearLastError();

    private:
        /**
         * @brief Implementation pointer (PIMPL idiom)
         */
        std::unique_ptr<ParametersImpl> pImpl;
    };

} // namespace cpg

// For backward compatibility, import types into global namespace
using cpg::CouplingParams;
using cpg::GaitParams;
using cpg::NetworkParams;
using cpg::OscillatorParams;
using cpg::Parameters;

#endif /* CPG_PARAMETERS_HPP */
