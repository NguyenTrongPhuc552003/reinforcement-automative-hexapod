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

#ifndef CPG_OSCILLATOR_HPP
#define CPG_OSCILLATOR_HPP

#include <memory>
#include <vector>
#include <functional>
#include "cpg/parameters.hpp"

/**
 * @brief Central Pattern Generator (CPG) oscillator implementation
 *
 * This namespace contains classes and functions for implementing individual
 * oscillators within the CPG network, based on Hopf oscillator dynamics.
 */
namespace cpg
{

    //==============================================================================
    // Oscillator State and Output
    //==============================================================================

    /**
     * @brief Oscillator state vector
     *
     * Represents the current state of a single oscillator in the CPG network.
     */
    struct OscillatorState
    {
        double x;         ///< X component (position-like)
        double y;         ///< Y component (velocity-like)
        double phase;     ///< Current phase (radians)
        double amplitude; ///< Current amplitude
        double frequency; ///< Current frequency (Hz)

        /**
         * @brief Default constructor
         */
        OscillatorState()
            : x(0.0), y(0.0), phase(0.0), amplitude(1.0), frequency(1.0)
        {
        }

        /**
         * @brief Parameterized constructor
         */
        OscillatorState(double x_val, double y_val, double ph = 0.0,
                        double amp = 1.0, double freq = 1.0)
            : x(x_val), y(y_val), phase(ph), amplitude(amp), frequency(freq)
        {
        }
    };

    /**
     * @brief Oscillator output structure
     *
     * Contains the computed output values from an oscillator.
     */
    struct OscillatorOutput
    {
        double raw_output;    ///< Raw oscillator output
        double scaled_output; ///< Scaled and offset output
        double phase_output;  ///< Phase-based output
        bool is_stance_phase; ///< Whether in stance phase (for gait)

        /**
         * @brief Default constructor
         */
        OscillatorOutput()
            : raw_output(0.0), scaled_output(0.0), phase_output(0.0), is_stance_phase(false)
        {
        }
    };

    //==============================================================================
    // Forward declarations
    //==============================================================================

    // Implementation class (PIMPL idiom)
    class OscillatorImpl;

    //==============================================================================
    // Main Oscillator Class
    //==============================================================================

    /**
     * @brief Individual CPG oscillator
     *
     * Implements a single Hopf oscillator with configurable parameters and
     * coupling capabilities for use in CPG networks.
     */
    class Oscillator
    {
    public:
        /**
         * @brief Construct a new Oscillator object
         *
         * @param id Unique identifier for this oscillator
         * @param params Initial parameters for the oscillator
         */
        Oscillator(size_t id, const OscillatorParams &params = OscillatorParams());

        /**
         * @brief Destroy the Oscillator object
         */
        ~Oscillator();

        // Non-copyable
        Oscillator(const Oscillator &) = delete;
        Oscillator &operator=(const Oscillator &) = delete;

        // Move semantics
        Oscillator(Oscillator &&other) noexcept;
        Oscillator &operator=(Oscillator &&other) noexcept;

        //--------------------------------------------------------------------------
        // Basic Properties
        //--------------------------------------------------------------------------

        /**
         * @brief Get the oscillator ID
         *
         * @return size_t Oscillator identifier
         */
        size_t getId() const;

        /**
         * @brief Set oscillator parameters
         *
         * @param params New parameters for the oscillator
         * @return true if parameters were set successfully
         * @return false if parameters are invalid
         */
        bool setParameters(const OscillatorParams &params);

        /**
         * @brief Get current oscillator parameters
         *
         * @return OscillatorParams Current parameters
         */
        OscillatorParams getParameters() const;

        //--------------------------------------------------------------------------
        // State Management
        //--------------------------------------------------------------------------

        /**
         * @brief Get current oscillator state
         *
         * @return OscillatorState Current state
         */
        OscillatorState getState() const;

        /**
         * @brief Set oscillator state
         *
         * @param state New state to set
         * @return true if state was set successfully
         * @return false if state is invalid
         */
        bool setState(const OscillatorState &state);

        /**
         * @brief Reset oscillator to initial state
         *
         * @param random_phase If true, use random initial phase
         */
        void reset(bool random_phase = false);

        //--------------------------------------------------------------------------
        // Integration and Update
        //--------------------------------------------------------------------------

        /**
         * @brief Update oscillator state by one time step
         *
         * Performs numerical integration using 4th-order Runge-Kutta method.
         *
         * @param dt Time step size (seconds)
         * @param coupling_input External coupling input
         * @return true if update was successful
         * @return false if update failed
         */
        bool update(double dt, double coupling_input = 0.0);

        /**
         * @brief Get current output values
         *
         * @return OscillatorOutput Current output structure
         */
        OscillatorOutput getOutput() const;

        /**
         * @brief Get raw output value
         *
         * @return double Raw oscillator output [-amplitude, +amplitude]
         */
        double getRawOutput() const;

        /**
         * @brief Get scaled output value
         *
         * @return double Scaled output including offset and amplitude scaling
         */
        double getScaledOutput() const;

        /**
         * @brief Get phase-based output
         *
         * @return double Phase output [0, 1] representing gait phase
         */
        double getPhaseOutput() const;

        //--------------------------------------------------------------------------
        // Phase and Frequency Control
        //--------------------------------------------------------------------------

        /**
         * @brief Get current phase
         *
         * @return double Current phase in radians [0, 2π)
         */
        double getPhase() const;

        /**
         * @brief Set phase directly
         *
         * @param phase New phase in radians
         * @return true if phase was set successfully
         * @return false if phase is invalid
         */
        bool setPhase(double phase);

        /**
         * @brief Get current frequency
         *
         * @return double Current frequency in Hz
         */
        double getFrequency() const;

        /**
         * @brief Set frequency
         *
         * @param frequency New frequency in Hz
         * @return true if frequency was set successfully
         * @return false if frequency is invalid
         */
        bool setFrequency(double frequency);

        /**
         * @brief Get current amplitude
         *
         * @return double Current amplitude
         */
        double getAmplitude() const;

        /**
         * @brief Set amplitude
         *
         * @param amplitude New amplitude
         * @return true if amplitude was set successfully
         * @return false if amplitude is invalid
         */
        bool setAmplitude(double amplitude);

        //--------------------------------------------------------------------------
        // Gait-specific Methods
        //--------------------------------------------------------------------------

        /**
         * @brief Check if oscillator is in stance phase
         *
         * @return true if in stance phase
         * @return false if in swing phase
         */
        bool isStancePhase() const;

        /**
         * @brief Get stance phase progress
         *
         * @return double Progress through stance phase [0.0, 1.0]
         */
        double getStanceProgress() const;

        /**
         * @brief Get swing phase progress
         *
         * @return double Progress through swing phase [0.0, 1.0]
         */
        double getSwingProgress() const;

        //--------------------------------------------------------------------------
        // Coupling Interface
        //--------------------------------------------------------------------------

        /**
         * @brief Add coupling input from another oscillator
         *
         * @param coupling_strength Strength of coupling [0.0, 1.0]
         * @param phase_difference Phase difference from coupling oscillator
         * @param weight Connection weight
         */
        void addCouplingInput(double coupling_strength, double phase_difference, double weight = 1.0);

        /**
         * @brief Clear all coupling inputs
         */
        void clearCouplingInputs();

        /**
         * @brief Get total coupling input
         *
         * @return double Sum of all coupling inputs
         */
        double getTotalCouplingInput() const;

        //--------------------------------------------------------------------------
        // Validation and Diagnostics
        //--------------------------------------------------------------------------

        /**
         * @brief Validate current oscillator state
         *
         * @return true if state is valid
         * @return false if state is invalid or unstable
         */
        bool validateState() const;

        /**
         * @brief Check if oscillator is stable
         *
         * @return true if oscillator is in stable limit cycle
         * @return false if oscillator is unstable or diverging
         */
        bool isStable() const;

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
        // Debugging and Analysis
        //--------------------------------------------------------------------------

        /**
         * @brief Get oscillator statistics
         *
         * @return std::string Formatted statistics string
         */
        std::string getStatistics() const;

        /**
         * @brief Enable/disable debug output
         *
         * @param enable True to enable debug output
         */
        void setDebugMode(bool enable);

    private:
        /**
         * @brief Implementation pointer (PIMPL idiom)
         */
        std::unique_ptr<OscillatorImpl> pImpl;
    };

    //==============================================================================
    // Utility Functions
    //==============================================================================

    /**
     * @brief Oscillator utility functions
     */
    namespace oscillator_utils
    {
        /**
         * @brief Normalize phase to [0, 2π) range
         *
         * @param phase Input phase in radians
         * @return double Normalized phase
         */
        double normalizePhase(double phase);

        /**
         * @brief Compute phase difference between two phases
         *
         * @param phase1 First phase
         * @param phase2 Second phase
         * @return double Phase difference in range [-π, π]
         */
        double phaseDifference(double phase1, double phase2);

        /**
         * @brief Convert phase to gait progress [0, 1]
         *
         * @param phase Phase in radians
         * @param duty_cycle Duty cycle [0, 1]
         * @return double Gait progress
         */
        double phaseToGaitProgress(double phase, double duty_cycle);

        /**
         * @brief Check if phase is in stance portion of gait
         *
         * @param phase Phase in radians
         * @param duty_cycle Duty cycle [0, 1]
         * @return true if in stance phase
         */
        bool isStancePhase(double phase, double duty_cycle);

        /**
         * @brief Validate oscillator parameters
         *
         * @param params Parameters to validate
         * @return true if parameters are valid
         */
        bool validateParameters(const OscillatorParams &params);

        /**
         * @brief Create default parameters for specific gait
         *
         * @param gait_type Type of gait ("tripod", "wave", "ripple")
         * @param leg_id Leg identifier (0-5)
         * @return OscillatorParams Default parameters for the gait
         */
        OscillatorParams createGaitParameters(const std::string &gait_type, size_t leg_id);
    }

} // namespace cpg

#endif // CPG_OSCILLATOR_HPP
