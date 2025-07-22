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
#include <string>
#include <chrono>
#include <cstdint>

/**
 * @brief Central Pattern Generator (CPG) system for hexapod locomotion
 *
 * This namespace contains classes and functions for implementing
 * neural oscillators that generate rhythmic patterns for robot locomotion.
 */
namespace cpg
{

    //==============================================================================
    // Constants and Types
    //==============================================================================

    /**
     * @brief Oscillator types supported by the CPG system
     */
    enum class OscillatorType : uint8_t
    {
        HOPF,           ///< Hopf oscillator (stable limit cycle)
        VAN_DER_POL,    ///< Van der Pol oscillator (self-sustaining)
        KURAMOTO,       ///< Kuramoto oscillator (phase coupling)
        MATSUOKA        ///< Matsuoka oscillator (mutual inhibition)
    };

    /**
     * @brief Oscillator state information
     */
    struct OscillatorState
    {
        double x;           ///< X component (position/activity)
        double y;           ///< Y component (velocity/recovery)
        double phase;       ///< Current phase (0-2π)
        double amplitude;   ///< Current amplitude
        double frequency;   ///< Current frequency (Hz)
        double timestamp;   ///< Last update timestamp
        bool active;        ///< Whether oscillator is active

        /**
         * @brief Default constructor with safe initial values
         */
        OscillatorState() : x(0.0), y(0.0), phase(0.0), amplitude(1.0), 
                           frequency(1.0), timestamp(0.0), active(false) {}
    };

    /**
     * @brief Oscillator configuration parameters
     */
    struct OscillatorConfig
    {
        OscillatorType type;        ///< Type of oscillator
        double frequency;           ///< Base frequency in Hz
        double amplitude;           ///< Target amplitude
        double damping;             ///< Damping coefficient (0.0-1.0)
        double coupling_strength;   ///< Coupling strength with other oscillators
        double phase_offset;        ///< Initial phase offset in radians
        double convergence_rate;    ///< Rate of convergence to limit cycle
        double noise_level;         ///< Noise injection level (0.0-1.0)
        bool adaptive;              ///< Enable adaptive frequency adjustment
        
        /**
         * @brief Default constructor with sensible defaults
         */
        OscillatorConfig() : type(OscillatorType::HOPF), frequency(1.0), amplitude(1.0),
                           damping(0.1), coupling_strength(0.5), phase_offset(0.0),
                           convergence_rate(1.0), noise_level(0.0), adaptive(false) {}
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
     * @brief Neural oscillator for Central Pattern Generation
     *
     * Implements various types of neural oscillators that can generate
     * rhythmic patterns for robot locomotion. Uses PIMPL idiom for
     * implementation hiding and better compilation times.
     */
    class Oscillator
    {
    public:
        /**
         * @brief Construct a new Oscillator object
         *
         * @param config Configuration parameters for the oscillator
         */
        explicit Oscillator(const OscillatorConfig& config = OscillatorConfig());

        /**
         * @brief Destroy the Oscillator object
         *
         * Ensures proper cleanup of internal resources
         */
        ~Oscillator();

        // Non-copyable but movable for performance
        Oscillator(const Oscillator&) = delete;
        Oscillator& operator=(const Oscillator&) = delete;
        Oscillator(Oscillator&& other) noexcept;
        Oscillator& operator=(Oscillator&& other) noexcept;

        //--------------------------------------------------------------------------
        // Lifecycle Methods
        //--------------------------------------------------------------------------

        /**
         * @brief Initialize the oscillator
         *
         * Sets up internal state and prepares for operation
         *
         * @return true if initialization successful
         * @return false if initialization failed (check getLastError())
         */
        bool init();

        /**
         * @brief Reset the oscillator to initial state
         *
         * @param preserve_config Whether to preserve current configuration
         */
        void reset(bool preserve_config = true);

        /**
         * @brief Check if oscillator is properly initialized
         *
         * @return true if oscillator is ready for operation
         * @return false if oscillator needs initialization
         */
        bool isInitialized() const;

        //--------------------------------------------------------------------------
        // Core Oscillator Operations
        //--------------------------------------------------------------------------

        /**
         * @brief Update the oscillator state
         *
         * Advances the oscillator by one time step using numerical integration
         *
         * @param dt Time step in seconds
         * @return true if update successful
         * @return false if update failed
         */
        bool update(double dt);

        /**
         * @brief Update with current system time
         *
         * Automatically calculates time step from last update
         *
         * @return true if update successful
         * @return false if update failed
         */
        bool update();

        /**
         * @brief Get the current output value
         *
         * @return double Current oscillator output (-1.0 to 1.0)
         */
        double getOutput() const;

        /**
         * @brief Get the normalized output value
         *
         * @param min_val Minimum output value
         * @param max_val Maximum output value
         * @return double Scaled output value
         */
        double getScaledOutput(double min_val, double max_val) const;

        //--------------------------------------------------------------------------
        // State Access
        //--------------------------------------------------------------------------

        /**
         * @brief Get the current oscillator state
         *
         * @return OscillatorState Current internal state
         */
        OscillatorState getState() const;

        /**
         * @brief Set the oscillator state
         *
         * @param state New state to set
         * @return true if state was set successfully
         * @return false if state is invalid
         */
        bool setState(const OscillatorState& state);

        /**
         * @brief Get the current phase in radians
         *
         * @return double Phase value (0 to 2π)
         */
        double getPhase() const;

        /**
         * @brief Set the current phase
         *
         * @param phase Phase value in radians
         */
        void setPhase(double phase);

        //--------------------------------------------------------------------------
        // Configuration Management
        //--------------------------------------------------------------------------

        /**
         * @brief Get the current configuration
         *
         * @return OscillatorConfig Current configuration parameters
         */
        OscillatorConfig getConfig() const;

        /**
         * @brief Update configuration parameters
         *
         * @param config New configuration to apply
         * @return true if configuration was applied successfully
         * @return false if configuration is invalid
         */
        bool setConfig(const OscillatorConfig& config);

        /**
         * @brief Set the oscillator frequency
         *
         * @param frequency New frequency in Hz (must be positive)
         */
        void setFrequency(double frequency);

        /**
         * @brief Set the oscillator amplitude
         *
         * @param amplitude New amplitude (must be positive)
         */
        void setAmplitude(double amplitude);

        /**
         * @brief Set the coupling strength
         *
         * @param strength Coupling strength (0.0 to 1.0)
         */
        void setCouplingStrength(double strength);

        //--------------------------------------------------------------------------
        // Coupling and Synchronization
        //--------------------------------------------------------------------------

        /**
         * @brief Apply coupling force from another oscillator
         *
         * @param other_phase Phase of the coupled oscillator
         * @param coupling_strength Strength of the coupling (0.0 to 1.0)
         * @param phase_difference Desired phase difference in radians
         */
        void applyCoupling(double other_phase, double coupling_strength, 
                          double phase_difference = 0.0);

        /**
         * @brief Apply external driving force
         *
         * @param force_magnitude Magnitude of external force
         * @param force_phase Phase of external force
         */
        void applyExternalForce(double force_magnitude, double force_phase);

        /**
         * @brief Synchronize to an external signal
         *
         * @param external_frequency Target frequency to synchronize to
         * @param sync_strength Synchronization strength (0.0 to 1.0)
         */
        void synchronizeToSignal(double external_frequency, double sync_strength);

        //--------------------------------------------------------------------------
        // Adaptive Behavior
        //--------------------------------------------------------------------------

        /**
         * @brief Enable or disable adaptive frequency adjustment
         *
         * @param enabled Whether to enable adaptive behavior
         */
        void setAdaptive(bool enabled);

        /**
         * @brief Set adaptation parameters
         *
         * @param learning_rate Rate of frequency adaptation (0.0 to 1.0)
         * @param target_energy Target energy level for adaptation
         */
        void setAdaptationParameters(double learning_rate, double target_energy);

        //--------------------------------------------------------------------------
        // Diagnostics and Monitoring
        //--------------------------------------------------------------------------

        /**
         * @brief Get the last error message
         *
         * @return std::string Description of last error
         */
        std::string getLastError() const;

        /**
         * @brief Get oscillator performance statistics
         *
         * @return std::string Formatted performance information
         */
        std::string getPerformanceStats() const;

        /**
         * @brief Check oscillator stability
         *
         * @return true if oscillator is in stable limit cycle
         * @return false if oscillator is unstable or diverging
         */
        bool isStable() const;

        /**
         * @brief Get energy level of the oscillator
         *
         * @return double Current energy level
         */
        double getEnergy() const;

        //--------------------------------------------------------------------------
        // Utility Methods
        //--------------------------------------------------------------------------

        /**
         * @brief Get string representation of oscillator type
         *
         * @param type Oscillator type enum
         * @return std::string Human-readable type name
         */
        static std::string getTypeName(OscillatorType type);

        /**
         * @brief Validate configuration parameters
         *
         * @param config Configuration to validate
         * @return true if configuration is valid
         * @return false if configuration contains invalid values
         */
        static bool validateConfig(const OscillatorConfig& config);

        /**
         * @brief Create default configuration for a specific type
         *
         * @param type Oscillator type
         * @return OscillatorConfig Default configuration for the type
         */
        static OscillatorConfig createDefaultConfig(OscillatorType type);

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
     * @brief Calculate phase difference between two oscillators
     *
     * @param phase1 First oscillator phase
     * @param phase2 Second oscillator phase
     * @return double Phase difference in range [-π, π]
     */
    double calculatePhaseDifference(double phase1, double phase2);

    /**
     * @brief Normalize phase to [0, 2π] range
     *
     * @param phase Phase value to normalize
     * @return double Normalized phase value
     */
    double normalizePhase(double phase);

    /**
     * @brief Calculate coupling force using Kuramoto model
     *
     * @param phase_diff Phase difference between oscillators
     * @param coupling_strength Strength of coupling
     * @return double Coupling force magnitude
     */
    double kuramotoCoupling(double phase_diff, double coupling_strength);

} // namespace cpg

// For backward compatibility, import main types into global namespace
using cpg::Oscillator;
using cpg::OscillatorConfig;
using cpg::OscillatorState;
using cpg::OscillatorType;

#endif /* CPG_OSCILLATOR_HPP */
