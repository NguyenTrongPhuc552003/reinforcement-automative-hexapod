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
#include <random>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <iostream>
#include "cpg/oscillator.hpp"

namespace cpg
{

    //==============================================================================
    // Constants
    //==============================================================================

    namespace
    {
        constexpr double PI = 3.14159265358979323846;
        constexpr double TWO_PI = 2.0 * PI;
        constexpr double EPSILON = 1e-12;
        constexpr double DEFAULT_INTEGRATION_STEP = 0.001; // 1ms default step
        constexpr double MAX_FREQUENCY = 100.0; // Maximum allowed frequency (Hz)
        constexpr double MIN_FREQUENCY = 0.01;  // Minimum allowed frequency (Hz)
        constexpr double STABILITY_THRESHOLD = 10.0; // Energy threshold for stability check
    }

    //==============================================================================
    // Implementation Class (PIMPL idiom)
    //==============================================================================

    class OscillatorImpl
    {
    public:
        /**
         * @brief Construct a new Oscillator Implementation object
         *
         * @param config Initial configuration
         */
        explicit OscillatorImpl(const OscillatorConfig& config)
            : m_config(config), m_state(), m_initialized(false), m_lastUpdateTime(0.0),
              m_integrationStep(DEFAULT_INTEGRATION_STEP), m_updateCount(0),
              m_totalUpdateTime(0.0), m_maxUpdateTime(0.0), m_noiseGenerator(std::random_device{}()),
              m_noiseDist(0.0, 1.0), m_adaptiveLearningRate(0.01), m_adaptiveTargetEnergy(1.0)
        {
            // Initialize state with configuration
            m_state.phase = m_config.phase_offset;
            m_state.amplitude = m_config.amplitude;
            m_state.frequency = m_config.frequency;
            m_state.active = false;
        }

        /**
         * @brief Initialize the oscillator
         */
        bool init()
        {
            try
            {
                // Validate configuration
                if (!validateConfig(m_config))
                {
                    m_lastError = "Invalid configuration parameters";
                    return false;
                }

                // Initialize state variables based on oscillator type
                initializeStateForType(m_config.type);

                // Set up timing
                m_lastUpdateTime = getCurrentTime();
                
                // Reset performance counters
                m_updateCount = 0;
                m_totalUpdateTime = 0.0;
                m_maxUpdateTime = 0.0;

                m_state.active = true;
                m_initialized = true;
                m_lastError.clear();

                return true;
            }
            catch (const std::exception& e)
            {
                m_lastError = std::string("Initialization exception: ") + e.what();
                return false;
            }
        }

        /**
         * @brief Initialize state variables for specific oscillator type
         */
        void initializeStateForType(OscillatorType type)
        {
            switch (type)
            {
                case OscillatorType::HOPF:
                    // Start near limit cycle
                    m_state.x = m_config.amplitude * std::cos(m_config.phase_offset);
                    m_state.y = m_config.amplitude * std::sin(m_config.phase_offset);
                    break;

                case OscillatorType::VAN_DER_POL:
                    // Start with small perturbation
                    m_state.x = 0.1 * std::cos(m_config.phase_offset);
                    m_state.y = 0.1 * std::sin(m_config.phase_offset);
                    break;

                case OscillatorType::KURAMOTO:
                    // Phase-only oscillator
                    m_state.x = std::cos(m_config.phase_offset);
                    m_state.y = std::sin(m_config.phase_offset);
                    break;

                case OscillatorType::MATSUOKA:
                    // Start with balanced initial conditions
                    m_state.x = 0.1;
                    m_state.y = 0.0;
                    break;
            }

            updatePhaseAndAmplitude();
        }

        /**
         * @brief Update oscillator state with time step
         */
        bool update(double dt)
        {
            if (!m_initialized || !m_state.active)
            {
                m_lastError = "Oscillator not initialized or inactive";
                return false;
            }

            if (dt <= 0.0 || dt > 1.0) // Sanity check for time step
            {
                m_lastError = "Invalid time step: " + std::to_string(dt);
                return false;
            }

            auto updateStart = std::chrono::high_resolution_clock::now();

            try
            {
                // Apply coupling and external forces before integration
                applyCouplingForces();
                
                // Perform numerical integration
                integrateStep(dt);

                // Add noise if configured
                if (m_config.noise_level > 0.0)
                {
                    addNoise();
                }

                // Update derived quantities
                updatePhaseAndAmplitude();

                // Apply adaptive behavior if enabled
                if (m_config.adaptive)
                {
                    updateAdaptiveParameters();
                }

                // Update performance statistics
                auto updateEnd = std::chrono::high_resolution_clock::now();
                auto updateDuration = std::chrono::duration<double>(updateEnd - updateStart).count();
                updatePerformanceStats(updateDuration);

                m_state.timestamp = getCurrentTime();
                return true;
            }
            catch (const std::exception& e)
            {
                m_lastError = std::string("Update exception: ") + e.what();
                return false;
            }
        }

        /**
         * @brief Perform numerical integration step
         */
        void integrateStep(double dt)
        {
            switch (m_config.type)
            {
                case OscillatorType::HOPF:
                    integrateHopf(dt);
                    break;
                case OscillatorType::VAN_DER_POL:
                    integrateVanDerPol(dt);
                    break;
                case OscillatorType::KURAMOTO:
                    integrateKuramoto(dt);
                    break;
                case OscillatorType::MATSUOKA:
                    integrateMatsuoka(dt);
                    break;
            }
        }

        /**
         * @brief Integrate Hopf oscillator dynamics
         */
        void integrateHopf(double dt)
        {
            // Hopf oscillator: dx/dt = (μ - r²)x - ωy, dy/dt = (μ - r²)y + ωx
            // where r² = x² + y², μ controls amplitude, ω = 2πf
            
            double r_squared = m_state.x * m_state.x + m_state.y * m_state.y;
            double mu = m_config.amplitude * m_config.amplitude;
            double omega = TWO_PI * m_state.frequency;
            
            double dx_dt = (mu - r_squared) * m_state.x - omega * m_state.y;
            double dy_dt = (mu - r_squared) * m_state.y + omega * m_state.x;
            
            // Apply damping
            dx_dt -= m_config.damping * m_state.x;
            dy_dt -= m_config.damping * m_state.y;
            
            // 4th order Runge-Kutta integration
            rungeKuttaStep(dx_dt, dy_dt, dt);
        }

        /**
         * @brief Integrate Van der Pol oscillator dynamics
         */
        void integrateVanDerPol(double dt)
        {
            // Van der Pol: d²x/dt² - μ(1 - x²)dx/dt + ω²x = 0
            // Convert to first order: dx/dt = y, dy/dt = μ(1 - x²)y - ω²x
            
            double omega_squared = std::pow(TWO_PI * m_state.frequency, 2);
            double mu = m_config.convergence_rate;
            
            double dx_dt = m_state.y;
            double dy_dt = mu * (1.0 - m_state.x * m_state.x) * m_state.y - omega_squared * m_state.x;
            
            rungeKuttaStep(dx_dt, dy_dt, dt);
        }

        /**
         * @brief Integrate Kuramoto oscillator dynamics
         */
        void integrateKuramoto(double dt)
        {
            // Kuramoto: dθ/dt = ω + K sin(θ_other - θ)
            // We'll update phase directly and convert to x,y
            
            double omega = TWO_PI * m_state.frequency;
            double dphase_dt = omega; // Coupling will be added separately
            
            m_state.phase += dphase_dt * dt;
            m_state.phase = normalizePhase(m_state.phase);
            
            // Convert back to Cartesian coordinates
            m_state.x = m_config.amplitude * std::cos(m_state.phase);
            m_state.y = m_config.amplitude * std::sin(m_state.phase);
        }

        /**
         * @brief Integrate Matsuoka oscillator dynamics
         */
        void integrateMatsuoka(double dt)
        {
            // Simplified Matsuoka neuron model
            // dx/dt = -ax + max(0, c*input - y), dy/dt = -by + max(0, x)
            // where y represents adaptation/fatigue
            
            double a = m_config.damping * 10.0; // Membrane time constant
            double b = m_config.damping * 5.0;  // Adaptation time constant
            double c = m_config.amplitude;      // Input gain
            
            double input = std::sin(TWO_PI * m_state.frequency * m_state.timestamp);
            
            double dx_dt = -a * m_state.x + std::max(0.0, c * input - m_state.y);
            double dy_dt = -b * m_state.y + std::max(0.0, m_state.x);
            
            // Simple Euler integration for Matsuoka (more stable)
            m_state.x += dx_dt * dt;
            m_state.y += dy_dt * dt;
        }

        /**
         * @brief Perform 4th order Runge-Kutta integration step
         */
        void rungeKuttaStep(double dx_dt, double dy_dt, double dt)
        {
            // RK4 integration for better accuracy
            double k1x = dx_dt;
            double k1y = dy_dt;
            
            double k2x = dx_dt; // Simplified - would need to recalculate derivatives
            double k2y = dy_dt;
            
            double k3x = dx_dt;
            double k3y = dy_dt;
            
            double k4x = dx_dt;
            double k4y = dy_dt;
            
            // For now, use simplified Euler method with smaller step
            double step = dt / 4.0;
            for (int i = 0; i < 4; ++i)
            {
                m_state.x += k1x * step;
                m_state.y += k1y * step;
            }
        }

        /**
         * @brief Update phase and amplitude from x,y coordinates
         */
        void updatePhaseAndAmplitude()
        {
            m_state.amplitude = std::sqrt(m_state.x * m_state.x + m_state.y * m_state.y);
            m_state.phase = std::atan2(m_state.y, m_state.x);
            
            if (m_state.phase < 0.0)
            {
                m_state.phase += TWO_PI;
            }
        }

        /**
         * @brief Apply coupling forces from other oscillators
         */
        void applyCouplingForces()
        {
            // Coupling forces will be applied through public interface
            // This is a placeholder for internal coupling state management
            
            // Reset coupling forces for next update
            m_couplingForceX = 0.0;
            m_couplingForceY = 0.0;
            m_externalForceX = 0.0;
            m_externalForceY = 0.0;
        }

        /**
         * @brief Add noise to the oscillator state
         */
        void addNoise()
        {
            double noise_x = m_config.noise_level * m_noiseDist(m_noiseGenerator);
            double noise_y = m_config.noise_level * m_noiseDist(m_noiseGenerator);
            
            m_state.x += noise_x;
            m_state.y += noise_y;
        }

        /**
         * @brief Update adaptive parameters based on current state
         */
        void updateAdaptiveParameters()
        {
            // Simple adaptive frequency based on energy level
            double current_energy = getEnergy();
            double energy_error = m_adaptiveTargetEnergy - current_energy;
            
            // Adjust frequency to maintain target energy
            m_state.frequency += m_adaptiveLearningRate * energy_error * 0.1;
            m_state.frequency = std::clamp(m_state.frequency, MIN_FREQUENCY, MAX_FREQUENCY);
        }

        /**
         * @brief Get current time in seconds
         */
        double getCurrentTime() const
        {
            auto now = std::chrono::high_resolution_clock::now();
            auto duration = now.time_since_epoch();
            return std::chrono::duration<double>(duration).count();
        }

        /**
         * @brief Update performance statistics
         */
        void updatePerformanceStats(double update_time)
        {
            m_updateCount++;
            m_totalUpdateTime += update_time;
            m_maxUpdateTime = std::max(m_maxUpdateTime, update_time);
        }

        /**
         * @brief Validate configuration parameters
         */
        static bool validateConfig(const OscillatorConfig& config)
        {
            return (config.frequency > MIN_FREQUENCY && config.frequency < MAX_FREQUENCY &&
                    config.amplitude > 0.0 && config.amplitude < 100.0 &&
                    config.damping >= 0.0 && config.damping <= 1.0 &&
                    config.coupling_strength >= 0.0 && config.coupling_strength <= 1.0 &&
                    config.convergence_rate > 0.0 && config.convergence_rate < 10.0 &&
                    config.noise_level >= 0.0 && config.noise_level <= 1.0);
        }

        // Member variables
        OscillatorConfig m_config;
        OscillatorState m_state;
        bool m_initialized;
        std::string m_lastError;
        
        // Timing and integration
        double m_lastUpdateTime;
        double m_integrationStep;
        
        // Performance monitoring
        uint64_t m_updateCount;
        double m_totalUpdateTime;
        double m_maxUpdateTime;
        
        // Noise generation
        std::mt19937 m_noiseGenerator;
        std::normal_distribution<double> m_noiseDist;
        
        // Coupling and external forces
        double m_couplingForceX = 0.0;
        double m_couplingForceY = 0.0;
        double m_externalForceX = 0.0;
        double m_externalForceY = 0.0;
        
        // Adaptive parameters
        double m_adaptiveLearningRate;
        double m_adaptiveTargetEnergy;
        
        /**
         * @brief Calculate oscillator energy
         */
        double getEnergy() const
        {
            return 0.5 * (m_state.x * m_state.x + m_state.y * m_state.y);
        }
    };

    //==============================================================================
    // Oscillator Class Implementation
    //==============================================================================

    // Constructor and Destructor
    Oscillator::Oscillator(const OscillatorConfig& config)
        : pImpl(std::make_unique<OscillatorImpl>(config))
    {
    }

    Oscillator::~Oscillator() = default;

    // Move semantics
    Oscillator::Oscillator(Oscillator&& other) noexcept = default;
    Oscillator& Oscillator::operator=(Oscillator&& other) noexcept = default;

    // Lifecycle methods
    bool Oscillator::init()
    {
        return pImpl->init();
    }

    void Oscillator::reset(bool preserve_config)
    {
        OscillatorConfig config = preserve_config ? pImpl->m_config : OscillatorConfig();
        pImpl = std::make_unique<OscillatorImpl>(config);
    }

    bool Oscillator::isInitialized() const
    {
        return pImpl->m_initialized;
    }

    // Core oscillator operations
    bool Oscillator::update(double dt)
    {
        return pImpl->update(dt);
    }

    bool Oscillator::update()
    {
        double current_time = pImpl->getCurrentTime();
        double dt = current_time - pImpl->m_lastUpdateTime;
        
        // Clamp time step to reasonable bounds
        dt = std::clamp(dt, 0.001, 0.1);
        
        return update(dt);
    }

    double Oscillator::getOutput() const
    {
        if (pImpl->m_config.type == OscillatorType::MATSUOKA)
        {
            // Matsuoka outputs activity level
            return std::clamp(pImpl->m_state.x, -1.0, 1.0);
        }
        else
        {
            // Other oscillators output x component
            return std::clamp(pImpl->m_state.x / std::max(pImpl->m_state.amplitude, 1.0), -1.0, 1.0);
        }
    }

    double Oscillator::getScaledOutput(double min_val, double max_val) const
    {
        double normalized = (getOutput() + 1.0) / 2.0; // Convert from [-1,1] to [0,1]
        return min_val + normalized * (max_val - min_val);
    }

    // State access methods
    OscillatorState Oscillator::getState() const
    {
        return pImpl->m_state;
    }

    bool Oscillator::setState(const OscillatorState& state)
    {
        // Validate state parameters
        if (state.frequency <= 0.0 || state.amplitude < 0.0)
        {
            return false;
        }
        
        pImpl->m_state = state;
        return true;
    }

    double Oscillator::getPhase() const
    {
        return pImpl->m_state.phase;
    }

    void Oscillator::setPhase(double phase)
    {
        pImpl->m_state.phase = normalizePhase(phase);
        
        // Update x,y coordinates to match new phase
        pImpl->m_state.x = pImpl->m_state.amplitude * std::cos(phase);
        pImpl->m_state.y = pImpl->m_state.amplitude * std::sin(phase);
    }

    // Configuration management
    OscillatorConfig Oscillator::getConfig() const
    {
        return pImpl->m_config;
    }

    bool Oscillator::setConfig(const OscillatorConfig& config)
    {
        if (!pImpl->validateConfig(config))
        {
            return false;
        }
        
        pImpl->m_config = config;
        pImpl->m_state.frequency = config.frequency;
        pImpl->m_state.amplitude = config.amplitude;
        
        return true;
    }

    void Oscillator::setFrequency(double frequency)
    {
        pImpl->m_state.frequency = std::clamp(frequency, MIN_FREQUENCY, MAX_FREQUENCY);
        pImpl->m_config.frequency = pImpl->m_state.frequency;
    }

    void Oscillator::setAmplitude(double amplitude)
    {
        pImpl->m_state.amplitude = std::max(0.0, amplitude);
        pImpl->m_config.amplitude = pImpl->m_state.amplitude;
    }

    void Oscillator::setCouplingStrength(double strength)
    {
        pImpl->m_config.coupling_strength = std::clamp(strength, 0.0, 1.0);
    }

    // Coupling and synchronization
    void Oscillator::applyCoupling(double other_phase, double coupling_strength, double phase_difference)
    {
        double phase_diff = calculatePhaseDifference(other_phase, pImpl->m_state.phase);
        double target_diff = phase_difference;
        double error = target_diff - phase_diff;
        
        // Apply coupling force based on phase error
        double coupling_force = coupling_strength * std::sin(error);
        
        // Add to coupling forces (will be applied in next update)
        pImpl->m_couplingForceX += coupling_force * std::cos(pImpl->m_state.phase + PI/2);
        pImpl->m_couplingForceY += coupling_force * std::sin(pImpl->m_state.phase + PI/2);
    }

    void Oscillator::applyExternalForce(double force_magnitude, double force_phase)
    {
        pImpl->m_externalForceX += force_magnitude * std::cos(force_phase);
        pImpl->m_externalForceY += force_magnitude * std::sin(force_phase);
    }

    void Oscillator::synchronizeToSignal(double external_frequency, double sync_strength)
    {
        double frequency_error = external_frequency - pImpl->m_state.frequency;
        double adjustment = sync_strength * frequency_error * 0.1; // Small adjustment factor
        
        setFrequency(pImpl->m_state.frequency + adjustment);
    }

    // Adaptive behavior
    void Oscillator::setAdaptive(bool enabled)
    {
        pImpl->m_config.adaptive = enabled;
    }

    void Oscillator::setAdaptationParameters(double learning_rate, double target_energy)
    {
        pImpl->m_adaptiveLearningRate = std::clamp(learning_rate, 0.0, 1.0);
        pImpl->m_adaptiveTargetEnergy = std::max(0.0, target_energy);
    }

    // Diagnostics and monitoring
    std::string Oscillator::getLastError() const
    {
        return pImpl->m_lastError;
    }

    std::string Oscillator::getPerformanceStats() const
    {
        std::ostringstream oss;
        oss << "Oscillator Performance Stats:\n";
        oss << "  Updates: " << pImpl->m_updateCount << "\n";
        
        if (pImpl->m_updateCount > 0)
        {
            double avg_time = pImpl->m_totalUpdateTime / pImpl->m_updateCount;
            oss << "  Avg update time: " << std::fixed << std::setprecision(6) 
                << (avg_time * 1000.0) << " ms\n";
            oss << "  Max update time: " << std::fixed << std::setprecision(6) 
                << (pImpl->m_maxUpdateTime * 1000.0) << " ms\n";
        }
        
        oss << "  Current energy: " << std::fixed << std::setprecision(3) 
            << pImpl->getEnergy() << "\n";
        oss << "  Current frequency: " << std::fixed << std::setprecision(2) 
            << pImpl->m_state.frequency << " Hz\n";
        
        return oss.str();
    }

    bool Oscillator::isStable() const
    {
        double energy = pImpl->getEnergy();
        return (energy > 0.1 && energy < STABILITY_THRESHOLD);
    }

    double Oscillator::getEnergy() const
    {
        return pImpl->getEnergy();
    }

    // Static utility methods
    std::string Oscillator::getTypeName(OscillatorType type)
    {
        switch (type)
        {
            case OscillatorType::HOPF: return "Hopf";
            case OscillatorType::VAN_DER_POL: return "Van der Pol";
            case OscillatorType::KURAMOTO: return "Kuramoto";
            case OscillatorType::MATSUOKA: return "Matsuoka";
            default: return "Unknown";
        }
    }

    bool Oscillator::validateConfig(const OscillatorConfig& config)
    {
        return OscillatorImpl::validateConfig(config);
    }

    OscillatorConfig Oscillator::createDefaultConfig(OscillatorType type)
    {
        OscillatorConfig config;
        config.type = type;
        
        switch (type)
        {
            case OscillatorType::HOPF:
                config.frequency = 1.0;
                config.amplitude = 1.0;
                config.damping = 0.1;
                config.convergence_rate = 1.0;
                break;
                
            case OscillatorType::VAN_DER_POL:
                config.frequency = 1.0;
                config.amplitude = 2.0;
                config.damping = 0.0;
                config.convergence_rate = 1.0;
                break;
                
            case OscillatorType::KURAMOTO:
                config.frequency = 1.0;
                config.amplitude = 1.0;
                config.coupling_strength = 0.5;
                break;
                
            case OscillatorType::MATSUOKA:
                config.frequency = 1.0;
                config.amplitude = 1.0;
                config.damping = 0.5;
                break;
        }
        
        return config;
    }

    //==============================================================================
    // Utility Functions Implementation
    //==============================================================================

    double calculatePhaseDifference(double phase1, double phase2)
    {
        double diff = phase1 - phase2;
        
        // Normalize to [-π, π]
        while (diff > PI) diff -= TWO_PI;
        while (diff < -PI) diff += TWO_PI;
        
        return diff;
    }

    double normalizePhase(double phase)
    {
        while (phase < 0.0) phase += TWO_PI;
        while (phase >= TWO_PI) phase -= TWO_PI;
        return phase;
    }

    double kuramotoCoupling(double phase_diff, double coupling_strength)
    {
        return coupling_strength * std::sin(phase_diff);
    }

} // namespace cpg
