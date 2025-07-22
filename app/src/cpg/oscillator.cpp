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
#include <random>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include "cpg/oscillator.hpp"

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

        // Numerical integration constants
        constexpr double MIN_TIME_STEP = 1e-6;
        constexpr double MAX_TIME_STEP = 0.1;

        // Stability thresholds
        constexpr double MAX_AMPLITUDE_THRESHOLD = 100.0;
        constexpr double MIN_AMPLITUDE_THRESHOLD = 1e-6;
        constexpr double CONVERGENCE_THRESHOLD = 1e-3;

        // Random number generator for initial conditions
        thread_local std::random_device rd;
        thread_local std::mt19937 gen(rd());
        thread_local std::uniform_real_distribution<double> phase_dist(0.0, TWO_PI);
    }

    //==============================================================================
    // Implementation Class (PIMPL idiom)
    //==============================================================================

    class OscillatorImpl
    {
    public:
        /**
         * @brief Construct a new Oscillator Implementation object
         */
        OscillatorImpl(size_t id, const OscillatorParams &params)
            : m_id(id),
              m_params(params),
              m_state(),
              m_totalCouplingInput(0.0),
              m_debugMode(false),
              m_updateCount(0),
              m_lastErrorMessage("")
        {
            // Initialize state with small random perturbation
            reset(true);
        }

        /**
         * @brief Reset oscillator to initial state
         */
        void reset(bool random_phase)
        {
            if (random_phase)
            {
                m_state.phase = phase_dist(gen);
            }
            else
            {
                m_state.phase = m_params.phase;
            }

            // Initialize x, y based on phase and amplitude
            m_state.x = m_params.amplitude * std::cos(m_state.phase);
            m_state.y = m_params.amplitude * std::sin(m_state.phase);
            m_state.amplitude = m_params.amplitude;
            m_state.frequency = m_params.frequency;

            m_totalCouplingInput = 0.0;
            m_updateCount = 0;
            clearLastError();
        }

        /**
         * @brief Update oscillator state using 4th-order Runge-Kutta integration
         */
        bool update(double dt, double coupling_input)
        {
            // Validate time step
            if (dt <= 0.0 || dt > MAX_TIME_STEP)
            {
                setLastError("Invalid time step: " + std::to_string(dt));
                return false;
            }

            // Store coupling input
            m_totalCouplingInput = coupling_input;

            // Save current state for RK4
            double x = m_state.x;
            double y = m_state.y;

            // RK4 integration
            auto derivatives = [this](double x_val, double y_val) -> std::pair<double, double>
            {
                return computeDerivatives(x_val, y_val);
            };

            // RK4 steps
            auto k1 = derivatives(x, y);
            auto k2 = derivatives(x + 0.5 * dt * k1.first, y + 0.5 * dt * k1.second);
            auto k3 = derivatives(x + 0.5 * dt * k2.first, y + 0.5 * dt * k2.second);
            auto k4 = derivatives(x + dt * k3.first, y + dt * k3.second);

            // Update state
            m_state.x = x + (dt / 6.0) * (k1.first + 2.0 * k2.first + 2.0 * k3.first + k4.first);
            m_state.y = y + (dt / 6.0) * (k1.second + 2.0 * k2.second + 2.0 * k3.second + k4.second);

            // Update phase and amplitude
            updatePhaseAndAmplitude();

            // Validate updated state
            if (!validateState())
            {
                setLastError("State became invalid after update");
                return false;
            }

            m_updateCount++;
            return true;
        }

        /**
         * @brief Compute derivatives for Hopf oscillator
         */
        std::pair<double, double> computeDerivatives(double x, double y) const
        {
            // Current radius (amplitude)
            double r = std::sqrt(x * x + y * y);

            // Hopf oscillator dynamics with coupling
            double omega = TWO_PI * m_params.frequency;
            double mu = m_params.amplitude * m_params.amplitude - r * r;

            // Derivatives including coupling input
            double dx_dt = mu * x - omega * y + m_totalCouplingInput * std::cos(m_state.phase);
            double dy_dt = mu * y + omega * x + m_totalCouplingInput * std::sin(m_state.phase);

            return std::make_pair(dx_dt, dy_dt);
        }

        /**
         * @brief Update phase and amplitude from Cartesian coordinates
         */
        void updatePhaseAndAmplitude()
        {
            m_state.amplitude = std::sqrt(m_state.x * m_state.x + m_state.y * m_state.y);
            m_state.phase = std::atan2(m_state.y, m_state.x);

            // Normalize phase to [0, 2π)
            if (m_state.phase < 0.0)
            {
                m_state.phase += TWO_PI;
            }
        }

        /**
         * @brief Validate current state
         */
        bool validateState() const
        {
            // Check for NaN or infinite values
            if (!std::isfinite(m_state.x) || !std::isfinite(m_state.y) ||
                !std::isfinite(m_state.phase) || !std::isfinite(m_state.amplitude))
            {
                return false;
            }

            // Check amplitude bounds
            if (m_state.amplitude > MAX_AMPLITUDE_THRESHOLD)
            {
                return false;
            }

            return true;
        }

        /**
         * @brief Check if oscillator is in stable limit cycle
         */
        bool isStable() const
        {
            if (m_updateCount < 100)
                return false; // Need time to stabilize

            double target_amplitude = m_params.amplitude;
            double amplitude_error = std::abs(m_state.amplitude - target_amplitude);

            return amplitude_error < CONVERGENCE_THRESHOLD;
        }

        /**
         * @brief Compute output values
         */
        OscillatorOutput computeOutput() const
        {
            OscillatorOutput output;

            // Raw output (sinusoidal)
            output.raw_output = m_state.amplitude * std::cos(m_state.phase);

            // Scaled output with offset
            output.scaled_output = output.raw_output + m_params.offset;

            // Phase-based output [0, 1]
            output.phase_output = m_state.phase / TWO_PI;

            // Determine stance/swing phase
            output.is_stance_phase = m_state.phase < (TWO_PI * m_params.duty_cycle);

            return output;
        }

        /**
         * @brief Get gait progress in current phase
         */
        double getGaitProgress(bool stance_phase) const
        {
            double normalized_phase = m_state.phase / TWO_PI;

            if (stance_phase)
            {
                if (normalized_phase <= m_params.duty_cycle)
                {
                    return normalized_phase / m_params.duty_cycle;
                }
                else
                {
                    return 0.0; // In swing phase
                }
            }
            else // swing phase
            {
                if (normalized_phase > m_params.duty_cycle)
                {
                    return (normalized_phase - m_params.duty_cycle) / (1.0 - m_params.duty_cycle);
                }
                else
                {
                    return 0.0; // In stance phase
                }
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
                std::cerr << "Oscillator " << m_id << " Error: " << message << std::endl;
            }
        }

        /**
         * @brief Clear error message
         */
        void clearLastError()
        {
            m_lastErrorMessage.clear();
        }

        /**
         * @brief Generate statistics string
         */
        std::string generateStatistics() const
        {
            std::ostringstream oss;
            oss << "Oscillator " << m_id << " Statistics:\n";
            oss << "  Phase: " << std::fixed << std::setprecision(3) << m_state.phase << " rad\n";
            oss << "  Amplitude: " << m_state.amplitude << "\n";
            oss << "  Frequency: " << m_state.frequency << " Hz\n";
            oss << "  Position: (" << m_state.x << ", " << m_state.y << ")\n";
            oss << "  Updates: " << m_updateCount << "\n";
            oss << "  Stable: " << (isStable() ? "Yes" : "No") << "\n";
            oss << "  Coupling Input: " << m_totalCouplingInput;
            return oss.str();
        }

        // Member variables
        size_t m_id;
        OscillatorParams m_params;
        OscillatorState m_state;
        double m_totalCouplingInput;
        bool m_debugMode;
        size_t m_updateCount;
        std::string m_lastErrorMessage;
    };

    //==============================================================================
    // Main Oscillator Class Implementation
    //==============================================================================

    Oscillator::Oscillator(size_t id, const OscillatorParams &params)
        : pImpl(std::make_unique<OscillatorImpl>(id, params))
    {
    }

    Oscillator::~Oscillator() = default;

    Oscillator::Oscillator(Oscillator &&other) noexcept
        : pImpl(std::move(other.pImpl))
    {
    }

    Oscillator &Oscillator::operator=(Oscillator &&other) noexcept
    {
        if (this != &other)
        {
            pImpl = std::move(other.pImpl);
        }
        return *this;
    }

    //--------------------------------------------------------------------------
    // Basic Properties
    //--------------------------------------------------------------------------

    size_t Oscillator::getId() const
    {
        return pImpl->m_id;
    }

    bool Oscillator::setParameters(const OscillatorParams &params)
    {
        if (!oscillator_utils::validateParameters(params))
        {
            pImpl->setLastError("Invalid parameters");
            return false;
        }

        pImpl->m_params = params;
        pImpl->reset(false); // Reset with new parameters
        return true;
    }

    OscillatorParams Oscillator::getParameters() const
    {
        return pImpl->m_params;
    }

    //--------------------------------------------------------------------------
    // State Management
    //--------------------------------------------------------------------------

    OscillatorState Oscillator::getState() const
    {
        return pImpl->m_state;
    }

    bool Oscillator::setState(const OscillatorState &state)
    {
        pImpl->m_state = state;
        return pImpl->validateState();
    }

    void Oscillator::reset(bool random_phase)
    {
        pImpl->reset(random_phase);
    }

    //--------------------------------------------------------------------------
    // Integration and Update
    //--------------------------------------------------------------------------

    bool Oscillator::update(double dt, double coupling_input)
    {
        return pImpl->update(dt, coupling_input);
    }

    OscillatorOutput Oscillator::getOutput() const
    {
        return pImpl->computeOutput();
    }

    double Oscillator::getRawOutput() const
    {
        return pImpl->computeOutput().raw_output;
    }

    double Oscillator::getScaledOutput() const
    {
        return pImpl->computeOutput().scaled_output;
    }

    double Oscillator::getPhaseOutput() const
    {
        return pImpl->computeOutput().phase_output;
    }

    //--------------------------------------------------------------------------
    // Phase and Frequency Control
    //--------------------------------------------------------------------------

    double Oscillator::getPhase() const
    {
        return pImpl->m_state.phase;
    }

    bool Oscillator::setPhase(double phase)
    {
        phase = oscillator_utils::normalizePhase(phase);
        pImpl->m_state.phase = phase;

        // Update x, y to match new phase
        pImpl->m_state.x = pImpl->m_state.amplitude * std::cos(phase);
        pImpl->m_state.y = pImpl->m_state.amplitude * std::sin(phase);

        return true;
    }

    double Oscillator::getFrequency() const
    {
        return pImpl->m_params.frequency;
    }

    bool Oscillator::setFrequency(double frequency)
    {
        if (frequency <= 0.0 || frequency > 10.0)
        {
            pImpl->setLastError("Invalid frequency: " + std::to_string(frequency));
            return false;
        }

        pImpl->m_params.frequency = frequency;
        return true;
    }

    double Oscillator::getAmplitude() const
    {
        return pImpl->m_params.amplitude;
    }

    bool Oscillator::setAmplitude(double amplitude)
    {
        if (amplitude < 0.0 || amplitude > 10.0)
        {
            pImpl->setLastError("Invalid amplitude: " + std::to_string(amplitude));
            return false;
        }

        pImpl->m_params.amplitude = amplitude;
        return true;
    }

    //--------------------------------------------------------------------------
    // Gait-specific Methods
    //--------------------------------------------------------------------------

    bool Oscillator::isStancePhase() const
    {
        return pImpl->computeOutput().is_stance_phase;
    }

    double Oscillator::getStanceProgress() const
    {
        return pImpl->getGaitProgress(true);
    }

    double Oscillator::getSwingProgress() const
    {
        return pImpl->getGaitProgress(false);
    }

    //--------------------------------------------------------------------------
    // Coupling Interface
    //--------------------------------------------------------------------------

    void Oscillator::addCouplingInput(double coupling_strength, double phase_difference, double weight)
    {
        double coupling_input = coupling_strength * weight * std::sin(phase_difference);
        pImpl->m_totalCouplingInput += coupling_input;
    }

    void Oscillator::clearCouplingInputs()
    {
        pImpl->m_totalCouplingInput = 0.0;
    }

    double Oscillator::getTotalCouplingInput() const
    {
        return pImpl->m_totalCouplingInput;
    }

    //--------------------------------------------------------------------------
    // Validation and Diagnostics
    //--------------------------------------------------------------------------

    bool Oscillator::validateState() const
    {
        return pImpl->validateState();
    }

    bool Oscillator::isStable() const
    {
        return pImpl->isStable();
    }

    std::string Oscillator::getLastErrorMessage() const
    {
        return pImpl->m_lastErrorMessage;
    }

    void Oscillator::clearLastError()
    {
        pImpl->clearLastError();
    }

    //--------------------------------------------------------------------------
    // Debugging and Analysis
    //--------------------------------------------------------------------------

    std::string Oscillator::getStatistics() const
    {
        return pImpl->generateStatistics();
    }

    void Oscillator::setDebugMode(bool enable)
    {
        pImpl->m_debugMode = enable;
    }

    //==============================================================================
    // Utility Functions Implementation
    //==============================================================================

    namespace oscillator_utils
    {
        double normalizePhase(double phase)
        {
            while (phase < 0.0)
                phase += TWO_PI;
            while (phase >= TWO_PI)
                phase -= TWO_PI;
            return phase;
        }

        double phaseDifference(double phase1, double phase2)
        {
            double diff = phase1 - phase2;
            while (diff > PI)
                diff -= TWO_PI;
            while (diff <= -PI)
                diff += TWO_PI;
            return diff;
        }

        double phaseToGaitProgress(double phase, double duty_cycle)
        {
            double normalized_phase = normalizePhase(phase) / TWO_PI;

            if (normalized_phase <= duty_cycle)
            {
                return normalized_phase / duty_cycle; // Stance progress
            }
            else
            {
                return (normalized_phase - duty_cycle) / (1.0 - duty_cycle); // Swing progress
            }
        }

        bool isStancePhase(double phase, double duty_cycle)
        {
            double normalized_phase = normalizePhase(phase) / TWO_PI;
            return normalized_phase <= duty_cycle;
        }

        bool validateParameters(const OscillatorParams &params)
        {
            if (params.frequency <= 0.0 || params.frequency > 10.0)
                return false;
            if (params.amplitude < 0.0 || params.amplitude > 10.0)
                return false;
            if (params.duty_cycle < 0.1 || params.duty_cycle > 0.9)
                return false;

            return std::isfinite(params.phase) && std::isfinite(params.offset);
        }

        OscillatorParams createGaitParameters(const std::string &gait_type, size_t leg_id)
        {
            OscillatorParams params;

            if (gait_type == "tripod")
            {
                params.frequency = 1.0;
                params.amplitude = 1.0;
                params.duty_cycle = 0.5;

                // Tripod gait: legs 0,2,4 vs 1,3,5
                params.phase = (leg_id % 2 == 0) ? 0.0 : PI;
            }
            else if (gait_type == "wave")
            {
                params.frequency = 0.8;
                params.amplitude = 1.0;
                params.duty_cycle = 0.83;

                // Wave gait: sequential leg lifting
                params.phase = (leg_id * PI) / 3.0;
            }
            else if (gait_type == "ripple")
            {
                params.frequency = 0.6;
                params.amplitude = 1.0;
                params.duty_cycle = 0.67;

                // Ripple gait: two legs at a time
                params.phase = (leg_id * TWO_PI) / 6.0;
            }
            else
            {
                // Default to tripod
                params = createGaitParameters("tripod", leg_id);
            }

            return params;
        }
    }

} // namespace cpg
