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

#ifndef GAIT_HPP
#define GAIT_HPP

#include <memory>
#include <vector>
#include "kinematics.hpp"

/**
 * @brief Gait generation and control system
 *
 * This namespace contains classes and functions for generating and controlling
 * robot walking gaits and movement patterns.
 */
namespace gait
{

    //==============================================================================
    // Gait Types
    //==============================================================================

    /**
     * @brief Available gait pattern types
     */
    enum class GaitType
    {
        TRIPOD, ///< Alternating tripod (3 legs at a time)
        WAVE,   ///< Wave gait (one leg at a time)
        RIPPLE  ///< Ripple gait (overlapping movement)
    };

    //==============================================================================
    // Configuration Parameters
    //==============================================================================

    /**
     * @brief Gait configuration parameters
     *
     * Defines the parameters that control how the gait is generated and executed
     */
    class GaitParameters
    {
    public:
        /**
         * @brief Construct a new Gait Parameters object with defaults
         */
        GaitParameters() : type(GaitType::TRIPOD),
                           stepHeight(30.0),
                           stepLength(60.0),
                           cycleTime(2.0),
                           dutyFactor(0.5) {}

        /**
         * @brief Construct a new Gait Parameters object with specific settings
         *
         * @param type Type of gait pattern
         * @param stepHeight Maximum step height in mm
         * @param stepLength Maximum stride length in mm
         * @param cycleTime Time for complete gait cycle in seconds
         * @param dutyFactor Portion of cycle in stance phase (0.0-1.0)
         */
        GaitParameters(
            GaitType type,
            double stepHeight,
            double stepLength,
            double cycleTime,
            double dutyFactor) : type(type),
                                 stepHeight(stepHeight),
                                 stepLength(stepLength),
                                 cycleTime(cycleTime),
                                 dutyFactor(dutyFactor) {}

        // Configuration parameters
        GaitType type;     ///< Type of gait pattern
        double stepHeight; ///< Maximum step height (mm)
        double stepLength; ///< Maximum stride length (mm)
        double cycleTime;  ///< Time for complete gait cycle (seconds)
        double dutyFactor; ///< Portion of cycle in stance phase (0.0-1.0)

        /**
         * @brief Validate parameters are within acceptable ranges
         *
         * @return bool True if parameters are valid
         */
        bool validate() const
        {
            return (stepHeight > 0.0 &&
                    stepLength > 0.0 &&
                    cycleTime > 0.0 &&
                    dutyFactor > 0.0 &&
                    dutyFactor < 1.0);
        }
    };

    //==============================================================================
    // Forward declarations
    //==============================================================================

    // Implementation class (PIMPL idiom)
    class GaitImpl;

    //==============================================================================
    // Main Gait Class
    //==============================================================================

    /**
     * @brief Gait control class
     *
     * Handles the generation and control of hexapod walking gaits
     */
    class Gait
    {
    public:
        /**
         * @brief Construct a new Gait object
         */
        Gait();

        /**
         * @brief Destroy the Gait object
         */
        ~Gait();

        // Non-copyable
        Gait(const Gait &) = delete;
        Gait &operator=(const Gait &) = delete;

        // Move constructor and assignment
        Gait(Gait &&) noexcept;
        Gait &operator=(Gait &&) noexcept;

        //--------------------------------------------------------------------------
        // Initialization and Configuration
        //--------------------------------------------------------------------------

        /**
         * @brief Initialize gait controller
         *
         * @param hexapod Reference to the hexapod controller
         * @param params Initial gait parameters
         * @return bool True if initialization successful
         */
        bool init(hexapod::Hexapod &hexapod, const GaitParameters &params);

        /**
         * @brief Get current parameters
         *
         * @return GaitParameters Current parameters
         */
        GaitParameters getParameters() const;

        /**
         * @brief Update parameters
         *
         * @param params New parameters to set
         * @return bool True if parameters were successfully applied
         */
        bool setParameters(const GaitParameters &params);

        //--------------------------------------------------------------------------
        // Gait Control
        //--------------------------------------------------------------------------

        /**
         * @brief Update gait for all legs
         *
         * @param time Current time in seconds
         * @param direction Direction of travel in degrees (0 = forward)
         * @param speed Movement speed factor (0.0 to 1.0)
         * @return bool True if update was successful
         */
        bool update(double time, double direction, double speed);

        /**
         * @brief Center all legs to default position
         *
         * @return bool True if centering successful
         */
        bool centerLegs();

        /**
         * @brief Calculate the phase for a specific leg at the given time
         *
         * @param legIndex The leg index (0-5)
         * @param time Current time in seconds
         * @return double Phase value (0.0 to 1.0)
         */
        double calculateLegPhase(int legIndex, double time) const;

        /**
         * @brief Check if gait controller is properly initialized
         *
         * @return bool True if initialized
         */
        bool isInitialized() const;

    private:
        /**
         * @brief Implementation pointer (PIMPL idiom)
         */
        std::unique_ptr<GaitImpl> pImpl;
    };

} // namespace gait

// For backward compatibility, import types into global namespace
using gait::Gait;
using gait::GaitParameters;
using gait::GaitType;

#endif /* GAIT_HPP */
