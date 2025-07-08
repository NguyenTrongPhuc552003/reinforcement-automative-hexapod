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

#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <cmath>
#include <memory>
#include "hexapod.hpp"

/**
 * @brief Kinematics and 3D geometry calculation system
 *
 * This namespace contains classes and functions for 3D geometry
 * and robot kinematics calculations.
 */
namespace kinematics
{

    //==============================================================================
    // Constants
    //==============================================================================

    /**
     * @brief Conversion constants for angle calculations
     */
    struct AngleConversion
    {
        static constexpr double DEG_TO_RAD = 0.0174532925;  ///< Convert degrees to radians
        static constexpr double RAD_TO_DEG = 57.2957795131; ///< Convert radians to degrees
    };

    //==============================================================================
    // 3D Point Class
    //==============================================================================

    /**
     * @brief 3D point class with vector operations
     *
     * Represents a point or vector in 3D space and provides mathematical operations.
     */
    class Point3D
    {
    public:
        /**
         * @brief Construct a new Point3D object
         *
         * @param x X-coordinate (default 0.0)
         * @param y Y-coordinate (default 0.0)
         * @param z Z-coordinate (default 0.0)
         */
        explicit Point3D(double x = 0.0, double y = 0.0, double z = 0.0);

        //--------------------------------------------------------------------------
        // Public data members (directly accessible for efficiency)
        //--------------------------------------------------------------------------

        double x; ///< X-coordinate
        double y; ///< Y-coordinate
        double z; ///< Z-coordinate

        //--------------------------------------------------------------------------
        // Vector operations
        //--------------------------------------------------------------------------

        /**
         * @brief Add two points/vectors
         *
         * @param other Point to add
         * @return Point3D Result of addition
         */
        Point3D operator+(const Point3D &other) const;

        /**
         * @brief Subtract a point/vector
         *
         * @param other Point to subtract
         * @return Point3D Result of subtraction
         */
        Point3D operator-(const Point3D &other) const;

        /**
         * @brief Multiply by a scalar
         *
         * @param scalar Scalar value
         * @return Point3D Result of multiplication
         */
        Point3D operator*(double scalar) const;

        /**
         * @brief Divide by a scalar
         *
         * @param scalar Scalar value (must not be zero)
         * @return Point3D Result of division
         * @throws std::runtime_error If scalar is near zero
         */
        Point3D operator/(double scalar) const;

        //--------------------------------------------------------------------------
        // Compound assignment operators
        //--------------------------------------------------------------------------

        /**
         * @brief Add and assign
         *
         * @param other Point to add
         * @return Point3D& Reference to this object
         */
        Point3D &operator+=(const Point3D &other);

        /**
         * @brief Subtract and assign
         *
         * @param other Point to subtract
         * @return Point3D& Reference to this object
         */
        Point3D &operator-=(const Point3D &other);

        /**
         * @brief Multiply by scalar and assign
         *
         * @param scalar Scalar value
         * @return Point3D& Reference to this object
         */
        Point3D &operator*=(double scalar);

        /**
         * @brief Divide by scalar and assign
         *
         * @param scalar Scalar value (must not be zero)
         * @return Point3D& Reference to this object
         * @throws std::runtime_error If scalar is near zero
         */
        Point3D &operator/=(double scalar);

        //--------------------------------------------------------------------------
        // Utility methods
        //--------------------------------------------------------------------------

        /**
         * @brief Calculate vector length
         *
         * @return double Vector length (magnitude)
         */
        double length() const;

        /**
         * @brief Normalize vector to unit length
         *
         * @throws std::runtime_error If vector length is near zero
         */
        void normalize();

        /**
         * @brief Calculate Euclidean distance to another point
         *
         * @param other Target point
         * @return double Distance
         */
        double distanceTo(const Point3D &other) const;

        /**
         * @brief Print point coordinates to stdout
         */
        void print() const;

        //--------------------------------------------------------------------------
        // Static helpers
        //--------------------------------------------------------------------------

        /**
         * @brief Linear interpolation between two points
         *
         * @param a Start point
         * @param b End point
         * @param t Interpolation factor (0.0 to 1.0)
         * @return Point3D Interpolated point
         */
        static Point3D lerp(const Point3D &a, const Point3D &b, double t);

        /**
         * @brief Calculate dot product of two vectors
         *
         * @param a First vector
         * @param b Second vector
         * @return double Dot product result
         */
        static double dot(const Point3D &a, const Point3D &b);

        /**
         * @brief Calculate cross product of two vectors
         *
         * @param a First vector
         * @param b Second vector
         * @return Point3D Cross product result
         */
        static Point3D cross(const Point3D &a, const Point3D &b);
    };

    //==============================================================================
    // Forward declarations
    //==============================================================================

    // Implementation class (PIMPL idiom)
    class KinematicsImpl;

    //==============================================================================
    // Kinematics Class
    //==============================================================================

    /**
     * @brief Kinematics calculation engine for the hexapod robot
     *
     * Provides forward and inverse kinematics calculations to translate between
     * joint angles and 3D coordinates. Implemented as a singleton for global access.
     */
    class Kinematics
    {
    public:
        /**
         * @brief Get the singleton instance
         *
         * @return Kinematics& Reference to the singleton instance
         */
        static Kinematics &getInstance();

        // Delete copy and move operations for singleton
        Kinematics(const Kinematics &) = delete;
        Kinematics &operator=(const Kinematics &) = delete;
        Kinematics(Kinematics &&) = delete;
        Kinematics &operator=(Kinematics &&) = delete;

        //--------------------------------------------------------------------------
        // Kinematics calculations
        //--------------------------------------------------------------------------

        /**
         * @brief Calculate forward kinematics (joint angles to position)
         *
         * Converts joint angles to the corresponding foot position in 3D space
         *
         * @param angles Input joint angles
         * @param[out] position Output position
         * @return bool True if calculation successful
         */
        bool forwardKinematics(const hexapod::LegPosition &angles, Point3D &position) const;

        /**
         * @brief Calculate inverse kinematics (position to joint angles)
         *
         * Converts a 3D position to the corresponding joint angles
         *
         * @param position Input position
         * @param[out] angles Output joint angles
         * @return bool True if calculation successful
         */
        bool inverseKinematics(const Point3D &position, hexapod::LegPosition &angles) const;

        //--------------------------------------------------------------------------
        // Utility functions
        //--------------------------------------------------------------------------

        /**
         * @brief Check if a position is within the robot's reach
         *
         * @param position Position to check
         * @return bool True if position is reachable
         */
        bool isPositionReachable(const Point3D &position) const;

        /**
         * @brief Set custom leg parameters for a specific leg
         *
         * Allows customizing the leg parameters for special cases
         *
         * @param legIndex Leg index (0-5)
         * @param coxaLength Coxa segment length
         * @param femurLength Femur segment length
         * @param tibiaLength Tibia segment length
         * @return bool True if successful
         */
        bool setLegParameters(int legIndex, double coxaLength, double femurLength, double tibiaLength);

    private:
        /**
         * @brief Private constructor for singleton
         */
        Kinematics();

        /**
         * @brief Implementation pointer (PIMPL idiom)
         */
        std::unique_ptr<KinematicsImpl> pImpl;
    };

} // namespace kinematics

// For backward compatibility, import types into global namespace
using kinematics::Kinematics;
using kinematics::Point3D;

#endif /* KINEMATICS_HPP */
