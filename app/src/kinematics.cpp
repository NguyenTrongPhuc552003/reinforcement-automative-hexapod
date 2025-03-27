#include <cstdio>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <array>
#include "kinematics.hpp"

namespace kinematics
{

    //==============================================================================
    // Implementation Class (PIMPL idiom)
    //==============================================================================

    class KinematicsImpl
    {
    public:
        KinematicsImpl() : initialized(true)
        {
            // Initialize with default leg parameters
            for (int i = 0; i < hexapod::Config::NUM_LEGS; ++i)
            {
                legParams[i] = {
                    hexapod::Dimensions::COXA_LENGTH,
                    hexapod::Dimensions::FEMUR_LENGTH,
                    hexapod::Dimensions::TIBIA_LENGTH};
            }
        }

        /**
         * @brief Custom leg parameters structure
         */
        struct LegParameters
        {
            double coxaLength;
            double femurLength;
            double tibiaLength;
        };

        /**
         * @brief Set custom parameters for a specific leg
         */
        bool setLegParameters(int legIndex, double coxaLength, double femurLength, double tibiaLength)
        {
            // Validate parameters
            if (legIndex < 0 || legIndex >= hexapod::Config::NUM_LEGS)
            {
                return false;
            }

            if (coxaLength <= 0 || femurLength <= 0 || tibiaLength <= 0)
            {
                return false;
            }

            // Set parameters for the specified leg
            legParams[legIndex] = {coxaLength, femurLength, tibiaLength};
            return true;
        }

        // Member variables
        bool initialized;
        std::array<LegParameters, hexapod::Config::NUM_LEGS> legParams;
    };

    //==============================================================================
    // Point3D Implementation
    //==============================================================================

    Point3D::Point3D(double x, double y, double z) : x(x), y(y), z(z) {}

    Point3D Point3D::operator+(const Point3D &other) const
    {
        return Point3D(x + other.x, y + other.y, z + other.z);
    }

    Point3D Point3D::operator-(const Point3D &other) const
    {
        return Point3D(x - other.x, y - other.y, z - other.z);
    }

    Point3D Point3D::operator*(double scalar) const
    {
        return Point3D(x * scalar, y * scalar, z * scalar);
    }

    Point3D Point3D::operator/(double scalar) const
    {
        // Check for division by near-zero (with some epsilon)
        if (std::abs(scalar) < 1e-6)
        {
            throw std::runtime_error("Point3D: Division by near-zero value");
        }
        return Point3D(x / scalar, y / scalar, z / scalar);
    }

    Point3D &Point3D::operator+=(const Point3D &other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Point3D &Point3D::operator-=(const Point3D &other)
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Point3D &Point3D::operator*=(double scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Point3D &Point3D::operator/=(double scalar)
    {
        // Check for division by near-zero (with some epsilon)
        if (std::abs(scalar) < 1e-6)
        {
            throw std::runtime_error("Point3D: Division by near-zero value");
        }
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    double Point3D::length() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    void Point3D::normalize()
    {
        double len = length();
        if (len < 1e-6)
        {
            throw std::runtime_error("Point3D: Cannot normalize vector of near-zero length");
        }
        x /= len;
        y /= len;
        z /= len;
    }

    double Point3D::distanceTo(const Point3D &other) const
    {
        const double dx = other.x - x;
        const double dy = other.y - y;
        const double dz = other.z - z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    void Point3D::print() const
    {
        printf("Point: (%.2f, %.2f, %.2f)\n", x, y, z);
    }

    Point3D Point3D::lerp(const Point3D &a, const Point3D &b, double t)
    {
        // Clamp t to [0,1]
        t = std::max(0.0, std::min(1.0, t));
        return Point3D(
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t,
            a.z + (b.z - a.z) * t);
    }

    double Point3D::dot(const Point3D &a, const Point3D &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    Point3D Point3D::cross(const Point3D &a, const Point3D &b)
    {
        return Point3D(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
    }

    //==============================================================================
    // Kinematics Class Implementation
    //==============================================================================

    // Singleton implementation
    Kinematics &Kinematics::getInstance()
    {
        static Kinematics instance; // Created only once
        return instance;
    }

    Kinematics::Kinematics() : pImpl(std::make_unique<KinematicsImpl>()) {}

    bool Kinematics::forwardKinematics(const hexapod::LegPosition &angles, Point3D &position) const
    {
        if (!pImpl->initialized)
        {
            return false;
        }

        // Get leg index from the position object or use default (leg 0) parameters
        const uint8_t legIndex = std::min(static_cast<uint8_t>(hexapod::Config::NUM_LEGS - 1), angles.leg_num);
        const auto &params = pImpl->legParams[legIndex];

        // Get direct references to joint values
        const auto hip = angles.getHip();
        const auto knee = angles.getKnee();
        const auto ankle = angles.getAnkle();

        // Precompute radians conversion to avoid repeated multiplication
        const double hipRad = hip * AngleConversion::DEG_TO_RAD;
        const double kneeRad = knee * AngleConversion::DEG_TO_RAD;
        const double ankleRad = ankle * AngleConversion::DEG_TO_RAD;

        // Compute sines and cosines once
        const double sinHip = std::sin(hipRad);
        const double cosHip = std::cos(hipRad);
        const double sinKnee = std::sin(kneeRad);
        const double cosKnee = std::cos(kneeRad);
        const double sinAnkleKnee = std::sin(kneeRad + ankleRad);
        const double cosAnkleKnee = std::cos(kneeRad + ankleRad);

        // Compute intermediate values for more efficient calculation
        const double term = params.coxaLength +
                            params.femurLength * cosKnee +
                            params.tibiaLength * cosAnkleKnee;

        // Calculate 3D position from joint angles
        position.x = cosHip * term;
        position.y = sinHip * term;
        position.z = params.femurLength * sinKnee + params.tibiaLength * sinAnkleKnee;

        return true;
    }

    bool Kinematics::inverseKinematics(const Point3D &position, hexapod::LegPosition &angles) const
    {
        if (!pImpl->initialized)
        {
            return false;
        }

        // Get leg index from the angles object or use default (leg 0) parameters
        const uint8_t legIndex = std::min(static_cast<uint8_t>(hexapod::Config::NUM_LEGS - 1), angles.leg_num);
        const auto &params = pImpl->legParams[legIndex];

        // Extract position coordinates
        const double x = position.x;
        const double y = position.y;
        const double z = position.z;

        // Calculate hip angle using atan2 (handles quadrant correctly)
        double hip = std::atan2(y, x) * AngleConversion::RAD_TO_DEG;

        // Calculate distance from hip to foot in the horizontal plane
        double L = std::sqrt(x * x + y * y) - params.coxaLength;

        // Calculate straight-line distance from knee to foot
        double L1 = std::sqrt(L * L + z * z);

        // Check if position is reachable and adjust if necessary
        if (L1 > (params.femurLength + params.tibiaLength))
        {
            // Target is too far away - adjust to maximum reachable distance
            L1 = params.femurLength + params.tibiaLength - 1.0;
        }

        // Calculate knee angle using law of cosines
        double cosKneeAngle = (L1 * L1 - params.femurLength * params.femurLength -
                               params.tibiaLength * params.tibiaLength) /
                              (-2.0 * params.femurLength * params.tibiaLength);

        // Clamp value to valid cosine range [-1, 1]
        cosKneeAngle = std::max(-1.0, std::min(1.0, cosKneeAngle));

        // Calculate knee angle in degrees (fix direction based on z)
        double knee = std::acos(cosKneeAngle) * AngleConversion::RAD_TO_DEG;
        if (z < 0)
        {
            knee = -knee; // Reflect angle for negative z
        }

        // Calculate ankle angle
        double alpha = std::atan2(z, L) * AngleConversion::RAD_TO_DEG;
        double beta = std::acos((params.femurLength * params.femurLength + L1 * L1 -
                                 params.tibiaLength * params.tibiaLength) /
                                (2.0 * params.femurLength * L1)) *
                      AngleConversion::RAD_TO_DEG;

        // Determine ankle angle based on knee direction
        double ankle = (knee >= 0) ? alpha - beta : alpha + beta;

        // Enforce joint limits
        hip = std::max(static_cast<double>(hexapod::AngleLimits::HIP_MIN),
                       std::min(static_cast<double>(hexapod::AngleLimits::HIP_MAX), hip));
        knee = std::max(static_cast<double>(hexapod::AngleLimits::KNEE_MIN),
                        std::min(static_cast<double>(hexapod::AngleLimits::KNEE_MAX), knee));
        ankle = std::max(static_cast<double>(hexapod::AngleLimits::ANKLE_MIN),
                         std::min(static_cast<double>(hexapod::AngleLimits::ANKLE_MAX), ankle));

        // Store results with proper rounding
        angles.setHip(static_cast<int16_t>(std::round(hip)));
        angles.setKnee(static_cast<int16_t>(std::round(knee)));
        angles.setAnkle(static_cast<int16_t>(std::round(ankle)));

        return true;
    }

    bool Kinematics::isPositionReachable(const Point3D &position) const
    {
        // For now, we're using leg 0 parameters by default
        const auto &params = pImpl->legParams[0];

        // Calculate distance from origin in XY plane (horizontal reach)
        double horizontalDistance = std::sqrt(position.x * position.x + position.y * position.y);

        // Check minimum reach (too close to body)
        if (horizontalDistance < (params.coxaLength - params.femurLength))
        {
            return false;
        }

        // Check maximum reach (too far from body)
        // Calculate total straight-line distance from origin
        double totalDistance = std::sqrt(
            std::pow(horizontalDistance - params.coxaLength, 2) +
            std::pow(position.z, 2));

        if (totalDistance > (params.femurLength + params.tibiaLength))
        {
            return false;
        }

        return true;
    }

    bool Kinematics::setLegParameters(int legIndex, double coxaLength, double femurLength, double tibiaLength)
    {
        if (!pImpl)
        {
            return false;
        }

        return pImpl->setLegParameters(legIndex, coxaLength, femurLength, tibiaLength);
    }

} // namespace kinematics
