#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <stdexcept>
#include "kinematics.hpp"

// Implementation class using PIMPL idiom
class KinematicsImpl
{
public:
    // Any private implementation details
};

// Point3D implementation
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
    if (std::abs(scalar) < 1e-6)
    {
        throw std::runtime_error("Division by near-zero value");
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
    if (std::abs(scalar) < 1e-6)
    {
        throw std::runtime_error("Division by near-zero value");
    }
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
}

double Point3D::length() const
{
    return sqrt(x * x + y * y + z * z);
}

void Point3D::normalize()
{
    double len = length();
    if (len < 1e-6)
    {
        throw std::runtime_error("Cannot normalize vector of near-zero length");
    }
    x /= len;
    y /= len;
    z /= len;
}

double Point3D::distanceTo(const Point3D &other) const
{
    return sqrt(pow(other.x - x, 2) + pow(other.y - y, 2) + pow(other.z - z, 2));
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

// Kinematics singleton implementation
Kinematics &Kinematics::getInstance()
{
    static Kinematics instance; // Created only once
    return instance;
}

Kinematics::Kinematics() : pImpl(new KinematicsImpl()) {}

bool Kinematics::forwardKinematics(const LegPosition &angles, Point3D &position) const
{
    double hip_rad, knee_rad, ankle_rad;
    double s1, c1, s2, c2, s23, c23;

    // Convert angles to radians
    hip_rad = angles.hip * DEG_TO_RAD;
    knee_rad = angles.knee * DEG_TO_RAD;
    ankle_rad = angles.ankle * DEG_TO_RAD;

    // Compute sines and cosines
    s1 = sin(hip_rad);
    c1 = cos(hip_rad);
    s2 = sin(knee_rad);
    c2 = cos(knee_rad);
    s23 = sin(knee_rad + ankle_rad);
    c23 = cos(knee_rad + ankle_rad);

    // Compute foot position
    position.x = c1 * (COXA_LENGTH + FEMUR_LENGTH * c2 + TIBIA_LENGTH * c23);
    position.y = s1 * (COXA_LENGTH + FEMUR_LENGTH * c2 + TIBIA_LENGTH * c23);
    position.z = FEMUR_LENGTH * s2 + TIBIA_LENGTH * s23;

    return true;
}

bool Kinematics::inverseKinematics(const Point3D &position, LegPosition &angles) const
{
    double x = position.x;
    double y = position.y;
    double z = position.z;
    double hip, knee, ankle;
    double L, L1, L2, alpha, beta;

    // Calculate hip angle (yaw)
    hip = atan2(y, x) * RAD_TO_DEG;

    // Calculate distance from hip to foot
    L = sqrt(x * x + y * y) - COXA_LENGTH;
    L1 = sqrt(L * L + z * z);

    // Check if position is reachable
    if (L1 > (FEMUR_LENGTH + TIBIA_LENGTH))
    {
        // Target is too far away - return the closest possible position
        L1 = FEMUR_LENGTH + TIBIA_LENGTH - 1.0;
    }

    // Calculate knee angle
    L2 = (L1 * L1 - FEMUR_LENGTH * FEMUR_LENGTH - TIBIA_LENGTH * TIBIA_LENGTH) / (2.0 * FEMUR_LENGTH * TIBIA_LENGTH);
    if (L2 > 1.0)
        L2 = 1.0;
    if (L2 < -1.0)
        L2 = -1.0;

    knee = acos(L2) * RAD_TO_DEG;
    if (z < 0)
        knee = -knee;

    // Calculate ankle angle
    alpha = atan2(z, L) * RAD_TO_DEG;
    beta = acos((FEMUR_LENGTH * FEMUR_LENGTH + L1 * L1 - TIBIA_LENGTH * TIBIA_LENGTH) / (2.0 * FEMUR_LENGTH * L1)) * RAD_TO_DEG;
    if (knee >= 0)
        ankle = alpha - beta;
    else
        ankle = alpha + beta;

    // Enforce joint limits
    hip = std::max(static_cast<double>(HIP_MIN_ANGLE), std::min(static_cast<double>(HIP_MAX_ANGLE), hip));
    knee = std::max(static_cast<double>(KNEE_MIN_ANGLE), std::min(static_cast<double>(KNEE_MAX_ANGLE), knee));
    ankle = std::max(static_cast<double>(ANKLE_MIN_ANGLE), std::min(static_cast<double>(ANKLE_MAX_ANGLE), ankle));

    // Store results
    angles.hip = static_cast<int16_t>(round(hip));
    angles.knee = static_cast<int16_t>(round(knee));
    angles.ankle = static_cast<int16_t>(round(ankle));

    return true;
}

bool Kinematics::isPositionReachable(const Point3D &position) const
{
    // Calculate distance from origin in XY plane
    double distance = sqrt(position.x * position.x + position.y * position.y);

    // Check min/max reach
    if (distance < (COXA_LENGTH - FEMUR_LENGTH))
        return false; // Too close to body

    if (distance > (COXA_LENGTH + FEMUR_LENGTH + TIBIA_LENGTH))
        return false; // Too far from body

    return true; // Position is potentially reachable
}
