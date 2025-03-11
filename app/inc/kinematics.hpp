#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <cmath>
#include <memory>
#include "hexapod.hpp"

// Forward declarations
class KinematicsImpl;

// 3D point class with vector operations
class Point3D
{
public:
    Point3D(double x = 0.0, double y = 0.0, double z = 0.0);

    double x;
    double y;
    double z;

    // Vector operations
    Point3D operator+(const Point3D &other) const;
    Point3D operator-(const Point3D &other) const;
    Point3D operator*(double scalar) const;
    Point3D operator/(double scalar) const;

    // Compound assignment
    Point3D &operator+=(const Point3D &other);
    Point3D &operator-=(const Point3D &other);
    Point3D &operator*=(double scalar);
    Point3D &operator/=(double scalar);

    // Utility methods
    double length() const;
    void normalize();
    double distanceTo(const Point3D &other) const;
    void print() const;

    // Static helpers
    static Point3D lerp(const Point3D &a, const Point3D &b, double t);
};

// Kinematics utility class (Singleton pattern)
class Kinematics
{
public:
    // Get singleton instance
    static Kinematics &getInstance();

    // Constructor and assignment operators for singleton
    Kinematics(const Kinematics &) = delete;
    Kinematics &operator=(const Kinematics &) = delete;

    // Move constructor and assignment operator for singleton
    Kinematics(Kinematics &&) = delete;
    Kinematics &operator=(Kinematics &&) = delete;

    // Forward and inverse kinematics
    bool forwardKinematics(const LegPosition &angles, Point3D &position) const;
    bool inverseKinematics(const Point3D &position, LegPosition &angles) const;

    // Utility functions
    bool isPositionReachable(const Point3D &position) const;

    // Constants for conversions
    static constexpr double DEG_TO_RAD = 0.0174532925;
    static constexpr double RAD_TO_DEG = 57.2957795131;

private:
    Kinematics(); // Private constructor for singleton
    std::unique_ptr<KinematicsImpl> pImpl;
};

#endif /* KINEMATICS_H */
