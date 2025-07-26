#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

struct Point3D
{
    float x, y, z;
    Point3D(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
};

struct Angles
{
    float coxa, femur, tibia;
    Angles(float c = 0, float f = 0, float t = 0) : coxa(c), femur(f), tibia(t) {}
};

class Kinematics
{
public:
    static const float COXA_LENGTH;
    static const float FEMUR_LENGTH;
    static const float TIBIA_LENGTH;

    static Angles inverseKinematics(const Point3D &target);
    static Point3D forwardKinematics(const Angles &angles);

private:
    static float radToDeg(float rad);
    static float degToRad(float deg);
};

#endif
