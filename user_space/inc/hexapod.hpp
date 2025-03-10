#ifndef HEXAPOD_H
#define HEXAPOD_H

#include <stdint.h>
#include <string>
#include <vector>
#include <memory>

// Robot dimensions in mm
constexpr int COXA_LENGTH = 30;
constexpr int FEMUR_LENGTH = 85;
constexpr int TIBIA_LENGTH = 130;

// Number of legs and servos
constexpr int NUM_LEGS = 6;
constexpr int SERVOS_PER_LEG = 3;
constexpr int TOTAL_SERVOS = NUM_LEGS * SERVOS_PER_LEG;

// Angle limits
constexpr int HIP_MIN_ANGLE = -90;
constexpr int HIP_MAX_ANGLE = 90;
constexpr int KNEE_MIN_ANGLE = -90;
constexpr int KNEE_MAX_ANGLE = 90;
constexpr int ANKLE_MIN_ANGLE = -90;
constexpr int ANKLE_MAX_ANGLE = 90;

// Forward declarations
class HexapodImpl;

// Structures for communication
class LegPosition
{
public:
    LegPosition(int16_t hip = 0, int16_t knee = 0, int16_t ankle = 0);

    int16_t hip;
    int16_t knee;
    int16_t ankle;
};

class ImuData
{
public:
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    // Helper methods to convert raw data to physical units
    float getAccelX() const { return accel_x / 16384.0f; } // Convert to g's
    float getAccelY() const { return accel_y / 16384.0f; }
    float getAccelZ() const { return accel_z / 16384.0f; }
    float getGyroX() const { return gyro_x / 65.5f; } // Convert to degrees/second
    float getGyroY() const { return gyro_y / 65.5f; }
    float getGyroZ() const { return gyro_z / 65.5f; }
};

class Calibration
{
public:
    uint8_t leg_num;
    int16_t hip_offset;
    int16_t knee_offset;
    int16_t ankle_offset;
};

// Main hexapod class with PIMPL idiom
class Hexapod
{
public:
    Hexapod();
    ~Hexapod();

    // Non-copyable
    Hexapod(const Hexapod &) = delete;
    Hexapod &operator=(const Hexapod &) = delete;

    // Move constructor and assignment
    Hexapod(Hexapod &&) noexcept;
    Hexapod &operator=(Hexapod &&) noexcept;

    // Initialize and clean up
    bool init();
    void cleanup();

    // Leg position control
    bool setLegPosition(uint8_t leg_num, const LegPosition &position);
    bool getLegPosition(uint8_t leg_num, LegPosition &position) const;

    // IMU data access
    bool getImuData(ImuData &data) const;

    // Calibration
    bool setCalibration(uint8_t leg_num, int16_t hip_offset, int16_t knee_offset, int16_t ankle_offset);

    // Center all legs to neutral position
    bool centerAll();

    // Error handling and utilities
    std::string getLastErrorMessage() const;
    int getLastErrorCode() const;

    // Debug utilities
    static void printLegPosition(const LegPosition &position);
    static void printImuData(const ImuData &data);

private:
    std::unique_ptr<HexapodImpl> pImpl; // PIMPL idiom for implementation hiding
};

#endif /* HEXAPOD_H */
