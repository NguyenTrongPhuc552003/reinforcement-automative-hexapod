#ifndef HEXAPOD_HPP
#define HEXAPOD_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief Hexapod Robot Control System
 *
 * This namespace contains all constants, types and classes related to the
 * hexapod robot control system.
 */
namespace hexapod
{

    //==============================================================================
    // Constants
    //==============================================================================

    /**
     * @brief Robot dimensions in mm (must match kernel driver definitions)
     */
    struct Dimensions
    {
        static constexpr int COXA_LENGTH = 30;
        static constexpr int FEMUR_LENGTH = 85;
        static constexpr int TIBIA_LENGTH = 130;
    };

    /**
     * @brief Robot configuration constants
     */
    struct Config
    {
        static constexpr int NUM_LEGS = 6;
        static constexpr int SERVOS_PER_LEG = 3;
        static constexpr int TOTAL_SERVOS = NUM_LEGS * SERVOS_PER_LEG;
    };

    /**
     * @brief Joint angle limits
     */
    struct AngleLimits
    {
        static constexpr int HIP_MIN = -90;
        static constexpr int HIP_MAX = 90;
        static constexpr int KNEE_MIN = -90;
        static constexpr int KNEE_MAX = 90;
        static constexpr int ANKLE_MIN = -90;
        static constexpr int ANKLE_MAX = 90;
    };

    //==============================================================================
    // Types and Data Classes
    //==============================================================================

    /**
     * @brief Error categories for better error classification
     */
    enum class ErrorCategory
    {
        NONE,          ///< No error
        DEVICE,        ///< Device access errors
        COMMUNICATION, ///< Communication errors
        PARAMETER,     ///< Invalid parameter errors
        SYSTEM,        ///< System resource errors
        HARDWARE       ///< Hardware-related errors
    };

    /**
     * @brief Joint angles structure - compatible with kernel's hexapod_leg_joint
     */
    struct JointAngles
    {
        int16_t hip;   ///< Hip joint angle in degrees
        int16_t knee;  ///< Knee joint angle in degrees
        int16_t ankle; ///< Ankle joint angle in degrees
    };

    /**
     * @brief Leg position class - compatible with kernel's hexapod_leg_cmd
     */
    class LegPosition
    {
    public:
        /**
         * @brief Construct a new Leg Position object
         *
         * @param hip Hip joint angle in degrees
         * @param knee Knee joint angle in degrees
         * @param ankle Ankle joint angle in degrees
         */
        explicit LegPosition(int16_t hip = 0, int16_t knee = 0, int16_t ankle = 0);

        /**
         * @brief Copy constructor
         */
        LegPosition(const LegPosition &other);

        /**
         * @brief Assignment operator
         */
        LegPosition &operator=(const LegPosition &other);

        // Leg identifier
        uint8_t leg_num;

        // Joint angles grouped together
        JointAngles joints;

        // Accessors with semantic naming
        int16_t getHip() const { return joints.hip; }
        int16_t getKnee() const { return joints.knee; }
        int16_t getAnkle() const { return joints.ankle; }

        // Setters with clear intent
        void setHip(int16_t value) { joints.hip = value; }
        void setKnee(int16_t value) { joints.knee = value; }
        void setAnkle(int16_t value) { joints.ankle = value; }
    };

    /**
     * @brief Sensor types supported by the system
     */
    enum class SensorType : uint8_t
    {
        MPU6050 = 0, ///< MPU6050 6-axis IMU
        ADXL345 = 1, ///< ADXL345 3-axis accelerometer
        AUTO = 2     ///< Auto-detect available sensor
    };

    /**
     * @brief IMU data class - compatible with kernel's hexapod_imu_data
     */
    class ImuData
    {
    public:
        int16_t accel_x;     ///< Raw X-axis acceleration
        int16_t accel_y;     ///< Raw Y-axis acceleration
        int16_t accel_z;     ///< Raw Z-axis acceleration
        int16_t gyro_x;      ///< Raw X-axis angular velocity (0 for ADXL345)
        int16_t gyro_y;      ///< Raw Y-axis angular velocity (0 for ADXL345)
        int16_t gyro_z;      ///< Raw Z-axis angular velocity (0 for ADXL345)
        uint8_t sensor_type; ///< Which sensor provided the data

        // Helper methods to convert raw data to physical units
        float getAccelX() const
        {
            return accel_x / (getSensorType() == SensorType::ADXL345 ? 256.0f : 16384.0f);
        } ///< Convert to g's

        float getAccelY() const
        {
            return accel_y / (getSensorType() == SensorType::ADXL345 ? 256.0f : 16384.0f);
        } ///< Convert to g's

        float getAccelZ() const
        {
            return accel_z / (getSensorType() == SensorType::ADXL345 ? 256.0f : 16384.0f);
        } ///< Convert to g's

        float getGyroX() const { return gyro_x / 65.5f; } ///< Convert to degrees/second
        float getGyroY() const { return gyro_y / 65.5f; } ///< Convert to degrees/second
        float getGyroZ() const { return gyro_z / 65.5f; } ///< Convert to degrees/second

        /**
         * @brief Check if gyroscope data is available
         * @return true if sensor provides gyroscope data
         */
        bool hasGyro() const { return getSensorType() == SensorType::MPU6050; }

        /**
         * @brief Get the sensor type that provided this data
         * @return SensorType enum value
         */
        SensorType getSensorType() const { return static_cast<SensorType>(sensor_type); }
    };

    /**
     * @brief Comprehensive error information class
     */
    class ErrorInfo
    {
    public:
        // Error codes
        struct ErrorCode
        {
            static constexpr int SUCCESS = 0;
            static constexpr int INVALID_PARAM = 1;
            static constexpr int DEVICE_NOT_FOUND = 2;
            static constexpr int PERMISSION_DENIED = 3;
            static constexpr int DEVICE_BUSY = 4;
            static constexpr int IO_ERROR = 5;
            static constexpr int BAD_FILE = 6;
            static constexpr int COMM_ERROR = 7;
            static constexpr int HARDWARE_ERROR = 8;
            static constexpr int NOT_INITIALIZED = 9;
            static constexpr int TIMEOUT = 10;
        };

        /**
         * @brief Construct a default success error info
         */
        ErrorInfo() : code(ErrorCode::SUCCESS), category(ErrorCategory::NONE), message("") {}

        /**
         * @brief Construct an error with details
         */
        ErrorInfo(int code, ErrorCategory category, const std::string &message)
            : code(code), category(category), message(message) {}

        // Accessors
        int getCode() const { return code; }
        ErrorCategory getCategory() const { return category; }
        std::string getMessage() const { return message; }
        bool hasError() const { return code != ErrorCode::SUCCESS; }

        // Factory methods for common errors
        static ErrorInfo invalidParameter(const std::string &details)
        {
            return ErrorInfo(ErrorCode::INVALID_PARAM, ErrorCategory::PARAMETER, "Invalid parameter: " + details);
        }

        static ErrorInfo deviceNotFound(const std::string &details)
        {
            return ErrorInfo(ErrorCode::DEVICE_NOT_FOUND, ErrorCategory::DEVICE, "Device not found: " + details);
        }

        static ErrorInfo communicationError(const std::string &details)
        {
            return ErrorInfo(ErrorCode::COMM_ERROR, ErrorCategory::COMMUNICATION, "Communication error: " + details);
        }

        static ErrorInfo notInitialized()
        {
            return ErrorInfo(ErrorCode::NOT_INITIALIZED, ErrorCategory::PARAMETER, "Hexapod not initialized");
        }

    private:
        int code;
        ErrorCategory category;
        std::string message;
    };

    // Forward declaration for the implementation class
    class HexapodImpl;

    //==============================================================================
    // Main Hexapod Class
    //==============================================================================

    /**
     * @brief Main hexapod robot control class
     *
     * This class provides the interface to control a hexapod robot through
     * the kernel driver. It uses PIMPL idiom for implementation hiding.
     */
    class Hexapod
    {
    public:
        /**
         * @brief Construct a new Hexapod object
         */
        Hexapod();

        /**
         * @brief Destroy the Hexapod object
         *
         * Ensures all resources are properly cleaned up
         */
        ~Hexapod();

        // Non-copyable
        Hexapod(const Hexapod &) = delete;
        Hexapod &operator=(const Hexapod &) = delete;

        // Move semantics
        Hexapod(Hexapod &&other) noexcept;
        Hexapod &operator=(Hexapod &&other) noexcept;

        //--------------------------------------------------------------------------
        // Initialization and Cleanup
        //--------------------------------------------------------------------------

        /**
         * @brief Initialize the hexapod hardware interface
         *
         * @return true if initialization successful
         * @return false if initialization failed (check getLastError())
         */
        bool init();

        /**
         * @brief Clean up resources and reset hardware to safe state
         */
        void cleanup();

        //--------------------------------------------------------------------------
        // Leg Control
        //--------------------------------------------------------------------------

        /**
         * @brief Set the position of a specific leg
         *
         * @param leg_num Leg number (0 to NUM_LEGS-1)
         * @param position Target joint angles
         * @return true if command was successful
         * @return false if command failed (check getLastError())
         */
        bool setLegPosition(uint8_t leg_num, const LegPosition &position);

        /**
         * @brief Get the current position of a specific leg
         *
         * @param leg_num Leg number (0 to NUM_LEGS-1)
         * @param[out] position Current joint angles
         * @return true if query was successful
         * @return false if query failed (check getLastError())
         */
        bool getLegPosition(uint8_t leg_num, LegPosition &position) const;

        /**
         * @brief Center all legs to their neutral position
         *
         * @return true if command was successful
         * @return false if command failed (check getLastError())
         */
        bool centerAll();

        //--------------------------------------------------------------------------
        // Sensor Access
        //--------------------------------------------------------------------------

        /**
         * @brief Get IMU sensor data
         *
         * @param[out] data IMU sensor readings
         * @return true if data retrieval was successful
         * @return false if data retrieval failed (check getLastError())
         */
        bool getImuData(ImuData &data) const;

        /**
         * @brief Set the active sensor type
         *
         * @param sensor_type Type of sensor to use
         * @return true if sensor type was set successfully
         * @return false if sensor type change failed
         */
        bool setSensorType(SensorType sensor_type);

        /**
         * @brief Get the current active sensor type
         *
         * @param[out] sensor_type Current sensor type
         * @return true if query was successful
         * @return false if query failed
         */
        bool getSensorType(SensorType &sensor_type) const;

        //--------------------------------------------------------------------------
        // Calibration
        //--------------------------------------------------------------------------

        /**
         * @brief Set calibration offsets for a specific leg
         *
         * @param leg_num Leg number (0 to NUM_LEGS-1)
         * @param hip_offset Hip joint calibration offset
         * @param knee_offset Knee joint calibration offset
         * @param ankle_offset Ankle joint calibration offset
         * @return true if calibration was successful
         * @return false if calibration failed (check getLastError())
         */
        bool setCalibration(uint8_t leg_num, int16_t hip_offset, int16_t knee_offset, int16_t ankle_offset);

        //--------------------------------------------------------------------------
        // Utility Methods
        //--------------------------------------------------------------------------

        /**
         * @brief Get current time from a monotonic clock
         *
         * @return double Time in seconds
         */
        double getCurrentTime() const;

        //--------------------------------------------------------------------------
        // Error Handling
        //--------------------------------------------------------------------------

        /**
         * @brief Get detailed information about the last error
         *
         * @return ErrorInfo Error information structure
         */
        ErrorInfo getLastError() const;

        /**
         * @brief Get the message describing the last error
         *
         * @return std::string Error message
         */
        std::string getLastErrorMessage() const;

        /**
         * @brief Get the error code of the last error
         *
         * @return int Error code
         */
        int getLastErrorCode() const;

        /**
         * @brief Get the category of the last error
         *
         * @return ErrorCategory Error category
         */
        ErrorCategory getLastErrorCategory() const;

        //--------------------------------------------------------------------------
        // Debug Utilities
        //--------------------------------------------------------------------------

        /**
         * @brief Print leg position to stdout for debugging
         *
         * @param position Leg position to print
         */
        static void printLegPosition(const LegPosition &position);

        /**
         * @brief Print IMU data to stdout for debugging
         *
         * @param data IMU data to print
         */
        static void printImuData(const ImuData &data);

    private:
        /**
         * @brief Implementation pointer (PIMPL idiom)
         */
        std::unique_ptr<HexapodImpl> pImpl;
    };

} // namespace hexapod

// For backward compatibility, import types into global namespace
// Note: In a new project, this wouldn't be recommended, but it ensures
// compatibility with existing code.
using hexapod::ErrorCategory;
using hexapod::ErrorInfo;
using hexapod::Hexapod;
using hexapod::ImuData;
using hexapod::JointAngles;
using hexapod::LegPosition;

// Global constants for backward compatibility
constexpr int NUM_LEGS = hexapod::Config::NUM_LEGS;
constexpr int SERVOS_PER_LEG = hexapod::Config::SERVOS_PER_LEG;
constexpr int TOTAL_SERVOS = hexapod::Config::TOTAL_SERVOS;
constexpr int COXA_LENGTH = hexapod::Dimensions::COXA_LENGTH;
constexpr int FEMUR_LENGTH = hexapod::Dimensions::FEMUR_LENGTH;
constexpr int TIBIA_LENGTH = hexapod::Dimensions::TIBIA_LENGTH;
constexpr int HIP_MIN_ANGLE = hexapod::AngleLimits::HIP_MIN;
constexpr int HIP_MAX_ANGLE = hexapod::AngleLimits::HIP_MAX;
constexpr int KNEE_MIN_ANGLE = hexapod::AngleLimits::KNEE_MIN;
constexpr int KNEE_MAX_ANGLE = hexapod::AngleLimits::KNEE_MAX;
constexpr int ANKLE_MIN_ANGLE = hexapod::AngleLimits::ANKLE_MIN;
constexpr int ANKLE_MAX_ANGLE = hexapod::AngleLimits::ANKLE_MAX;

#endif /* HEXAPOD_HPP */
