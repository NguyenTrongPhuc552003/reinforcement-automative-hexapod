#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <string>
#include <memory>
#include <fstream>
#include <vector>
#include <mutex>
#include <map>
#include <chrono>
#include <atomic>
#include "hexapod.hpp" // For peripheral data types

namespace logger
{

    /**
     * @brief Log severity levels
     */
    enum class LogLevel
    {
        TRACE = 0,   ///< Detailed tracing information
        DEBUG = 1,   ///< Debug information
        INFO = 2,    ///< General information messages
        WARNING = 3, ///< Warning messages
        ERROR = 4,   ///< Error messages
        FATAL = 5    ///< Critical errors
    };

    /**
     * @brief Log category for different system components
     */
    enum class LogCategory
    {
        SYSTEM,   ///< System-level messages
        IMU,      ///< IMU (MPU6050) related messages
        SERVO,    ///< Servo motor related messages
        PWM,      ///< PWM controller (PCA9685) related messages
        MOVEMENT, ///< Robot movement related messages
        NETWORK,  ///< Network communication related messages
        SENSOR    ///< Other sensor related messages
    };

    /**
     * @brief Configuration for the logger
     */
    struct LoggerConfig
    {
        bool console_output = true;              ///< Output logs to console
        bool file_output = false;                ///< Output logs to file
        std::string log_file = "hexapod.log";    ///< Log filename
        LogLevel console_level = LogLevel::INFO; ///< Minimum level for console output
        LogLevel file_level = LogLevel::DEBUG;   ///< Minimum level for file output
        bool show_timestamp = true;              ///< Include timestamp in log messages
        bool show_category = true;               ///< Include category in log messages
        size_t max_file_size = 10 * 1024 * 1024; ///< Maximum log file size in bytes
        int max_backup_files = 3;                ///< Maximum number of backup log files
        bool flush_each_write = false;           ///< Flush file after each write
    };

    /**
     * @brief Peripheral data for logging
     */
    struct PeripheralLogData
    {
        // IMU data (MPU6050)
        ImuData imu_data;
        bool imu_valid = false;

        // PCA9685 PWM data
        struct PwmChannelData
        {
            uint8_t channel;
            uint16_t pulse_us;
            bool enabled;
        };
        std::vector<PwmChannelData> pwm_channels;

        // Servo angle data
        struct ServoData
        {
            uint8_t id;
            int16_t angle;
            int16_t target_angle;
            uint16_t pulse_us;
        };
        std::vector<ServoData> servo_data;

        // Timestamp
        double timestamp = 0.0;
    };

    // Forward declaration for PIMPL idiom
    class LoggerImpl;

    /**
     * @brief Thread-safe logger for the hexapod robot
     *
     * This class provides logging functionality for all components of the robot,
     * with support for different log levels, categories, and outputs (console/file).
     */
    class Logger
    {
    public:
        /**
         * @brief Get the singleton logger instance
         *
         * @return Logger& Logger instance
         */
        static Logger &getInstance();

        /**
         * @brief Initialize the logger
         *
         * @param config Logger configuration
         * @return true if initialization was successful
         */
        bool init(const LoggerConfig &config = LoggerConfig());

        /**
         * @brief Log a message
         *
         * @param level Severity level
         * @param category Message category
         * @param message Log message
         */
        void log(LogLevel level, LogCategory category, const std::string &message);

        /**
         * @brief Convenience method to log a trace message
         *
         * @param category Message category
         * @param message Log message
         */
        void trace(LogCategory category, const std::string &message);

        /**
         * @brief Convenience method to log a debug message
         *
         * @param category Message category
         * @param message Log message
         */
        void debug(LogCategory category, const std::string &message);

        /**
         * @brief Convenience method to log an info message
         *
         * @param category Message category
         * @param message Log message
         */
        void info(LogCategory category, const std::string &message);

        /**
         * @brief Convenience method to log a warning message
         *
         * @param category Message category
         * @param message Log message
         */
        void warning(LogCategory category, const std::string &message);

        /**
         * @brief Convenience method to log an error message
         *
         * @param category Message category
         * @param message Log message
         */
        void error(LogCategory category, const std::string &message);

        /**
         * @brief Convenience method to log a fatal message
         *
         * @param category Message category
         * @param message Log message
         */
        void fatal(LogCategory category, const std::string &message);

        /**
         * @brief Log IMU (MPU6050) data
         *
         * @param imu_data IMU data to log
         */
        void logImuData(const ImuData &imu_data);

        /**
         * @brief Log PWM channel data
         *
         * @param channel Channel number
         * @param pulse_us Pulse width in microseconds
         * @param enabled Whether the channel is enabled
         */
        void logPwmChannel(uint8_t channel, uint16_t pulse_us, bool enabled);

        /**
         * @brief Log servo data
         *
         * @param id Servo ID
         * @param angle Current angle
         * @param target_angle Target angle
         * @param pulse_us Pulse width in microseconds
         */
        void logServoData(uint8_t id, int16_t angle, int16_t target_angle, uint16_t pulse_us);

        /**
         * @brief Log all peripheral data in a single frame
         *
         * @param data Peripheral data to log
         */
        void logPeripheralFrame(const PeripheralLogData &data);

        /**
         * @brief Save peripheral log data to a CSV file
         *
         * @param filename Output filename
         * @return true if save was successful
         */
        bool savePeripheralLogToCsv(const std::string &filename);

        /**
         * @brief Get the current peripheral log data
         *
         * @return PeripheralLogData Current peripheral log data
         */
        PeripheralLogData getPeripheralData() const;

        /**
         * @brief Clear the peripheral log data
         */
        void clearPeripheralLog();

        /**
         * @brief Set the maximum number of peripheral log frames to keep
         *
         * @param max_frames Maximum number of frames
         */
        void setMaxPeripheralLogFrames(size_t max_frames);

        /**
         * @brief Get the number of peripheral log frames
         *
         * @return size_t Number of frames
         */
        size_t getPeripheralLogFrameCount() const;

        /**
         * @brief Get a specific peripheral log frame
         *
         * @param index Frame index
         * @return PeripheralLogData Frame data
         */
        PeripheralLogData getPeripheralLogFrame(size_t index) const;

        /**
         * @brief Start automatic periodic logging of peripheral data
         *
         * @param interval_ms Logging interval in milliseconds
         * @return true if automatic logging was started successfully
         */
        bool startPeriodicLogging(uint32_t interval_ms = 100);

        /**
         * @brief Stop automatic periodic logging of peripheral data
         */
        void stopPeriodicLogging();

        /**
         * @brief Check if automatic periodic logging is running
         *
         * @return true if automatic logging is running
         */
        bool isPeriodicLoggingRunning() const;

        /**
         * @brief Set the hexapod interface for data acquisition
         *
         * @param hexapod Hexapod interface
         */
        void setHexapod(Hexapod *hexapod);

        /**
         * @brief Convert log level to string
         *
         * @param level Log level
         * @return std::string String representation
         */
        static std::string logLevelToString(LogLevel level);

        /**
         * @brief Convert log category to string
         *
         * @param category Log category
         * @return std::string String representation
         */
        static std::string logCategoryToString(LogCategory category);

        /**
         * @brief Flush log file buffers
         */
        void flush();

        /**
         * @brief Close the logger and release resources
         */
        void close();

    private:
        // Private constructor and destructor for singleton pattern
        Logger();
        ~Logger();

        // Non-copyable and non-movable
        Logger(const Logger &) = delete;
        Logger &operator=(const Logger &) = delete;
        Logger(Logger &&) = delete;
        Logger &operator=(Logger &&) = delete;

        // Implementation details (PIMPL idiom)
        std::unique_ptr<LoggerImpl> pImpl;
    };

} // namespace hexapod

// For convenience, add these macros for easier logging
#define LOG_TRACE(category, message) logger::Logger::getInstance().trace(category, message)
#define LOG_DEBUG(category, message) logger::Logger::getInstance().debug(category, message)
#define LOG_INFO(category, message) logger::Logger::getInstance().info(category, message)
#define LOG_WARNING(category, message) logger::Logger::getInstance().warning(category, message)
#define LOG_ERROR(category, message) logger::Logger::getInstance().error(category, message)
#define LOG_FATAL(category, message) logger::Logger::getInstance().fatal(category, message)

#endif // LOGGER_HPP
