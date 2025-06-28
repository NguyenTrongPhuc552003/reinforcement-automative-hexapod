#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <queue>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include "logger.hpp"

namespace logger
{

    // Implementation class for PIMPL idiom
    class LoggerImpl
    {
    public:
        LoggerImpl() : m_initialized(false),
                       m_hexapod(nullptr),
                       m_periodic_logging_active(false),
                       m_max_peripheral_frames(1000)
        {
        }

        ~LoggerImpl()
        {
            close();
        }

        bool init(const LoggerConfig &config)
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            m_config = config;

            // Open log file if file output is enabled
            if (m_config.file_output)
            {
                try
                {
                    // Create directory if needed
                    std::filesystem::path logPath(m_config.log_file);
                    if (logPath.has_parent_path() && !std::filesystem::exists(logPath.parent_path()))
                    {
                        std::filesystem::create_directories(logPath.parent_path());
                    }

                    // Open log file
                    m_log_file.open(m_config.log_file, std::ios::out | std::ios::app);
                    if (!m_log_file.is_open())
                    {
                        std::cerr << "Failed to open log file: " << m_config.log_file << std::endl;
                        return false;
                    }
                }
                catch (const std::exception &e)
                {
                    std::cerr << "Exception during log file initialization: " << e.what() << std::endl;
                    return false;
                }
            }

            m_initialized = true;
            log(LogLevel::INFO, LogCategory::SYSTEM, "Logger initialized");
            return true;
        }

        void log(LogLevel level, LogCategory category, const std::string &message)
        {
            if (!m_initialized)
                return;

            std::lock_guard<std::mutex> lock(m_mutex);

            std::stringstream formatted_message;

            // Add timestamp if enabled
            if (m_config.show_timestamp)
            {
                auto now = std::chrono::system_clock::now();
                auto time_t_now = std::chrono::system_clock::to_time_t(now);
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              now.time_since_epoch()) %
                          1000;

                formatted_message << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S")
                                  << '.' << std::setfill('0') << std::setw(3) << ms.count() << ' ';
            }

            // Add log level
            formatted_message << '[' << Logger::logLevelToString(level) << ']';

            // Add category if enabled
            if (m_config.show_category)
            {
                formatted_message << '[' << Logger::logCategoryToString(category) << ']';
            }

            formatted_message << ' ' << message;

            // Write to console if enabled and level is sufficient
            if (m_config.console_output && level >= m_config.console_level)
            {
                if (level >= LogLevel::ERROR)
                {
                    std::cerr << formatted_message.str() << std::endl;
                }
                else
                {
                    std::cout << formatted_message.str() << std::endl;
                }
            }

            // Write to file if enabled and level is sufficient
            if (m_config.file_output && m_log_file.is_open() && level >= m_config.file_level)
            {
                m_log_file << formatted_message.str() << std::endl;

                // Check if we need to flush
                if (m_config.flush_each_write)
                {
                    m_log_file.flush();
                }

                // Check if we need to rotate log files
                if (m_config.max_file_size > 0 &&
                    static_cast<size_t>(m_log_file.tellp()) > m_config.max_file_size)
                {
                    rotateLogFiles();
                }
            }
        }

        void rotateLogFiles()
        {
            // Close current log file
            m_log_file.close();

            // Rotate old log files
            for (int i = m_config.max_backup_files - 1; i >= 1; i--)
            {
                std::string old_name = m_config.log_file + "." + std::to_string(i);
                std::string new_name = m_config.log_file + "." + std::to_string(i + 1);

                // Try to remove the oldest log file if it exists
                if (i == m_config.max_backup_files - 1)
                {
                    std::filesystem::remove(new_name);
                }

                // Rename existing backup files
                if (std::filesystem::exists(old_name))
                {
                    std::filesystem::rename(old_name, new_name);
                }
            }

            // Rename the current log file to .1
            std::filesystem::rename(m_config.log_file, m_config.log_file + ".1");

            // Open a new log file
            m_log_file.open(m_config.log_file, std::ios::out | std::ios::app);
        }

        void logImuData(const ImuData &imu_data)
        {
            if (!m_initialized)
                return;

            std::lock_guard<std::mutex> lock(m_mutex);

            // Update current peripheral data
            m_current_peripheral_data.imu_data = imu_data;
            m_current_peripheral_data.imu_valid = true;
            m_current_peripheral_data.timestamp = getCurrentTime();

            // Log the data
            std::stringstream ss;
            ss << "IMU Data: "
               << "AccelX=" << imu_data.getAccelX() << "g, "
               << "AccelY=" << imu_data.getAccelY() << "g, "
               << "AccelZ=" << imu_data.getAccelZ() << "g, "
               << "GyroX=" << imu_data.getGyroX() << "°/s, "
               << "GyroY=" << imu_data.getGyroY() << "°/s, "
               << "GyroZ=" << imu_data.getGyroZ() << "°/s";

            log(LogLevel::DEBUG, LogCategory::IMU, ss.str());
        }

        void logPwmChannel(uint8_t channel, uint16_t pulse_us, bool enabled)
        {
            if (!m_initialized)
                return;

            std::lock_guard<std::mutex> lock(m_mutex);

            // Update current peripheral data
            bool channel_found = false;
            for (auto &pwm : m_current_peripheral_data.pwm_channels)
            {
                if (pwm.channel == channel)
                {
                    pwm.pulse_us = pulse_us;
                    pwm.enabled = enabled;
                    channel_found = true;
                    break;
                }
            }

            if (!channel_found)
            {
                PeripheralLogData::PwmChannelData pwm_data;
                pwm_data.channel = channel;
                pwm_data.pulse_us = pulse_us;
                pwm_data.enabled = enabled;
                m_current_peripheral_data.pwm_channels.push_back(pwm_data);
            }

            m_current_peripheral_data.timestamp = getCurrentTime();

            // Log the data
            std::stringstream ss;
            ss << "PWM Channel " << static_cast<int>(channel) << ": "
               << pulse_us << "μs, "
               << (enabled ? "enabled" : "disabled");

            log(LogLevel::DEBUG, LogCategory::PWM, ss.str());
        }

        void logServoData(uint8_t id, int16_t angle, int16_t target_angle, uint16_t pulse_us)
        {
            if (!m_initialized)
                return;

            std::lock_guard<std::mutex> lock(m_mutex);

            // Update current peripheral data
            bool servo_found = false;
            for (auto &servo : m_current_peripheral_data.servo_data)
            {
                if (servo.id == id)
                {
                    servo.angle = angle;
                    servo.target_angle = target_angle;
                    servo.pulse_us = pulse_us;
                    servo_found = true;
                    break;
                }
            }

            if (!servo_found)
            {
                PeripheralLogData::ServoData servo_data;
                servo_data.id = id;
                servo_data.angle = angle;
                servo_data.target_angle = target_angle;
                servo_data.pulse_us = pulse_us;
                m_current_peripheral_data.servo_data.push_back(servo_data);
            }

            m_current_peripheral_data.timestamp = getCurrentTime();

            // Log the data
            std::stringstream ss;
            ss << "Servo " << static_cast<int>(id) << ": "
               << "Angle=" << angle << "°, "
               << "Target=" << target_angle << "°, "
               << "PWM=" << pulse_us << "μs";

            log(LogLevel::DEBUG, LogCategory::SERVO, ss.str());
        }

        void logPeripheralFrame(const PeripheralLogData &data)
        {
            if (!m_initialized)
                return;

            std::lock_guard<std::mutex> lock(m_mutex);

            // Store the frame in the log
            m_peripheral_log_frames.push_back(data);

            // Limit the number of frames in memory
            if (m_peripheral_log_frames.size() > m_max_peripheral_frames)
            {
                m_peripheral_log_frames.erase(m_peripheral_log_frames.begin());
            }

            // Update current data
            m_current_peripheral_data = data;

            // Log frame information
            std::stringstream ss;
            ss << "Peripheral log frame captured: "
               << m_peripheral_log_frames.size() << " frames total";

            log(LogLevel::DEBUG, LogCategory::SYSTEM, ss.str());
        }

        bool savePeripheralLogToCsv(const std::string &filename)
        {
            if (!m_initialized || m_peripheral_log_frames.empty())
            {
                log(LogLevel::ERROR, LogCategory::SYSTEM, "Cannot save peripheral log: No data or not initialized");
                return false;
            }

            std::lock_guard<std::mutex> lock(m_mutex);

            try
            {
                // Open output file
                std::ofstream csv_file(filename);
                if (!csv_file.is_open())
                {
                    log(LogLevel::ERROR, LogCategory::SYSTEM, "Failed to open CSV file: " + filename);
                    return false;
                }

                // Write CSV header
                csv_file << "Timestamp,";

                // IMU headers
                csv_file << "IMU_AccelX,IMU_AccelY,IMU_AccelZ,";
                csv_file << "IMU_GyroX,IMU_GyroY,IMU_GyroZ,";

                // PWM channel headers - find the maximum channel number used
                int max_pwm_channel = -1;
                for (const auto &frame : m_peripheral_log_frames)
                {
                    for (const auto &pwm : frame.pwm_channels)
                    {
                        max_pwm_channel = std::max(max_pwm_channel, static_cast<int>(pwm.channel));
                    }
                }

                for (int i = 0; i <= max_pwm_channel; i++)
                {
                    csv_file << "PWM_Ch" << i << "_us,";
                    csv_file << "PWM_Ch" << i << "_enabled,";
                }

                // Servo data headers - find the maximum servo ID used
                int max_servo_id = -1;
                for (const auto &frame : m_peripheral_log_frames)
                {
                    for (const auto &servo : frame.servo_data)
                    {
                        max_servo_id = std::max(max_servo_id, static_cast<int>(servo.id));
                    }
                }

                for (int i = 0; i <= max_servo_id; i++)
                {
                    csv_file << "Servo" << i << "_Angle,";
                    csv_file << "Servo" << i << "_Target,";
                    csv_file << "Servo" << i << "_PWM,";
                }

                // Remove last comma and add newline
                csv_file.seekp(-1, std::ios_base::cur);
                csv_file << std::endl;

                // Write data rows
                for (const auto &frame : m_peripheral_log_frames)
                {
                    // Timestamp
                    csv_file << std::fixed << std::setprecision(6) << frame.timestamp << ",";

                    // IMU data
                    if (frame.imu_valid)
                    {
                        csv_file << frame.imu_data.getAccelX() << ","
                                 << frame.imu_data.getAccelY() << ","
                                 << frame.imu_data.getAccelZ() << ","
                                 << frame.imu_data.getGyroX() << ","
                                 << frame.imu_data.getGyroY() << ","
                                 << frame.imu_data.getGyroZ() << ",";
                    }
                    else
                    {
                        csv_file << ",,,,,,";
                    }

                    // PWM channel data
                    for (int i = 0; i <= max_pwm_channel; i++)
                    {
                        bool found = false;
                        for (const auto &pwm : frame.pwm_channels)
                        {
                            if (pwm.channel == i)
                            {
                                csv_file << pwm.pulse_us << ","
                                         << (pwm.enabled ? "1" : "0") << ",";
                                found = true;
                                break;
                            }
                        }
                        if (!found)
                        {
                            csv_file << ",,";
                        }
                    }

                    // Servo data
                    for (int i = 0; i <= max_servo_id; i++)
                    {
                        bool found = false;
                        for (const auto &servo : frame.servo_data)
                        {
                            if (servo.id == i)
                            {
                                csv_file << servo.angle << ","
                                         << servo.target_angle << ","
                                         << servo.pulse_us << ",";
                                found = true;
                                break;
                            }
                        }
                        if (!found)
                        {
                            csv_file << ",,,";
                        }
                    }

                    // Remove last comma and add newline
                    csv_file.seekp(-1, std::ios_base::cur);
                    csv_file << std::endl;
                }

                csv_file.close();

                std::stringstream ss;
                ss << "Saved " << m_peripheral_log_frames.size() << " peripheral log frames to " << filename;
                log(LogLevel::INFO, LogCategory::SYSTEM, ss.str());
                return true;
            }
            catch (const std::exception &e)
            {
                std::stringstream ss;
                ss << "Exception while saving peripheral log to CSV: " << e.what();
                log(LogLevel::ERROR, LogCategory::SYSTEM, ss.str());
                return false;
            }
        }

        PeripheralLogData getPeripheralData() const
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            return m_current_peripheral_data;
        }

        void clearPeripheralLog()
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_peripheral_log_frames.clear();
            log(LogLevel::INFO, LogCategory::SYSTEM, "Peripheral log cleared");
        }

        void setMaxPeripheralLogFrames(size_t max_frames)
        {
            if (max_frames < 1)
                max_frames = 1;

            std::lock_guard<std::mutex> lock(m_mutex);
            m_max_peripheral_frames = max_frames;

            // Resize if necessary
            if (m_peripheral_log_frames.size() > m_max_peripheral_frames)
            {
                m_peripheral_log_frames.erase(
                    m_peripheral_log_frames.begin(),
                    m_peripheral_log_frames.begin() + (m_peripheral_log_frames.size() - m_max_peripheral_frames));
            }

            std::stringstream ss;
            ss << "Max peripheral log frames set to " << m_max_peripheral_frames;
            log(LogLevel::INFO, LogCategory::SYSTEM, ss.str());
        }

        size_t getPeripheralLogFrameCount() const
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            return m_peripheral_log_frames.size();
        }

        PeripheralLogData getPeripheralLogFrame(size_t index) const
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (index < m_peripheral_log_frames.size())
            {
                return m_peripheral_log_frames[index];
            }
            return PeripheralLogData(); // Return empty data if index is out of range
        }

        bool startPeriodicLogging(uint32_t interval_ms)
        {
            if (m_periodic_logging_active)
            {
                log(LogLevel::WARNING, LogCategory::SYSTEM, "Periodic logging is already active");
                return true; // Not an error
            }

            if (interval_ms < 10)
                interval_ms = 10; // Minimum 10ms interval

            if (!m_hexapod)
            {
                log(LogLevel::ERROR, LogCategory::SYSTEM, "Cannot start periodic logging: No hexapod interface");
                return false;
            }

            // Create and start the logging thread
            m_periodic_logging_active = true;
            m_periodic_logging_thread = std::thread([this, interval_ms]()
                                                    {
            log(LogLevel::INFO, LogCategory::SYSTEM, 
                "Periodic logging started with " + std::to_string(interval_ms) + "ms interval");
            
            while (m_periodic_logging_active) {
                capturePeripheralData();
                std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
            }
            
            log(LogLevel::INFO, LogCategory::SYSTEM, "Periodic logging stopped"); });

            return true;
        }

        void stopPeriodicLogging()
        {
            if (!m_periodic_logging_active)
                return;

            m_periodic_logging_active = false;

            if (m_periodic_logging_thread.joinable())
            {
                m_periodic_logging_thread.join();
            }
        }

        bool isPeriodicLoggingRunning() const
        {
            return m_periodic_logging_active;
        }

        void setHexapod(Hexapod *hexapod)
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_hexapod = hexapod;
        }

        void capturePeripheralData()
        {
            if (!m_hexapod)
                return;

            PeripheralLogData data;
            data.timestamp = getCurrentTime();

            // Capture IMU data
            ImuData imu_data;
            if (m_hexapod->getImuData(imu_data))
            {
                data.imu_data = imu_data;
                data.imu_valid = true;
            }

            // Capture leg position data
            for (uint8_t leg = 0; leg < hexapod::Config::NUM_LEGS; leg++)
            {
                LegPosition position;
                if (m_hexapod->getLegPosition(leg, position))
                {
                    // Add as servo data
                    for (int joint = 0; joint < hexapod::Config::SERVOS_PER_LEG; joint++)
                    {
                        PeripheralLogData::ServoData servo_data;
                        servo_data.id = leg * hexapod::Config::SERVOS_PER_LEG + joint;

                        // Set angle based on joint type
                        switch (joint)
                        {
                        case 0: // Hip
                            servo_data.angle = position.getHip();
                            break;
                        case 1: // Knee
                            servo_data.angle = position.getKnee();
                            break;
                        case 2: // Ankle
                            servo_data.angle = position.getAnkle();
                            break;
                        }

                        servo_data.target_angle = servo_data.angle; // Actual = target in this simple case

                        // Calculate approximate PWM value (this is an approximation)
                        // Assuming 1500μs is center, and each degree is ~10μs
                        servo_data.pulse_us = 1500 + servo_data.angle * 10;

                        data.servo_data.push_back(servo_data);
                    }
                }
            }

            // Store the captured data
            logPeripheralFrame(data);
        }

        void flush()
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            if (m_log_file.is_open())
            {
                m_log_file.flush();
            }
        }

        void close()
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            // Stop periodic logging if active
            stopPeriodicLogging();

            // Close log file
            if (m_log_file.is_open())
            {
                m_log_file.close();
            }

            m_initialized = false;
        }

        double getCurrentTime() const
        {
            auto now = std::chrono::steady_clock::now();
            auto duration = now.time_since_epoch();
            return std::chrono::duration<double>(duration).count();
        }

        // Member variables
        LoggerConfig m_config;
        bool m_initialized;

        std::ofstream m_log_file;
        mutable std::mutex m_mutex;

        PeripheralLogData m_current_peripheral_data;
        std::vector<PeripheralLogData> m_peripheral_log_frames;
        size_t m_max_peripheral_frames;

        Hexapod *m_hexapod;

        std::atomic<bool> m_periodic_logging_active;
        std::thread m_periodic_logging_thread;
    };

    //==============================================================================
    // Logger Implementation
    //==============================================================================

    Logger &Logger::getInstance()
    {
        static Logger instance;
        return instance;
    }

    Logger::Logger() : pImpl(std::make_unique<LoggerImpl>()) {}

    Logger::~Logger()
    {
        close();
    }

    bool Logger::init(const LoggerConfig &config)
    {
        return pImpl->init(config);
    }

    void Logger::log(LogLevel level, LogCategory category, const std::string &message)
    {
        pImpl->log(level, category, message);
    }

    void Logger::trace(LogCategory category, const std::string &message)
    {
        log(LogLevel::TRACE, category, message);
    }

    void Logger::debug(LogCategory category, const std::string &message)
    {
        log(LogLevel::DEBUG, category, message);
    }

    void Logger::info(LogCategory category, const std::string &message)
    {
        log(LogLevel::INFO, category, message);
    }

    void Logger::warning(LogCategory category, const std::string &message)
    {
        log(LogLevel::WARNING, category, message);
    }

    void Logger::error(LogCategory category, const std::string &message)
    {
        log(LogLevel::ERROR, category, message);
    }

    void Logger::fatal(LogCategory category, const std::string &message)
    {
        log(LogLevel::FATAL, category, message);
    }

    void Logger::logImuData(const ImuData &imu_data)
    {
        pImpl->logImuData(imu_data);
    }

    void Logger::logPwmChannel(uint8_t channel, uint16_t pulse_us, bool enabled)
    {
        pImpl->logPwmChannel(channel, pulse_us, enabled);
    }

    void Logger::logServoData(uint8_t id, int16_t angle, int16_t target_angle, uint16_t pulse_us)
    {
        pImpl->logServoData(id, angle, target_angle, pulse_us);
    }

    void Logger::logPeripheralFrame(const PeripheralLogData &data)
    {
        pImpl->logPeripheralFrame(data);
    }

    bool Logger::savePeripheralLogToCsv(const std::string &filename)
    {
        return pImpl->savePeripheralLogToCsv(filename);
    }

    PeripheralLogData Logger::getPeripheralData() const
    {
        return pImpl->getPeripheralData();
    }

    void Logger::clearPeripheralLog()
    {
        pImpl->clearPeripheralLog();
    }

    void Logger::setMaxPeripheralLogFrames(size_t max_frames)
    {
        pImpl->setMaxPeripheralLogFrames(max_frames);
    }

    size_t Logger::getPeripheralLogFrameCount() const
    {
        return pImpl->getPeripheralLogFrameCount();
    }

    PeripheralLogData Logger::getPeripheralLogFrame(size_t index) const
    {
        return pImpl->getPeripheralLogFrame(index);
    }

    bool Logger::startPeriodicLogging(uint32_t interval_ms)
    {
        return pImpl->startPeriodicLogging(interval_ms);
    }

    void Logger::stopPeriodicLogging()
    {
        pImpl->stopPeriodicLogging();
    }

    bool Logger::isPeriodicLoggingRunning() const
    {
        return pImpl->isPeriodicLoggingRunning();
    }

    void Logger::setHexapod(Hexapod *hexapod)
    {
        pImpl->setHexapod(hexapod);
    }

    void Logger::flush()
    {
        pImpl->flush();
    }

    void Logger::close()
    {
        pImpl->close();
    }

    std::string Logger::logLevelToString(LogLevel level)
    {
        switch (level)
        {
        case LogLevel::TRACE:
            return "TRACE";
        case LogLevel::DEBUG:
            return "DEBUG";
        case LogLevel::INFO:
            return "INFO";
        case LogLevel::WARNING:
            return "WARN";
        case LogLevel::ERROR:
            return "ERROR";
        case LogLevel::FATAL:
            return "FATAL";
        default:
            return "UNKNOWN";
        }
    }

    std::string Logger::logCategoryToString(LogCategory category)
    {
        switch (category)
        {
        case LogCategory::SYSTEM:
            return "SYSTEM";
        case LogCategory::IMU:
            return "IMU";
        case LogCategory::SERVO:
            return "SERVO";
        case LogCategory::PWM:
            return "PWM";
        case LogCategory::MOVEMENT:
            return "MOVEMENT";
        case LogCategory::NETWORK:
            return "NETWORK";
        case LogCategory::SENSOR:
            return "SENSOR";
        default:
            return "UNKNOWN";
        }
    }

} // namespace hexapod
