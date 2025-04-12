#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <stdexcept>
#include <filesystem>
#include "td3learn/utils.hpp"

namespace td3learn
{
    namespace utils
    {

        // Logger implementation
        class LoggerImpl
        {
        public:
            static void init(
                const std::string &name,
                Logger::Level level,
                bool log_to_file,
                const std::string &file_path)
            {
                std::lock_guard<std::mutex> lock(getMutex());
                instance().name_ = name;
                instance().level_ = level;

                if (log_to_file)
                {
                    instance().log_file_.open(file_path, std::ios::out | std::ios::app);
                    if (!instance().log_file_.is_open())
                    {
                        std::cerr << "Failed to open log file: " << file_path << std::endl;
                    }
                }
            }

            static void setLevel(Logger::Level level)
            {
                std::lock_guard<std::mutex> lock(getMutex());
                instance().level_ = level;
            }

            static void log(Logger::Level level, const std::string &message)
            {
                if (level < instance().level_)
                {
                    return;
                }

                std::lock_guard<std::mutex> lock(getMutex());

                // Get current time
                auto now = std::chrono::system_clock::now();
                auto time_t_now = std::chrono::system_clock::to_time_t(now);
                auto tm_now = std::localtime(&time_t_now);

                // Format output
                std::stringstream ss;
                ss << std::put_time(tm_now, "%Y-%m-%d %H:%M:%S") << " ";
                ss << instance().name_ << " [" << getLevelString(level) << "] " << message;

                // Output to console
                std::cout << ss.str() << std::endl;

                // Output to file if enabled
                if (instance().log_file_.is_open())
                {
                    instance().log_file_ << ss.str() << std::endl;
                    instance().log_file_.flush();
                }
            }

        private:
            LoggerImpl() : level_(Logger::Level::INFO), name_("td3learn") {}

            static LoggerImpl &instance()
            {
                static LoggerImpl instance;
                return instance;
            }

            static std::mutex &getMutex()
            {
                static std::mutex mutex;
                return mutex;
            }

            static std::string getLevelString(Logger::Level level)
            {
                switch (level)
                {
                case Logger::Level::DEBUG:
                    return "DEBUG";
                case Logger::Level::INFO:
                    return "INFO";
                case Logger::Level::WARNING:
                    return "WARNING";
                case Logger::Level::ERROR:
                    return "ERROR";
                case Logger::Level::FATAL:
                    return "FATAL";
                default:
                    return "UNKNOWN";
                }
            }

            Logger::Level level_;
            std::string name_;
            std::ofstream log_file_;
        };

        void Logger::init(
            const std::string &name,
            Level level,
            bool log_to_file,
            const std::string &file_path)
        {
            LoggerImpl::init(name, level, log_to_file, file_path);
        }

        void Logger::setLevel(Level level)
        {
            LoggerImpl::setLevel(level);
        }

        void Logger::log(Level level, const std::string &message)
        {
            LoggerImpl::log(level, message);
        }

        void Logger::debug(const std::string &message)
        {
            log(Level::DEBUG, message);
        }

        void Logger::info(const std::string &message)
        {
            log(Level::INFO, message);
        }

        void Logger::warning(const std::string &message)
        {
            log(Level::WARNING, message);
        }

        void Logger::error(const std::string &message)
        {
            log(Level::ERROR, message);
        }

        void Logger::fatal(const std::string &message)
        {
            log(Level::FATAL, message);
        }

        // Timer implementation
        using Clock = std::chrono::high_resolution_clock;
        using TimePoint = std::chrono::time_point<Clock>;

        static TimePoint start_time;

        void Timer::start()
        {
            start_time = Clock::now();
        }

        double Timer::stop()
        {
            const auto end_time = Clock::now();
            const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            return static_cast<double>(duration.count()) / 1000.0; // Convert to milliseconds
        }

        double Timer::elapsed()
        {
            const auto current_time = Clock::now();
            const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time);
            return static_cast<double>(duration.count()) / 1000.0; // Convert to milliseconds
        }

        // Normalizer implementation
        Normalizer::Normalizer(const std::vector<Scalar> &mean, const std::vector<Scalar> &std)
            : mean_(mean), std_(std), count_(mean.size(), 0)
        {
            if (mean.size() != std.size())
            {
                throw std::invalid_argument("Mean and std vectors must have the same size");
            }
        }

        Normalizer::Normalizer(size_t dimension)
            : mean_(dimension, 0.0f), std_(dimension, 1.0f), count_(dimension, 0)
        {
        }

        std::vector<Scalar> Normalizer::normalize(const std::vector<Scalar> &input) const
        {
            if (input.size() != mean_.size())
            {
                throw std::invalid_argument("Input dimension does not match normalizer dimension");
            }

            std::vector<Scalar> output(input.size());
            for (size_t i = 0; i < input.size(); i++)
            {
                output[i] = (input[i] - mean_[i]) / std_[i];
            }

            return output;
        }

        std::vector<Scalar> Normalizer::denormalize(const std::vector<Scalar> &input) const
        {
            if (input.size() != mean_.size())
            {
                throw std::invalid_argument("Input dimension does not match normalizer dimension");
            }

            std::vector<Scalar> output(input.size());
            for (size_t i = 0; i < input.size(); i++)
            {
                output[i] = input[i] * std_[i] + mean_[i];
            }

            return output;
        }

        void Normalizer::update(const std::vector<Scalar> &data)
        {
            if (data.size() != mean_.size())
            {
                throw std::invalid_argument("Data dimension does not match normalizer dimension");
            }

            for (size_t i = 0; i < data.size(); i++)
            {
                const Scalar delta = data[i] - mean_[i];
                count_[i]++;
                // Online mean update
                mean_[i] += delta / count_[i];
                // Online variance update (Welford's algorithm)
                if (count_[i] > 1)
                {
                    const Scalar delta2 = data[i] - mean_[i];
                    std_[i] = std::sqrt((std_[i] * std_[i] * (count_[i] - 1) + delta * delta2) / count_[i]);
                }
            }
        }

        const std::vector<Scalar> &Normalizer::getMean() const
        {
            return mean_;
        }

        const std::vector<Scalar> &Normalizer::getStd() const
        {
            return std_;
        }

        // FileSystem implementation
        bool FileSystem::fileExists(const std::string &path)
        {
            return std::filesystem::exists(path);
        }

        bool FileSystem::createDirectory(const std::string &path)
        {
            return std::filesystem::create_directories(path);
        }

        std::vector<std::string> FileSystem::listFiles(const std::string &path)
        {
            std::vector<std::string> files;

            if (!std::filesystem::exists(path))
            {
                return files;
            }

            for (const auto &entry : std::filesystem::directory_iterator(path))
            {
                files.push_back(entry.path().filename().string());
            }

            return files;
        }

    } // namespace utils
} // namespace td3learn
