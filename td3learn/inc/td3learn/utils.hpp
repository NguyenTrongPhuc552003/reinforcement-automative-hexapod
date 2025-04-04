#ifndef TD3LEARN_UTILS_HPP
#define TD3LEARN_UTILS_HPP

#include <string>
#include <vector>
#include <memory>
#include "td3learn/types.hpp"

namespace td3learn
{
    namespace utils
    {

        /**
         * @brief Logger class for consistent logging
         */
        class Logger
        {
        public:
            /**
             * @brief Logging levels
             */
            enum class Level
            {
                DEBUG,
                INFO,
                WARNING,
                ERROR,
                FATAL
            };

            /**
             * @brief Initialize logger
             * @param name Logger name
             * @param level Minimum log level
             * @param log_to_file Whether to log to file
             * @param file_path Log file path
             */
            static void init(
                const std::string &name = "td3learn",
                Level level = Level::INFO,
                bool log_to_file = false,
                const std::string &file_path = "td3learn.log");

            /**
             * @brief Set log level
             * @param level New log level
             */
            static void setLevel(Level level);

            /**
             * @brief Log a message
             * @param level Log level
             * @param message Message
             */
            static void log(Level level, const std::string &message);

            /**
             * @brief Log debug message
             * @param message Message
             */
            static void debug(const std::string &message);

            /**
             * @brief Log info message
             * @param message Message
             */
            static void info(const std::string &message);

            /**
             * @brief Log warning message
             * @param message Message
             */
            static void warning(const std::string &message);

            /**
             * @brief Log error message
             * @param message Message
             */
            static void error(const std::string &message);

            /**
             * @brief Log fatal message
             * @param message Message
             */
            static void fatal(const std::string &message);
        };

        /**
         * @brief Timer utility for performance measurement
         */
        class Timer
        {
        public:
            /**
             * @brief Start the timer
             */
            static void start();

            /**
             * @brief Stop the timer
             * @return Elapsed time in milliseconds
             */
            static double stop();

            /**
             * @brief Get elapsed time without stopping
             * @return Elapsed time in milliseconds
             */
            static double elapsed();
        };

        /**
         * @brief Data normalization utilities
         */
        class Normalizer
        {
        public:
            /**
             * @brief Create a normalizer
             * @param mean Mean values
             * @param std Standard deviation values
             */
            Normalizer(const std::vector<Scalar> &mean, const std::vector<Scalar> &std);

            /**
             * @brief Create an identity normalizer
             * @param dimension Input dimension
             */
            explicit Normalizer(size_t dimension);

            /**
             * @brief Normalize a vector
             * @param input Input vector
             * @return Normalized vector
             */
            std::vector<Scalar> normalize(const std::vector<Scalar> &input) const;

            /**
             * @brief Denormalize a vector
             * @param input Normalized vector
             * @return Denormalized vector
             */
            std::vector<Scalar> denormalize(const std::vector<Scalar> &input) const;

            /**
             * @brief Update statistics with new data
             * @param data New data point
             */
            void update(const std::vector<Scalar> &data);

            /**
             * @brief Get current mean values
             * @return Mean vector
             */
            const std::vector<Scalar> &getMean() const;

            /**
             * @brief Get current standard deviation values
             * @return Standard deviation vector
             */
            const std::vector<Scalar> &getStd() const;

        private:
            std::vector<Scalar> mean_;
            std::vector<Scalar> std_;
            std::vector<size_t> count_;
        };

        /**
         * @brief File system utilities
         */
        class FileSystem
        {
        public:
            /**
             * @brief Check if file exists
             * @param path File path
             * @return True if file exists
             */
            static bool fileExists(const std::string &path);

            /**
             * @brief Create directory
             * @param path Directory path
             * @return True if successful
             */
            static bool createDirectory(const std::string &path);

            /**
             * @brief List files in directory
             * @param path Directory path
             * @return Vector of file names
             */
            static std::vector<std::string> listFiles(const std::string &path);
        };

    } // namespace utils
} // namespace td3learn

#endif // TD3LEARN_UTILS_HPP
