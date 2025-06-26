#ifndef COMMON_HPP
#define COMMON_HPP

#include <string>
#include <chrono>
#include <termios.h>
#include <memory>
#include <functional>
#include <atomic>
#include <vector>

/**
 * @brief Common utilities and API functions used across the hexapod system
 */
namespace common
{

    //==============================================================================
    // Time Utilities
    //==============================================================================

    /**
     * @brief Get current time in seconds with high precision
     * @return double Current time in seconds since epoch
     */
    double getCurrentTime();

    /**
     * @brief Get current time in milliseconds
     * @return unsigned long Current time in milliseconds
     */
    unsigned long getCurrentTimeMs();

    /**
     * @brief Sleep for specified milliseconds
     * @param milliseconds Time to sleep
     */
    void sleepMs(unsigned int milliseconds);

    /**
     * @brief Sleep for specified microseconds
     * @param microseconds Time to sleep
     */
    void sleepUs(unsigned int microseconds);

    //==============================================================================
    // Terminal Management
    //==============================================================================

    /**
     * @brief Terminal configuration manager
     */
    class TerminalManager
    {
    public:
        /**
         * @brief Setup terminal for non-blocking input
         * @return true if setup successful
         */
        static bool setupNonBlocking();

        /**
         * @brief Setup terminal for immediate input (no echo, no buffering)
         * @return true if setup successful
         */
        static bool setupImmediate();

        /**
         * @brief Restore terminal to original state
         */
        static void restore();

        /**
         * @brief Check if terminal has been modified
         * @return true if terminal settings have been changed
         */
        static bool isModified();

        /**
         * @brief Read a single character without blocking
         * @param[out] ch Character read
         * @return true if character was read
         */
        static bool readChar(char &ch);

    private:
        static struct termios s_originalTermios;
        static bool s_isModified;
    };

    //==============================================================================
    // Signal Handling
    //==============================================================================

    /**
     * @brief Common signal handler type
     */
    using SignalHandler = std::function<void(int)>;

    /**
     * @brief Signal management utilities
     */
    class SignalManager
    {
    public:
        /**
         * @brief Setup common signal handlers for graceful shutdown
         * @param handler Custom handler function
         * @param running Atomic flag to set to false on signal
         */
        static void setupGracefulShutdown(std::atomic<bool> &running,
                                          SignalHandler handler = nullptr);

        /**
         * @brief Default signal handler that sets running flag to false
         * @param signal Signal number
         */
        static void defaultHandler(int signal);

    private:
        static std::atomic<bool> *s_runningFlag;
        static SignalHandler s_customHandler;
    };

    //==============================================================================
    // Performance Monitoring
    //==============================================================================

    /**
     * @brief Performance metrics tracking
     */
    class PerformanceMonitor
    {
    public:
        /**
         * @brief Construct a new Performance Monitor
         */
        PerformanceMonitor();

        /**
         * @brief Start timing a frame/operation
         */
        void startFrame();

        /**
         * @brief End timing and record metrics
         */
        void endFrame();

        /**
         * @brief Update metrics with a specific duration
         * @param durationMs Duration in milliseconds
         */
        void recordFrame(double durationMs);

        /**
         * @brief Get average frame time in milliseconds
         * @return double Average frame time
         */
        double getAverageFrameTime() const;

        /**
         * @brief Get maximum frame time in milliseconds
         * @return double Maximum frame time
         */
        double getMaxFrameTime() const;

        /**
         * @brief Get current FPS
         * @return double Frames per second
         */
        double getFPS() const;

        /**
         * @brief Get total number of frames processed
         * @return unsigned long Frame count
         */
        unsigned long getFrameCount() const;

        /**
         * @brief Reset all metrics
         */
        void reset();

        /**
         * @brief Print performance report
         * @param prefix Optional prefix for output
         */
        void printReport(const std::string &prefix = "") const;

    private:
        std::chrono::high_resolution_clock::time_point m_frameStart;
        unsigned long m_frameCount;
        double m_totalFrameTime;
        double m_maxFrameTime;
        bool m_frameInProgress;
    };

    //==============================================================================
    // Progress Display
    //==============================================================================

    /**
     * @brief Progress bar utilities
     */
    class ProgressBar
    {
    public:
        /**
         * @brief Display a progress bar
         * @param current Current progress value
         * @param total Total progress value
         * @param width Width of the progress bar in characters
         * @param label Optional label to display
         */
        static void display(int current, int total, int width = 40,
                            const std::string &label = "");

        /**
         * @brief Display a progress bar with percentage
         * @param percentage Percentage complete (0.0-1.0)
         * @param width Width of the progress bar in characters
         * @param label Optional label to display
         */
        static void displayPercent(double percentage, int width = 40,
                                   const std::string &label = "");

        /**
         * @brief Clear the current progress line
         */
        static void clear();
    };

    //==============================================================================
    // Error Handling
    //==============================================================================

    /**
     * @brief Common error reporting utilities
     */
    class ErrorReporter
    {
    public:
        /**
         * @brief Report an error with context
         * @param component Component name where error occurred
         * @param operation Operation that failed
         * @param details Additional error details
         */
        static void reportError(const std::string &component,
                                const std::string &operation,
                                const std::string &details);

        /**
         * @brief Report a warning with context
         * @param component Component name
         * @param message Warning message
         */
        static void reportWarning(const std::string &component,
                                  const std::string &message);

        /**
         * @brief Report informational message
         * @param component Component name
         * @param message Information message
         */
        static void reportInfo(const std::string &component,
                               const std::string &message);
    };

    //==============================================================================
    // Validation Utilities
    //==============================================================================

    /**
     * @brief Common validation functions
     */
    class Validator
    {
    public:
        /**
         * @brief Validate angle is within range
         * @param angle Angle to validate
         * @param min Minimum allowed angle
         * @param max Maximum allowed angle
         * @return true if angle is valid
         */
        static bool validateAngle(double angle, double min, double max);

        /**
         * @brief Validate leg number
         * @param legNum Leg number to validate
         * @param maxLegs Maximum number of legs
         * @return true if leg number is valid
         */
        static bool validateLegNumber(int legNum, int maxLegs = 6);

        /**
         * @brief Validate speed factor
         * @param speed Speed factor to validate (should be 0.0-1.0)
         * @return true if speed is valid
         */
        static bool validateSpeed(double speed);

        /**
         * @brief Validate direction angle
         * @param direction Direction in degrees (should be 0.0-360.0)
         * @return true if direction is valid
         */
        static bool validateDirection(double direction);

        /**
         * @brief Clamp value to range
         * @param value Value to clamp
         * @param min Minimum value
         * @param max Maximum value
         * @return Clamped value
         */
        template <typename T>
        static T clamp(T value, T min, T max)
        {
            return (value < min) ? min : (value > max) ? max
                                                       : value;
        }
    };

    //==============================================================================
    // String Utilities
    //==============================================================================

    /**
     * @brief String utility functions
     */
    class StringUtils
    {
    public:
        /**
         * @brief Format a number with specified precision
         * @param value Number to format
         * @param precision Number of decimal places
         * @return Formatted string
         */
        static std::string formatNumber(double value, int precision = 2);

        /**
         * @brief Format time duration in human-readable format
         * @param seconds Duration in seconds
         * @return Formatted string (e.g., "1m 23.5s")
         */
        static std::string formatDuration(double seconds);

        /**
         * @brief Create a padded string with specified width
         * @param text Text to pad
         * @param width Total width
         * @param fillChar Character to use for padding
         * @return Padded string
         */
        static std::string padString(const std::string &text, int width, char fillChar = ' ');

        /**
         * @brief Convert string to lowercase
         * @param str String to convert
         * @return Lowercase string
         */
        static std::string toLower(const std::string &str);

        /**
         * @brief Convert string to uppercase
         * @param str String to convert
         * @return Uppercase string
         */
        static std::string toUpper(const std::string &str);
    };

    //==============================================================================
    // Math Utilities
    //==============================================================================

    /**
     * @brief Mathematical utility functions
     */
    class MathUtils
    {
    public:
        /**
         * @brief Convert degrees to radians
         * @param degrees Angle in degrees
         * @return Angle in radians
         */
        static double degToRad(double degrees);

        /**
         * @brief Convert radians to degrees
         * @param radians Angle in radians
         * @return Angle in degrees
         */
        static double radToDeg(double radians);

        /**
         * @brief Linear interpolation between two values
         * @param a Start value
         * @param b End value
         * @param t Interpolation factor (0.0-1.0)
         * @return Interpolated value
         */
        static double lerp(double a, double b, double t);

        /**
         * @brief Normalize angle to 0-360 degree range
         * @param angle Angle in degrees
         * @return Normalized angle
         */
        static double normalizeAngle(double angle);

        /**
         * @brief Calculate moving average
         * @param values Vector of values
         * @param windowSize Size of averaging window
         * @return Moving average
         */
        static std::vector<double> movingAverage(const std::vector<double> &values,
                                                 int windowSize);
    };

    //==============================================================================
    // Constants
    //==============================================================================

    namespace Constants
    {
        constexpr double PI = 3.14159265358979323846;
        constexpr double TWO_PI = 2.0 * PI;
        constexpr double HALF_PI = PI / 2.0;
        constexpr double DEG_TO_RAD = PI / 180.0;
        constexpr double RAD_TO_DEG = 180.0 / PI;

        // Common timing values
        constexpr int DEFAULT_UPDATE_RATE_HZ = 20;
        constexpr int DEFAULT_DELAY_MS = 50;
        constexpr int DEFAULT_RETRY_COUNT = 3;

        // Common validation limits
        constexpr double MIN_SPEED = 0.0;
        constexpr double MAX_SPEED = 1.0;
        constexpr double MIN_DIRECTION = 0.0;
        constexpr double MAX_DIRECTION = 360.0;
    }

} // namespace common

#endif // COMMON_HPP
