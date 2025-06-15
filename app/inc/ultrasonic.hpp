#ifndef ULTRASONIC_HPP
#define ULTRASONIC_HPP

#include <memory>
#include <string>
#include <system_error>

/**
 * @brief Ultrasonic sensor system
 *
 * This namespace contains classes and functions for interfacing with
 * HC-SR04 ultrasonic distance sensor using sysfs GPIO interface.
 */
namespace ultrasonic
{

    //==============================================================================
    // Configuration Constants
    //==============================================================================

    /**
     * @brief Configuration for HC-SR04 ultrasonic sensor and timing parameters
     */
    struct Config
    {
        // GPIO pin configuration
        static constexpr uint32_t DEFAULT_TRIGGER_GPIO = 51; // P9_16 (gpio1[19])
        static constexpr uint32_t DEFAULT_ECHO_GPIO = 20;    // P9_41 (gpio0[20])

        // Timing parameters
        static constexpr uint32_t TRIGGER_PULSE_US = 10;            // Trigger pulse width in microseconds
        static constexpr uint32_t ECHO_TIMEOUT_MS = 100;            // Echo timeout in milliseconds
        static constexpr uint32_t MIN_MEASUREMENT_INTERVAL_MS = 60; // Minimum time between measurements

        // Operational parameters
        static constexpr float MIN_DISTANCE_CM = 2.0f;   // Minimum valid distance
        static constexpr float MAX_DISTANCE_CM = 400.0f; // Maximum valid distance
        static constexpr float SPEED_OF_SOUND = 0.0343f; // Speed of sound in cm/μs at room temp
    };

    //==============================================================================
    // Error Handling
    //==============================================================================

    /**
     * @brief Error codes for ultrasonic sensor operations
     */
    enum class Error
    {
        SUCCESS = 0,           ///< Operation completed successfully
        NOT_INITIALIZED,       ///< Sensor not initialized
        GPIO_EXPORT_FAILED,    ///< Failed to export GPIO
        GPIO_DIRECTION_FAILED, ///< Failed to set GPIO direction
        GPIO_WRITE_FAILED,     ///< Failed to write GPIO value
        GPIO_READ_FAILED,      ///< Failed to read GPIO value
        MEASUREMENT_TIMEOUT,   ///< Echo timeout occurred
        INVALID_MEASUREMENT,   ///< Invalid measurement value
        SYSTEM_ERROR           ///< System-level error
    };

    //==============================================================================
    // Main Ultrasonic Class
    //==============================================================================

    /**
     * @brief Class for controlling HC-SR04 ultrasonic sensor through sysfs
     *
     * Provides high-level interface for distance measurements using the
     * HC-SR04 ultrasonic sensor with proper error handling and timing.
     */
    class Ultrasonic
    {
    public:
        /**
         * @brief Construct a new Ultrasonic sensor object
         * @param trigger_gpio GPIO number for trigger pin
         * @param echo_gpio GPIO number for echo pin
         */
        explicit Ultrasonic(uint32_t trigger_gpio = Config::DEFAULT_TRIGGER_GPIO,
                            uint32_t echo_gpio = Config::DEFAULT_ECHO_GPIO);

        /**
         * @brief Destroy the Ultrasonic object and cleanup GPIO
         */
        ~Ultrasonic();

        // Delete copy operations
        Ultrasonic(const Ultrasonic &) = delete;
        Ultrasonic &operator=(const Ultrasonic &) = delete;

        // Allow move operations
        Ultrasonic(Ultrasonic &&) noexcept;
        Ultrasonic &operator=(Ultrasonic &&) noexcept;

        /**
         * @brief Initialize GPIO pins and prepare sensor
         * @return std::error_code Empty on success, error code otherwise
         */
        std::error_code initialize();

        /**
         * @brief Get distance measurement
         * @return std::pair<float, std::error_code> Distance in cm and error code
         */
        std::pair<float, std::error_code> getDistance();

        /**
         * @brief Check if sensor is ready for measurement
         * @return bool True if ready
         */
        bool isReady() const noexcept;

        /**
         * @brief Get last error message
         * @return std::string_view Error message
         */
        std::string_view getLastError() const noexcept;

    private:
        class Impl;
        std::unique_ptr<Impl> m_pImpl;
    };

    // Error category for ultrasonic sensor errors
    std::error_category &ultrasonic_category() noexcept;

    // Make error codes from Error enum
    inline std::error_code make_error_code(Error e) noexcept
    {
        return {static_cast<int>(e), ultrasonic_category()};
    }

} // namespace ultrasonic

// Register Error with std::error_code system
namespace std
{
    template <>
    struct is_error_code_enum<ultrasonic::Error> : true_type
    {
    };
}

#endif // ULTRASONIC_HPP
