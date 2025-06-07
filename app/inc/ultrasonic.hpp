#ifndef ULTRASONIC_HPP
#define ULTRASONIC_HPP

#include <memory>
#include <vector>
#include <chrono>

namespace ultrasonic
{

    class UltrasonicImpl;

    /**
     * @brief Default GPIO pin numbers for HC-SR04 sensor
     *
     * These GPIO numbers correspond to the BeagleBone AI pin mappings.
     * For example:
     * - TRIGGER_PIN (51) corresponds to P9_16
     * - ECHO_PIN (20) corresponds to P9_41
     */
    struct DefaultPins
    {
        static constexpr int TRIGGER_PIN = 51;
        static constexpr int ECHO_PIN = 20;
    };

    /**
     * @brief HC-SR04 ultrasonic sensor interface
     *
     * Manages a HC-SR04 ultrasonic sensor connected to BeagleBone AI GPIO pins
     */
    class Ultrasonic
    {
    public:
        /**
         * @brief GPIO pin configuration for HC-SR04
         */
        struct PinConfig
        {
            int trigger_pin; ///< GPIO pin number for TRIG
            int echo_pin;    ///< GPIO pin number for ECHO
        };

        /**
         * @brief Construct ultrasonic sensor interface
         *
         * @param config Pin configuration for sensor
         */
        explicit Ultrasonic(const PinConfig &config);
        ~Ultrasonic();

        // Delete copy operations
        Ultrasonic(const Ultrasonic &) = delete;
        Ultrasonic &operator=(const Ultrasonic &) = delete;

        // Allow move operations
        Ultrasonic(Ultrasonic &&) noexcept;
        Ultrasonic &operator=(Ultrasonic &&) noexcept;

        /**
         * @brief Initialize sensor interface
         *
         * @return true if initialization successful
         */
        bool init();

        /**
         * @brief Get distance measurement
         *
         * @return float Distance in centimeters, -1 if error
         */
        float getDistance();

        /**
         * @brief Check if sensor is ready
         *
         * @return true if sensor initialized and working
         */
        bool isReady() const;

        /**
         * @brief Get error message for last error
         *
         * @return std::string Error message
         */
        std::string getLastError() const;

    private:
        std::unique_ptr<UltrasonicImpl> pImpl;
    };

} // namespace ultrasonic

#endif // ULTRASONIC_HPP
