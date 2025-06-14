#ifndef ULTRASONIC_HPP
#define ULTRASONIC_HPP

#include <memory>
#include <string>
#include <gpiod.h>

namespace ultrasonic
{

    /**
     * @brief Default GPIO pin configuration for HC-SR04 sensor on BeagleBone AI
     *
     * Using gpiod chip and line numbering:
     * - TRIGGER: gpio1[19] (P9_16) -> chip "gpiochip1", line 19
     * - ECHO: gpio0[20] (P9_41) -> chip "gpiochip0", line 20
     */
    struct DefaultPins
    {
        static constexpr const char *TRIGGER_CHIP = "gpiochip1";
        static constexpr unsigned int TRIGGER_LINE = 19;
        static constexpr const char *ECHO_CHIP = "gpiochip0";
        static constexpr unsigned int ECHO_LINE = 20;
    };

    /**
     * @brief Configuration structure for ultrasonic sensor
     */
    struct PinConfig
    {
        std::string trigger_chip = DefaultPins::TRIGGER_CHIP;
        unsigned int trigger_line = DefaultPins::TRIGGER_LINE;
        std::string echo_chip = DefaultPins::ECHO_CHIP;
        unsigned int echo_line = DefaultPins::ECHO_LINE;
    };

    /**
     * @brief HC-SR04 Ultrasonic sensor class using libgpiod
     */
    class Ultrasonic
    {
    public:
        /**
         * @brief Construct a new Ultrasonic object
         * @param config Pin configuration
         */
        explicit Ultrasonic(const PinConfig &config = PinConfig());

        /**
         * @brief Destroy the Ultrasonic object
         */
        ~Ultrasonic();

        /**
         * @brief Initialize the sensor
         * @return true if initialization successful
         */
        bool init();

        /**
         * @brief Get distance measurement
         * @return float Distance in centimeters or -1 if measurement failed
         */
        float getDistance();

        /**
         * @brief Check if sensor is ready for measurement
         * @return true if sensor is initialized and ready
         */
        bool isReady() const;

        /**
         * @brief Get last error message
         * @return std::string Last error message
         */
        std::string getLastError() const;

    private:
        class Impl;
        std::unique_ptr<Impl> pImpl;
    };

} // namespace ultrasonic

#endif // ULTRASONIC_HPP
