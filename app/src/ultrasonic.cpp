#include <fstream>
#include <chrono>
#include <thread>
#include <system_error>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include "ultrasonic.hpp"

namespace ultrasonic
{

    //==============================================================================
    // Error Category Implementation
    //==============================================================================

    namespace
    {
        class UltrasonicCategory : public std::error_category
        {
        public:
            const char *name() const noexcept override { return "ultrasonic"; }

            std::string message(int ev) const override
            {
                switch (static_cast<Error>(ev))
                {
                case Error::SUCCESS:
                    return "Success";
                case Error::NOT_INITIALIZED:
                    return "Sensor not initialized";
                case Error::GPIO_EXPORT_FAILED:
                    return "Failed to export GPIO";
                case Error::GPIO_DIRECTION_FAILED:
                    return "Failed to set GPIO direction";
                case Error::GPIO_WRITE_FAILED:
                    return "Failed to write GPIO value";
                case Error::GPIO_READ_FAILED:
                    return "Failed to read GPIO value";
                case Error::MEASUREMENT_TIMEOUT:
                    return "Echo timeout occurred";
                case Error::INVALID_MEASUREMENT:
                    return "Invalid measurement value";
                case Error::SYSTEM_ERROR:
                    return "System error occurred";
                default:
                    return "Unknown error";
                }
            }
        };
    }

    std::error_category &ultrasonic_category() noexcept
    {
        static UltrasonicCategory c;
        return c;
    }

    //==============================================================================
    // Implementation Class
    //==============================================================================

    class Ultrasonic::Impl
    {
    public:
        explicit Impl(uint32_t trigger_gpio, uint32_t echo_gpio)
            : m_triggerGpio(trigger_gpio), m_echoGpio(echo_gpio), m_initialized(false), m_lastError("No error"), m_lastMeasurementTime(std::chrono::steady_clock::now())
        {
        }

        std::error_code initialize()
        {
            try
            {
                // Export GPIOs if not already exported
                if (!exportGpio(m_triggerGpio))
                    return make_error_code(Error::GPIO_EXPORT_FAILED);

                if (!exportGpio(m_echoGpio))
                    return make_error_code(Error::GPIO_EXPORT_FAILED);

                // Configure trigger as output
                if (!setDirection(m_triggerGpio, "out"))
                    return make_error_code(Error::GPIO_DIRECTION_FAILED);

                // Configure echo as input
                if (!setDirection(m_echoGpio, "in"))
                    return make_error_code(Error::GPIO_DIRECTION_FAILED);

                // Set initial trigger state to low
                if (!setValue(m_triggerGpio, 0))
                    return make_error_code(Error::GPIO_WRITE_FAILED);

                m_initialized = true;
                return {};
            }
            catch (const std::system_error &e)
            {
                m_lastError = e.what();
                return e.code();
            }
        }

        std::pair<float, std::error_code> getDistance()
        {
            if (!m_initialized)
                return {-1.0f, make_error_code(Error::NOT_INITIALIZED)};

            // Rate limiting - ensure minimum interval between measurements
            auto now = std::chrono::steady_clock::now();
            auto timeSinceLastMeasurement = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                now - m_lastMeasurementTime)
                                                .count();

            if (timeSinceLastMeasurement < Config::MIN_MEASUREMENT_INTERVAL_MS)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(
                    Config::MIN_MEASUREMENT_INTERVAL_MS - timeSinceLastMeasurement));
            }

            try
            {
                // Send trigger pulse
                if (!setValue(m_triggerGpio, 1))
                    return {-1.0f, make_error_code(Error::GPIO_WRITE_FAILED)};

                std::this_thread::sleep_for(std::chrono::microseconds(Config::TRIGGER_PULSE_US));

                if (!setValue(m_triggerGpio, 0))
                    return {-1.0f, make_error_code(Error::GPIO_WRITE_FAILED)};

                // Wait for echo to start
                auto startTime = std::chrono::steady_clock::now();
                while (getValue(m_echoGpio) == 0)
                {
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - startTime)
                            .count() > Config::ECHO_TIMEOUT_MS)
                    {
                        return {-1.0f, make_error_code(Error::MEASUREMENT_TIMEOUT)};
                    }
                }

                // Get echo start time
                auto echoStart = std::chrono::steady_clock::now();

                // Wait for echo to end
                while (getValue(m_echoGpio) == 1)
                {
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - startTime)
                            .count() > Config::ECHO_TIMEOUT_MS)
                    {
                        return {-1.0f, make_error_code(Error::MEASUREMENT_TIMEOUT)};
                    }
                }

                // Calculate duration and distance
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                                    std::chrono::steady_clock::now() - echoStart)
                                    .count();

                float distance = duration * Config::SPEED_OF_SOUND / 2.0f;

                // Validate measurement
                if (distance < Config::MIN_DISTANCE_CM || distance > Config::MAX_DISTANCE_CM)
                {
                    return {-1.0f, make_error_code(Error::INVALID_MEASUREMENT)};
                }

                m_lastMeasurementTime = now;
                return {distance, {}};
            }
            catch (const std::system_error &e)
            {
                m_lastError = e.what();
                return {-1.0f, e.code()};
            }
        }

        bool isReady() const noexcept
        {
            return m_initialized;
        }

        std::string_view getLastError() const noexcept
        {
            return m_lastError;
        }

    private:
        bool exportGpio(uint32_t gpio)
        {
            std::ofstream exportFile("/sys/class/gpio/export");
            if (!exportFile)
            {
                m_lastError = "Failed to open export file: " + std::string(strerror(errno));
                return false;
            }
            exportFile << gpio;
            return true;
        }

        bool setDirection(uint32_t gpio, const std::string &direction)
        {
            std::string path = "/sys/class/gpio/gpio" + std::to_string(gpio) + "/direction";
            std::ofstream directionFile(path);
            if (!directionFile)
            {
                m_lastError = "Failed to set direction for GPIO " +
                              std::to_string(gpio) + ": " + std::string(strerror(errno));
                return false;
            }
            directionFile << direction;
            return true;
        }

        bool setValue(uint32_t gpio, int value)
        {
            std::string path = "/sys/class/gpio/gpio" + std::to_string(gpio) + "/value";
            std::ofstream valueFile(path);
            if (!valueFile)
            {
                m_lastError = "Failed to set value for GPIO " +
                              std::to_string(gpio) + ": " + std::string(strerror(errno));
                return false;
            }
            valueFile << value;
            return true;
        }

        int getValue(uint32_t gpio)
        {
            std::string path = "/sys/class/gpio/gpio" + std::to_string(gpio) + "/value";
            std::ifstream valueFile(path);
            if (!valueFile)
            {
                m_lastError = "Failed to read value from GPIO " +
                              std::to_string(gpio) + ": " + std::string(strerror(errno));
                throw std::system_error(errno, std::system_category());
            }
            char value;
            valueFile >> value;
            return value - '0';
        }

        // Member variables
        const uint32_t m_triggerGpio;
        const uint32_t m_echoGpio;
        bool m_initialized;
        std::string m_lastError;
        std::chrono::steady_clock::time_point m_lastMeasurementTime;
    };

    //==============================================================================
    // Public Interface Implementation
    //==============================================================================

    Ultrasonic::Ultrasonic(uint32_t trigger_gpio, uint32_t echo_gpio)
        : m_pImpl(std::make_unique<Impl>(trigger_gpio, echo_gpio))
    {
    }

    Ultrasonic::~Ultrasonic() = default;
    Ultrasonic::Ultrasonic(Ultrasonic &&) noexcept = default;
    Ultrasonic &Ultrasonic::operator=(Ultrasonic &&) noexcept = default;

    std::error_code Ultrasonic::initialize()
    {
        return m_pImpl->initialize();
    }

    std::pair<float, std::error_code> Ultrasonic::getDistance()
    {
        return m_pImpl->getDistance();
    }

    bool Ultrasonic::isReady() const noexcept
    {
        return m_pImpl->isReady();
    }

    std::string_view Ultrasonic::getLastError() const noexcept
    {
        return m_pImpl->getLastError();
    }

} // namespace ultrasonic
