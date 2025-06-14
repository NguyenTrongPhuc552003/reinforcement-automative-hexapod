#include <chrono>
#include <thread>
#include <system_error>
#include "ultrasonic.hpp"

namespace ultrasonic
{

    class Ultrasonic::Impl
    {
    public:
        explicit Impl(const PinConfig &config)
            : m_config(config), m_initialized(false), m_lastError("")
        {
            // Chips and lines will be initialized in init()
        }

        bool init()
        {
            try
            {
                // Open GPIO chips
                m_triggerChip = gpiod::chip(m_config.trigger_chip);
                m_echoChip = gpiod::chip(m_config.echo_chip);

                // Get GPIO lines
                m_triggerLine = m_triggerChip.get_line(m_config.trigger_line);
                m_echoLine = m_echoChip.get_line(m_config.echo_line);

                // Configure trigger as output
                m_triggerLine.request(
                    {
                        "hexapod-ultrasonic",
                        gpiod::line_request::DIRECTION_OUTPUT,
                        0,
                    });

                // Configure echo as input
                m_echoLine.request(
                    {
                        "hexapod-ultrasonic",
                        gpiod::line_request::DIRECTION_INPUT,
                        0,
                    });

                m_initialized = true;
                return true;
            }
            catch (const std::system_error &e)
            {
                m_lastError = "GPIO initialization failed: " + std::string(e.what());
                m_initialized = false;
                return false;
            }
        }

        float getDistance()
        {
            if (!m_initialized)
            {
                m_lastError = "Sensor not initialized";
                return -1.0f;
            }

            try
            {
                // Send trigger pulse
                m_triggerLine.set_value(1);
                std::this_thread::sleep_for(std::chrono::microseconds(10));
                m_triggerLine.set_value(0);

                // Wait for echo to start (timeout after 100ms)
                auto startTime = std::chrono::steady_clock::now();
                while (m_echoLine.get_value() == 0)
                {
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - startTime)
                            .count() >
                        100)
                    {
                        m_lastError = "Echo start timeout";
                        return -1.0f;
                    }
                }

                // Get echo start time
                auto echoStart = std::chrono::steady_clock::now();

                // Wait for echo to end (timeout after 100ms)
                startTime = std::chrono::steady_clock::now();
                while (m_echoLine.get_value() == 1)
                {
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - startTime)
                            .count() >
                        100)
                    {
                        m_lastError = "Echo end timeout";
                        return -1.0f;
                    }
                }

                // Calculate duration
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                                    std::chrono::steady_clock::now() - echoStart)
                                    .count();

                // Speed of sound = 343 m/s at room temperature
                // Distance = (speed * time) / 2 (round trip)
                // 343 * 100 = 34300 cm/s
                // 34300 / 1000000 = 0.0343 cm/µs
                return static_cast<float>(duration * 0.0343f / 2.0f);
            }
            catch (const std::system_error &e)
            {
                m_lastError = "Measurement failed: " + std::string(e.what());
                return -1.0f;
            }
        }

        bool isReady() const
        {
            return m_initialized;
        }

        std::string getLastError() const
        {
            return m_lastError;
        }

    private:
        PinConfig m_config;
        bool m_initialized;
        std::string m_lastError;

        gpiod::chip m_triggerChip;
        gpiod::chip m_echoChip;
        gpiod::line m_triggerLine;
        gpiod::line m_echoLine;
    };

    // Public interface implementation
    Ultrasonic::Ultrasonic(const PinConfig &config)
        : pImpl(std::make_unique<Impl>(config))
    {
    }

    Ultrasonic::~Ultrasonic() = default;

    bool Ultrasonic::init()
    {
        return pImpl->init();
    }

    float Ultrasonic::getDistance()
    {
        return pImpl->getDistance();
    }

    bool Ultrasonic::isReady() const
    {
        return pImpl->isReady();
    }

    std::string Ultrasonic::getLastError() const
    {
        return pImpl->getLastError();
    }

} // namespace ultrasonic
