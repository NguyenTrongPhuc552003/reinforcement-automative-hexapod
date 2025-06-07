#include <string>
#include <fstream>
#include <thread>
#include <chrono>
#include <iostream>
#include <cmath>
#include "ultrasonic.hpp"

namespace ultrasonic
{

    class UltrasonicImpl
    {
    public:
        UltrasonicImpl(const Ultrasonic::PinConfig &config)
            : m_triggerPin(config.trigger_pin), m_echoPin(config.echo_pin), m_initialized(false), m_lastError("")
        {
        }

        bool init()
        {
            try
            {
                // Export GPIO pins
                exportGPIO(m_triggerPin);
                exportGPIO(m_echoPin);

                // Configure directions
                setDirection(m_triggerPin, "out");
                setDirection(m_echoPin, "in");

                // Set initial trigger state
                writeValue(m_triggerPin, "0");

                m_initialized = true;
                return true;
            }
            catch (const std::exception &e)
            {
                m_lastError = std::string("Initialization failed: ") + e.what();
                return false;
            }
        }

        float getDistance()
        {
            if (!m_initialized)
            {
                m_lastError = "Sensor not initialized";
                return -1;
            }

            try
            {
                // Send trigger pulse
                writeValue(m_triggerPin, "1");
                std::this_thread::sleep_for(std::chrono::microseconds(10));
                writeValue(m_triggerPin, "0");

                // Wait for echo start
                auto start = std::chrono::high_resolution_clock::now();
                while (readValue(m_echoPin) == "0")
                {
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::high_resolution_clock::now() - start)
                            .count() > 100)
                    {
                        m_lastError = "Echo timeout";
                        return -1;
                    }
                }

                // Measure echo duration
                start = std::chrono::high_resolution_clock::now();
                while (readValue(m_echoPin) == "1")
                {
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::high_resolution_clock::now() - start)
                            .count() > 100)
                    {
                        m_lastError = "Echo timeout";
                        return -1;
                    }
                }
                auto end = std::chrono::high_resolution_clock::now();

                // Calculate distance
                float duration = std::chrono::duration<float>(end - start).count();
                return (duration * 34300.0f) / 2.0f; // Speed of sound / 2 for return trip
            }
            catch (const std::exception &e)
            {
                m_lastError = std::string("Measurement failed: ") + e.what();
                return -1;
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
        void exportGPIO(int pin)
        {
            std::ofstream export_file("/sys/class/gpio/export");
            if (!export_file)
            {
                throw std::runtime_error("Failed to open GPIO export file");
            }
            export_file << pin;
        }

        void setDirection(int pin, const std::string &direction)
        {
            std::string path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
            std::ofstream dir_file(path);
            if (!dir_file)
            {
                throw std::runtime_error("Failed to set GPIO direction");
            }
            dir_file << direction;
        }

        void writeValue(int pin, const std::string &value)
        {
            std::string path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
            std::ofstream value_file(path);
            if (!value_file)
            {
                throw std::runtime_error("Failed to write GPIO value");
            }
            value_file << value;
        }

        std::string readValue(int pin)
        {
            std::string path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
            std::ifstream value_file(path);
            if (!value_file)
            {
                throw std::runtime_error("Failed to read GPIO value");
            }
            std::string value;
            value_file >> value;
            return value;
        }

        int m_triggerPin;
        int m_echoPin;
        bool m_initialized;
        std::string m_lastError;
    };

    // Implement Ultrasonic class methods
    Ultrasonic::Ultrasonic(const PinConfig &config)
        : pImpl(std::make_unique<UltrasonicImpl>(config))
    {
    }

    Ultrasonic::~Ultrasonic() = default;
    Ultrasonic::Ultrasonic(Ultrasonic &&) noexcept = default;
    Ultrasonic &Ultrasonic::operator=(Ultrasonic &&) noexcept = default;

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
