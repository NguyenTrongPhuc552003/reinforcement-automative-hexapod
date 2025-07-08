/*
 * Hexapod Project - A Reinforcement Learning-based Autonomous Hexapod
 * Copyright (C) 2025  Nguyen Trong Phuc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <vector>
#include <numeric>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <map>
#include "ultrasonic.hpp"

namespace
{
    // Constants for HC-SR04 sensor
    constexpr int TRIGGER_PULSE_US = 10;         // Trigger pulse duration
    constexpr float SOUND_SPEED_CM_US = 0.0343f; // Speed of sound: 343 m/s = 0.0343 cm/μs
    constexpr int MEASUREMENT_DELAY_US = 1000;   // Minimum delay between measurements
    constexpr int GPIO_EXPORT_RETRY_COUNT = 3;   // Retry count for GPIO operations
    constexpr int GPIO_SETUP_DELAY_MS = 100;     // Delay for GPIO setup
}

/**
 * @brief GPIO management helper class
 */
class GPIOManager
{
public:
    /**
     * @brief GPIO pin direction
     */
    enum Direction
    {
        INPUT,
        OUTPUT
    };

    /**
     * @brief GPIO pin value
     */
    enum Value
    {
        LOW = 0,
        HIGH = 1
    };

    /**
     * @brief Export a GPIO pin for use
     */
    static bool exportPin(const std::string &pin)
    {
        // Extract pin number from pin name (e.g., "P9_12" -> "60")
        std::string pinNumber = getPinNumber(pin);
        if (pinNumber.empty())
        {
            return false;
        }

        // Check if already exported
        std::string gpioPath = "/sys/class/gpio/gpio" + pinNumber;
        if (access(gpioPath.c_str(), F_OK) == 0)
        {
            return true; // Already exported
        }

        // Export the pin
        std::ofstream exportFile("/sys/class/gpio/export");
        if (!exportFile.is_open())
        {
            return false;
        }

        exportFile << pinNumber;
        exportFile.close();

        // Wait for the GPIO to be available
        for (int i = 0; i < GPIO_EXPORT_RETRY_COUNT; ++i)
        {
            if (access(gpioPath.c_str(), F_OK) == 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(GPIO_SETUP_DELAY_MS));
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        return false;
    }

    /**
     * @brief Set GPIO pin direction
     */
    static bool setDirection(const std::string &pin, Direction dir)
    {
        std::string pinNumber = getPinNumber(pin);
        if (pinNumber.empty())
        {
            return false;
        }

        std::string directionPath = "/sys/class/gpio/gpio" + pinNumber + "/direction";
        std::ofstream dirFile(directionPath);
        if (!dirFile.is_open())
        {
            return false;
        }

        dirFile << (dir == OUTPUT ? "out" : "in");
        return dirFile.good();
    }

    /**
     * @brief Set GPIO pin value
     */
    static bool setValue(const std::string &pin, Value value)
    {
        std::string pinNumber = getPinNumber(pin);
        if (pinNumber.empty())
        {
            return false;
        }

        std::string valuePath = "/sys/class/gpio/gpio" + pinNumber + "/value";
        std::ofstream valueFile(valuePath);
        if (!valueFile.is_open())
        {
            return false;
        }

        valueFile << static_cast<int>(value);
        return valueFile.good();
    }

    /**
     * @brief Get GPIO pin value
     */
    static Value getValue(const std::string &pin)
    {
        std::string pinNumber = getPinNumber(pin);
        if (pinNumber.empty())
        {
            return LOW;
        }

        std::string valuePath = "/sys/class/gpio/gpio" + pinNumber + "/value";
        std::ifstream valueFile(valuePath);
        if (!valueFile.is_open())
        {
            return LOW;
        }

        int value;
        valueFile >> value;
        return (value == 1) ? HIGH : LOW;
    }

    /**
     * @brief Unexport GPIO pin
     */
    static bool unexportPin(const std::string &pin)
    {
        std::string pinNumber = getPinNumber(pin);
        if (pinNumber.empty())
        {
            return false;
        }

        std::ofstream unexportFile("/sys/class/gpio/unexport");
        if (!unexportFile.is_open())
        {
            return false;
        }

        unexportFile << pinNumber;
        return unexportFile.good();
    }

private:
    /**
     * @brief Convert BeagleBone pin name to GPIO number
     */
    static std::string getPinNumber(const std::string &pin)
    {
        // Simplified mapping for common pins
        std::map<std::string, std::string> pinMap = {
            {"P9_11", "30"}, {"P9_12", "60"}, {"P9_13", "31"}, {"P9_14", "50"}, {"P9_15", "48"}, {"P9_16", "51"}, {"P9_17", "5"}, {"P9_18", "4"}, {"P9_21", "3"}, {"P9_22", "2"}, {"P9_23", "49"}, {"P9_24", "15"}, {"P9_25", "117"}, {"P9_26", "14"}, {"P9_27", "115"}, {"P9_28", "113"}, {"P9_29", "111"}, {"P9_30", "112"}, {"P9_31", "110"}, {"P9_42", "7"}, {"P8_07", "66"}, {"P8_08", "67"}, {"P8_09", "69"}, {"P8_10", "68"}, {"P8_11", "45"}, {"P8_12", "44"}, {"P8_13", "23"}, {"P8_14", "26"}, {"P8_15", "47"}, {"P8_16", "46"}, {"P8_17", "27"}, {"P8_18", "65"}};

        auto it = pinMap.find(pin);
        return (it != pinMap.end()) ? it->second : "";
        return (it != pinMap.end()) ? it->second : "";
    }
};

/**
 * @brief Implementation class for UltrasonicSensor
 */
class UltrasonicSensor::Impl
{
public:
    explicit Impl(const SensorConfig &config)
        : config(config), initialized(false), measurementCount(0),
          totalMeasurements(0), validMeasurements(0), lastError("")
    {
        filterBuffer.reserve(config.filterWindowSize);
    }

    ~Impl()
    {
        cleanup();
    }

    bool init()
    {
        if (initialized)
        {
            return true;
        }

        try
        {
            // Export and configure trigger pin
            if (!GPIOManager::exportPin(config.triggerPin))
            {
                lastError = "Failed to export trigger pin: " + config.triggerPin;
                return false;
            }

            if (!GPIOManager::setDirection(config.triggerPin, GPIOManager::OUTPUT))
            {
                lastError = "Failed to set trigger pin direction";
                return false;
            }

            // Export and configure echo pin
            if (!GPIOManager::exportPin(config.echoPin))
            {
                lastError = "Failed to export echo pin: " + config.echoPin;
                return false;
            }

            if (!GPIOManager::setDirection(config.echoPin, GPIOManager::INPUT))
            {
                lastError = "Failed to set echo pin direction";
                return false;
            }

            // Initialize trigger pin to LOW
            if (!GPIOManager::setValue(config.triggerPin, GPIOManager::LOW))
            {
                lastError = "Failed to initialize trigger pin state";
                return false;
            }

            // Wait for pins to stabilize
            std::this_thread::sleep_for(std::chrono::milliseconds(GPIO_SETUP_DELAY_MS));

            initialized = true;
            lastError = "";
            return true;
        }
        catch (const std::exception &e)
        {
            lastError = "Initialization exception: " + std::string(e.what());
            return false;
        }
    }

    void cleanup()
    {
        if (initialized)
        {
            // Set trigger pin to LOW before cleanup
            GPIOManager::setValue(config.triggerPin, GPIOManager::LOW);

            // Unexport pins
            GPIOManager::unexportPin(config.triggerPin);
            GPIOManager::unexportPin(config.echoPin);

            initialized = false;
        }
    }

    Measurement measure()
    {
        Measurement result;
        result.timestamp = std::chrono::high_resolution_clock::now();

        if (!initialized)
        {
            lastError = "Sensor not initialized";
            return result;
        }

        try
        {
            // Send trigger pulse
            if (!sendTriggerPulse())
            {
                lastError = "Failed to send trigger pulse";
                return result;
            }

            // Measure echo pulse duration
            int echoTimeUs = measureEchoPulse();
            if (echoTimeUs < 0)
            {
                lastError = "Echo measurement timeout or error";
                return result;
            }

            // Convert to distance
            float rawDistance = (echoTimeUs * SOUND_SPEED_CM_US) / 2.0f;

            // Validate measurement
            if (rawDistance < config.minDistance || rawDistance > config.maxDistance)
            {
                lastError = "Measurement out of range: " + std::to_string(rawDistance) + " cm";
                return result;
            }

            // Store raw values
            result.rawDistance = rawDistance;
            result.echoTimeUs = echoTimeUs;
            result.valid = true;

            // Apply filtering if enabled
            if (config.filteringEnabled)
            {
                result.distance = applyFilter(rawDistance);
            }
            else
            {
                result.distance = rawDistance;
            }

            // Update statistics
            updateStatistics(result);

            lastError = "";
            return result;
        }
        catch (const std::exception &e)
        {
            lastError = "Measurement exception: " + std::string(e.what());
            return result;
        }
    }

    Measurement measureAverage(int samples, int delayMs)
    {
        if (samples <= 0)
        {
            samples = 1;
        }

        std::vector<float> validDistances;
        validDistances.reserve(samples);

        Measurement lastMeasurement;
        int totalEchoTime = 0;
        int validSamples = 0;

        for (int i = 0; i < samples; ++i)
        {
            auto measurement = measure();

            if (measurement.valid)
            {
                validDistances.push_back(measurement.rawDistance);
                totalEchoTime += measurement.echoTimeUs;
                lastMeasurement = measurement;
                validSamples++;
            }

            if (i < samples - 1 && delayMs > 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
            }
        }

        Measurement result;
        result.timestamp = std::chrono::high_resolution_clock::now();

        if (validDistances.empty())
        {
            lastError = "No valid measurements in average calculation";
            return result;
        }

        // Calculate average
        float sum = std::accumulate(validDistances.begin(), validDistances.end(), 0.0f);
        result.rawDistance = sum / validDistances.size();
        result.echoTimeUs = totalEchoTime / validSamples;
        result.valid = true;

        // Apply filtering to averaged result
        if (config.filteringEnabled)
        {
            result.distance = applyFilter(result.rawDistance);
        }
        else
        {
            result.distance = result.rawDistance;
        }

        return result;
    }

    SensorConfig getConfig() const
    {
        return config;
    }

    void setConfig(const SensorConfig &newConfig)
    {
        config = newConfig;
    }

    void setFilteringEnabled(bool enabled)
    {
        config.filteringEnabled = enabled;
    }

    std::string getLastError() const
    {
        return lastError;
    }

    bool isInitialized() const
    {
        return initialized;
    }

    bool isObjectDetected(float maxRange)
    {
        auto measurement = measure();
        return measurement.valid && measurement.distance <= maxRange;
    }

    bool selfTest()
    {
        if (!initialized)
        {
            lastError = "Cannot perform self-test: sensor not initialized";
            return false;
        }

        try
        {
            // Test 1: Verify GPIO pins can be controlled
            if (!GPIOManager::setValue(config.triggerPin, GPIOManager::HIGH))
            {
                lastError = "Self-test failed: Cannot control trigger pin";
                return false;
            }

            std::this_thread::sleep_for(std::chrono::microseconds(10));

            if (!GPIOManager::setValue(config.triggerPin, GPIOManager::LOW))
            {
                lastError = "Self-test failed: Cannot reset trigger pin";
                return false;
            }

            // Test 2: Take a few measurements and check for consistency
            std::vector<float> testMeasurements;
            const int testSamples = 5;

            for (int i = 0; i < testSamples; ++i)
            {
                auto measurement = measure();
                if (measurement.valid)
                {
                    testMeasurements.push_back(measurement.distance);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (testMeasurements.empty())
            {
                lastError = "Self-test failed: No valid measurements obtained";
                return false;
            }

            // Test 3: Check measurement variance (should be reasonable)
            if (testMeasurements.size() >= 3)
            {
                float mean = std::accumulate(testMeasurements.begin(), testMeasurements.end(), 0.0f) / testMeasurements.size();
                float variance = 0.0f;

                for (float measurement : testMeasurements)
                {
                    variance += std::pow(measurement - mean, 2);
                }
                variance /= testMeasurements.size();
                float stdDev = std::sqrt(variance);

                // If standard deviation is too high, there might be an issue
                if (stdDev > mean * 0.5f)
                { // 50% of mean as threshold
                    lastError = "Self-test warning: High measurement variance (σ=" +
                                std::to_string(stdDev) + " cm)";
                    // Don't fail, just warn
                }
            }

            lastError = "";
            return true;
        }
        catch (const std::exception &e)
        {
            lastError = "Self-test exception: " + std::string(e.what());
            return false;
        }
    }

    std::string getStatistics() const
    {
        std::ostringstream oss;
        oss << "Ultrasonic Sensor Statistics:\n";
        oss << "  Total measurements: " << totalMeasurements << "\n";
        oss << "  Valid measurements: " << validMeasurements << "\n";
        oss << "  Success rate: " << std::fixed << std::setprecision(1);

        if (totalMeasurements > 0)
        {
            oss << (100.0f * validMeasurements / totalMeasurements) << "%\n";
        }
        else
        {
            oss << "N/A\n";
        }

        oss << "  Filtering: " << (config.filteringEnabled ? "enabled" : "disabled") << "\n";
        oss << "  Filter window: " << config.filterWindowSize << " samples\n";
        oss << "  Range: " << config.minDistance << " - " << config.maxDistance << " cm\n";
        oss << "  Timeout: " << config.timeoutUs << " μs\n";

        return oss.str();
    }

private:
    bool sendTriggerPulse()
    {
        // Ensure trigger is LOW
        if (!GPIOManager::setValue(config.triggerPin, GPIOManager::LOW))
        {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(2));

        // Send HIGH pulse
        if (!GPIOManager::setValue(config.triggerPin, GPIOManager::HIGH))
        {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(TRIGGER_PULSE_US));

        // Return to LOW
        return GPIOManager::setValue(config.triggerPin, GPIOManager::LOW);
    }

    int measureEchoPulse()
    {
        auto startTime = std::chrono::high_resolution_clock::now();
        auto timeoutTime = startTime + std::chrono::microseconds(config.timeoutUs);

        // Wait for echo to go HIGH
        while (GPIOManager::getValue(config.echoPin) == GPIOManager::LOW)
        {
            if (std::chrono::high_resolution_clock::now() > timeoutTime)
            {
                return -1; // Timeout waiting for echo start
            }
        }

        auto echoStart = std::chrono::high_resolution_clock::now();

        // Wait for echo to go LOW
        while (GPIOManager::getValue(config.echoPin) == GPIOManager::HIGH)
        {
            if (std::chrono::high_resolution_clock::now() > timeoutTime)
            {
                return -1; // Timeout waiting for echo end
            }
        }

        auto echoEnd = std::chrono::high_resolution_clock::now();

        // Calculate pulse duration in microseconds
        auto pulseDuration = std::chrono::duration_cast<std::chrono::microseconds>(echoEnd - echoStart);
        return static_cast<int>(pulseDuration.count());
    }

    float applyFilter(float newValue)
    {
        // Add new value to buffer
        filterBuffer.push_back(newValue);

        // Remove old values if buffer is full
        if (static_cast<int>(filterBuffer.size()) > config.filterWindowSize)
        {
            filterBuffer.erase(filterBuffer.begin());
        }

        // Calculate moving average
        float sum = std::accumulate(filterBuffer.begin(), filterBuffer.end(), 0.0f);
        return sum / filterBuffer.size();
    }

    void updateStatistics(const Measurement &measurement)
    {
        measurementCount++;
        totalMeasurements++;

        if (measurement.valid)
        {
            validMeasurements++;
        }
    }

    // Member variables
    SensorConfig config;
    bool initialized;
    std::vector<float> filterBuffer;

    // Statistics
    unsigned long measurementCount;
    unsigned long totalMeasurements;
    unsigned long validMeasurements;

    std::string lastError;
};

// Public interface implementation
UltrasonicSensor::UltrasonicSensor(const SensorConfig &config)
    : pImpl(std::make_unique<Impl>(config))
{
}

UltrasonicSensor::~UltrasonicSensor() = default;

bool UltrasonicSensor::init()
{
    return pImpl->init();
}

void UltrasonicSensor::cleanup()
{
    return pImpl->cleanup();
}

UltrasonicSensor::Measurement UltrasonicSensor::measure()
{
    return pImpl->measure();
}

UltrasonicSensor::Measurement UltrasonicSensor::measureAverage(int samples, int delayMs)
{
    return pImpl->measureAverage(samples, delayMs);
}

UltrasonicSensor::SensorConfig UltrasonicSensor::getConfig() const
{
    return pImpl->getConfig();
}

void UltrasonicSensor::setConfig(const SensorConfig &config)
{
    return pImpl->setConfig(config);
}

void UltrasonicSensor::setFilteringEnabled(bool enabled)
{
    pImpl->setFilteringEnabled(enabled);
}

std::string UltrasonicSensor::getLastError() const
{
    return pImpl->getLastError();
}

bool UltrasonicSensor::isInitialized() const
{
    return pImpl->isInitialized();
}

bool UltrasonicSensor::isObjectDetected(float maxRange)
{
    return pImpl->isObjectDetected(maxRange);
}

bool UltrasonicSensor::selfTest()
{
    return pImpl->selfTest();
}

std::string UltrasonicSensor::getStatistics() const
{
    return pImpl->getStatistics();
}
