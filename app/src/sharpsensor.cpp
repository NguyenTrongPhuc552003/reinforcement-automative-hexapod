#include "sharpsensor.hpp"
#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>
#include <thread>
#include <numeric>
#include <algorithm>
#include <stdexcept>
#include <filesystem>
#include <unistd.h>

// Possible ADC paths on different BeagleBone variants
const std::vector<std::string> POSSIBLE_ADC_PATHS = {
    "/sys/bus/iio/devices/iio:device0/",
    "/sys/devices/platform/ocp/44e0d000.tscadc/tiadc/iio:device0/",
    "/sys/devices/platform/ocp/tscadc@44e0d000/TI-am335x-adc/iio:device0/",
    "/sys/bus/iio/devices/iio:device1/"};

class SharpSensor::Impl
{
public:
    Impl(const std::string &ain_path)
        : m_ainPath(ain_path),
          m_initialized(false),
          m_scale(1.0f),
          m_offset(0.0f),
          m_filterEnabled(true),
          m_lastReadings(5, 0.0f), // Initialize filter buffer with 5 zeros
          m_adcPath("")
    {
    }

    bool init()
    {
        try
        {
            // Find the correct ADC path by checking all possible locations
            if (!findAdcPath())
            {
                std::cerr << "Error: Could not find ADC interface. Please check if ADC is enabled." << std::endl;
                printDiagnosticInfo();
                return false;
            }

            // Check permissions on the ADC files
            if (!checkPermissions())
            {
                return false;
            }

            m_initialized = true;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error initializing Sharp sensor: " << e.what() << std::endl;
            return false;
        }
    }

    bool findAdcPath()
    {
        // Try each possible ADC path
        for (const auto &path : POSSIBLE_ADC_PATHS)
        {
            try
            {
                std::ifstream test(path + "name");
                if (test.good())
                {
                    m_adcPath = path;
                    std::cout << "Found ADC at: " << m_adcPath << std::endl;

                    // Verify that specified AIN path will work
                    std::string fullPath = m_adcPath + m_ainPath;
                    std::ifstream ain_test(fullPath);
                    if (ain_test.good())
                    {
                        return true;
                    }
                    else
                    {
                        // Try with "_raw" suffix if needed
                        fullPath = m_adcPath + m_ainPath + "_raw";
                        std::ifstream ain_test_raw(fullPath);
                        if (ain_test_raw.good())
                        {
                            m_ainPath += "_raw"; // Add _raw suffix if needed
                            return true;
                        }
                    }
                }
            }
            catch (...)
            {
                // Continue to next path
            }
        }
        return false;
    }

    bool checkPermissions()
    {
        // Create path to ADC file
        std::string fullPath = m_adcPath + m_ainPath;

        // Check access permissions
        if (access(fullPath.c_str(), R_OK) != 0)
        {
            std::cerr << "Error: Permission denied for ADC file: " << fullPath << std::endl;
            std::cerr << "Try running with sudo privileges or add your user to the correct group" << std::endl;
            return false;
        }
        return true;
    }

    void printDiagnosticInfo()
    {
        std::cout << "\n--- Diagnostic Information ---" << std::endl;
        std::cout << "Checking for ADC files:" << std::endl;

        // Check for commonly used ADC devices
        for (const auto &path : POSSIBLE_ADC_PATHS)
        {
            std::cout << "Checking path: " << path << " - ";
            if (std::filesystem::exists(path))
            {
                std::cout << "EXISTS" << std::endl;

                // Check for in_voltage files
                for (int i = 0; i < 8; i++)
                {
                    std::string voltage_path = path + "in_voltage" + std::to_string(i) + "_raw";
                    std::cout << "  " << voltage_path << " - ";
                    if (std::filesystem::exists(voltage_path))
                    {
                        std::cout << "EXISTS";
                        if (access(voltage_path.c_str(), R_OK) == 0)
                        {
                            std::cout << " (Readable)";
                        }
                        else
                        {
                            std::cout << " (Permission denied)";
                        }
                    }
                    else
                    {
                        std::cout << "NOT FOUND";
                    }
                    std::cout << std::endl;
                }
            }
            else
            {
                std::cout << "NOT FOUND" << std::endl;
            }
        }

        std::cout << "\nMake sure ADC is enabled in BeagleBone device tree." << std::endl;
        std::cout << "You might need to run: sudo sh -c 'echo BB-ADC > /sys/devices/platform/bone_capemgr/slots'" << std::endl;
        std::cout << "--------------------------------" << std::endl;
    }

    int readRawValue()
    {
        if (!m_initialized)
        {
            std::cerr << "Error: Sensor not initialized" << std::endl;
            return -1;
        }

        try
        {
            // Use detected ADC path + AIN path
            std::string fullPath = m_adcPath + m_ainPath;
            std::ifstream ain_file(fullPath);
            if (!ain_file.good())
            {
                std::cerr << "Error: Failed to open ADC input file: " << fullPath << std::endl;
                return -1;
            }

            int value;
            ain_file >> value;
            return value;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error reading ADC value: " << e.what() << std::endl;
            return -1;
        }
    }

    float readDistanceCm()
    {
        int rawValue = readRawValue();
        if (rawValue < 0)
        {
            return -1.0f;
        }

        // Apply scaling and offset
        float adjustedValue = rawValue * m_scale + m_offset;

        // Convert to distance using the Sharp GP2Y0A21YK0F characteristic curve
        // This is an approximation of the inverse relationship
        float distance = convertToDistance(adjustedValue);

        // Apply filtering if enabled
        if (m_filterEnabled)
        {
            distance = applyFilter(distance);
        }

        return distance;
    }

    float readAverageDistanceCm(int samples)
    {
        if (samples <= 0)
        {
            return -1.0f;
        }

        std::vector<float> readings;
        readings.reserve(samples);

        for (int i = 0; i < samples; i++)
        {
            float dist = readDistanceCm();
            if (dist > 0)
            { // Only include valid readings
                readings.push_back(dist);
            }
            // Small delay between readings
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        if (readings.empty())
        {
            return -1.0f;
        }

        // Calculate average
        return std::accumulate(readings.begin(), readings.end(), 0.0f) / readings.size();
    }

    void setScale(float scale)
    {
        m_scale = scale;
    }

    void setOffset(float offset)
    {
        m_offset = offset;
    }

    void enableFiltering(bool enable)
    {
        m_filterEnabled = enable;
        // Reset filter buffer when toggling filtering
        std::fill(m_lastReadings.begin(), m_lastReadings.end(), 0.0f);
    }

private:
    std::string m_ainPath;
    bool m_initialized;
    float m_scale;
    float m_offset;
    bool m_filterEnabled;
    std::vector<float> m_lastReadings;
    std::string m_adcPath;

    float convertToDistance(float adcValue)
    {
        // For GP2Y0A21YK0F, the relationship between voltage and distance is roughly inverse
        // This is calibrated for BeagleBone AI with 12-bit ADC (0-4095)

        // Prevent division by zero or negative values
        if (adcValue <= 0)
        {
            return 80.0f; // Maximum range
        }

        // Constants calibrated for GP2Y0A21YK0F based on datasheet
        constexpr float a = 12000.0f;
        constexpr float b = 0.0f;

        float distance = a / (adcValue - b);

        // Clamp to sensor's operating range (10-80cm)
        distance = std::max(10.0f, std::min(80.0f, distance));

        return distance;
    }

    float applyFilter(float newReading)
    {
        // Simple moving average filter
        // Shift readings in the buffer
        std::rotate(m_lastReadings.begin(), m_lastReadings.begin() + 1, m_lastReadings.end());
        m_lastReadings.back() = newReading;

        // Calculate average, excluding zeros (which indicate no prior readings)
        float sum = 0.0f;
        int count = 0;
        for (float reading : m_lastReadings)
        {
            if (reading > 0)
            {
                sum += reading;
                count++;
            }
        }

        return count > 0 ? sum / count : newReading;
    }
};

// Implementation of the public interface

SharpSensor::SharpSensor(const std::string &ain_path)
    : pImpl(std::make_unique<Impl>(ain_path))
{
}

SharpSensor::~SharpSensor() = default;

bool SharpSensor::init()
{
    return pImpl->init();
}

int SharpSensor::readRawValue()
{
    return pImpl->readRawValue();
}

float SharpSensor::readDistanceCm()
{
    return pImpl->readDistanceCm();
}

float SharpSensor::readAverageDistanceCm(int samples)
{
    return pImpl->readAverageDistanceCm(samples);
}

void SharpSensor::setScale(float scale)
{
    pImpl->setScale(scale);
}

void SharpSensor::setOffset(float offset)
{
    pImpl->setOffset(offset);
}

void SharpSensor::enableFiltering(bool enable)
{
    pImpl->enableFiltering(enable);
}
