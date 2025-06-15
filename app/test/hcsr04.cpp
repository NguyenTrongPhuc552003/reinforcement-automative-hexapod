#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <thread>
#include <csignal>
#include <cmath>
#include <algorithm>
#include "ultrasonic.hpp"

//==============================================================================
// Test Program Support Types and Functions
//==============================================================================

namespace
{
    // Program control and statistics
    volatile bool running = true;

    /**
     * @brief Collection of measurement statistics
     */
    struct Statistics
    {
        float min = std::numeric_limits<float>::max();
        float max = std::numeric_limits<float>::min();
        float sum = 0.0f;
        size_t count = 0;
        size_t errors = 0;

        void update(float value)
        {
            min = std::min(min, value);
            max = std::max(max, value);
            sum += value;
            count++;
        }

        void addError() { errors++; }

        float getAverage() const
        {
            return count > 0 ? sum / count : 0.0f;
        }

        float getSuccessRate() const
        {
            size_t total = count + errors;
            return total > 0 ? (count * 100.0f) / total : 0.0f;
        }

        float getStandardDeviation(const std::vector<float> &measurements) const
        {
            if (count < 2)
                return 0.0f;

            float mean = getAverage();
            float variance = 0.0f;

            for (float value : measurements)
            {
                variance += (value - mean) * (value - mean);
            }

            return std::sqrt(variance / (count - 1));
        }
    };

    /**
     * @brief Signal handler for graceful shutdown
     */
    void signalHandler(int sig)
    {
        std::cout << "\nReceived signal " << sig << ", shutting down...\n";
        running = false;
    }

    /**
     * @brief Display real-time measurement data
     */
    void printMeasurement(float distance, const Statistics &stats)
    {
        static const char spinner[] = {'|', '/', '-', '\\'};
        std::cout << "\r" << spinner[stats.count % 4]
                  << " Distance: " << std::fixed << std::setprecision(1)
                  << distance << " cm (Valid: " << stats.count
                  << ", Errors: " << stats.errors << ")"
                  << std::flush;
    }

    /**
     * @brief Display final measurement statistics
     */
    void printStatistics(const Statistics &stats, const std::vector<float> &measurements)
    {
        if (stats.count == 0)
            return;

        float stddev = stats.getStandardDeviation(measurements);

        std::cout << "\n\nMeasurement Statistics:\n"
                  << "  Total Samples: " << (stats.count + stats.errors) << "\n"
                  << "  Valid Samples: " << stats.count << "\n"
                  << "  Errors: " << stats.errors << "\n"
                  << "  Success Rate: " << std::fixed << std::setprecision(1)
                  << stats.getSuccessRate() << "%\n"
                  << "  Distance Range: " << stats.min << " to " << stats.max << " cm\n"
                  << "  Average Distance: " << stats.getAverage() << " cm\n"
                  << "  Standard Deviation: " << stddev << " cm\n";
    }
}

//==============================================================================
// Main Test Program
//==============================================================================

int main()
{
    // Setup signal handlers for graceful shutdown
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout << "HC-SR04 Ultrasonic Sensor Test\n"
              << "==============================\n"
              << "Testing sensor with:\n"
              << "  - Trigger GPIO: " << ultrasonic::Config::DEFAULT_TRIGGER_GPIO << "\n"
              << "  - Echo GPIO: " << ultrasonic::Config::DEFAULT_ECHO_GPIO << "\n"
              << "  - Measurement interval: "
              << ultrasonic::Config::MIN_MEASUREMENT_INTERVAL_MS << "ms\n";

    // Create and initialize sensor
    ultrasonic::Ultrasonic sensor;
    if (auto err = sensor.initialize())
    {
        std::cerr << "Failed to initialize sensor: " << err.message() << '\n';
        return 1;
    }

    // Measurement tracking
    Statistics stats;
    std::vector<float> measurements;
    measurements.reserve(1000); // Pre-allocate for efficiency

    std::cout << "\nMeasuring distances... Press Ctrl+C to stop\n\n";

    // Main measurement loop
    auto testStart = std::chrono::steady_clock::now();

    while (running)
    {
        // Get measurement with error handling
        auto [distance, error] = sensor.getDistance();

        if (error)
        {
            stats.addError();
            std::cerr << "\rError: " << error.message() << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Record valid measurement
        stats.update(distance);
        measurements.push_back(distance);
        printMeasurement(distance, stats);

        // Rate limit measurements
        std::this_thread::sleep_for(
            std::chrono::milliseconds(ultrasonic::Config::MIN_MEASUREMENT_INTERVAL_MS));
    }

    // Calculate test duration
    auto testDuration = std::chrono::duration_cast<std::chrono::seconds>(
                            std::chrono::steady_clock::now() - testStart)
                            .count();

    // Display final results
    std::cout << "\n\nTest completed after " << testDuration << " seconds.\n";
    printStatistics(stats, measurements);

    // Display measurement rate
    float measurementRate = float(stats.count + stats.errors) / testDuration;
    std::cout << "Average measurement rate: " << std::fixed << std::setprecision(1)
              << measurementRate << " Hz\n";

    return 0;
}
