#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <atomic>
#include <algorithm>
#include "hexapod.hpp"
#include "common.hpp"

// Global running flag for signal handling
static std::atomic<bool> running(true);

// Signal handler using common utilities
void signalHandler(int signal)
{
    running.store(false);
    common::ErrorReporter::reportInfo("MPU6050-Test", "Received termination signal " + std::to_string(signal));
}

// Display timing statistics for IMU readings
static void print_timing_stats(const double *intervals, int count)
{
    if (count < 2)
        return;

    double min = intervals[0];
    double max = intervals[0];
    double sum = 0.0;
    double avg, std_dev = 0.0;

    // Calculate min, max, sum
    for (int i = 0; i < count; i++)
    {
        min = std::min(min, intervals[i]);
        max = std::max(max, intervals[i]);
        sum += intervals[i];
    }

    // Calculate average
    avg = sum / count;

    // Calculate standard deviation
    for (int i = 0; i < count; i++)
    {
        std_dev += (intervals[i] - avg) * (intervals[i] - avg);
    }
    std_dev = std::sqrt(std_dev / count);

    std::cout << "\nIMU Timing Statistics (ms):" << std::endl;
    std::cout << "  Min: " << common::StringUtils::formatNumber(min * 1000.0)
              << " | Max: " << common::StringUtils::formatNumber(max * 1000.0)
              << " | Avg: " << common::StringUtils::formatNumber(avg * 1000.0)
              << " | Std Dev: " << common::StringUtils::formatNumber(std_dev * 1000.0)
              << " | Samples: " << count << std::endl;
}

int main()
{
    // Setup graceful shutdown handling using common utilities
    common::SignalManager::setupGracefulShutdown(running, signalHandler);

    // Initialize performance monitoring
    common::PerformanceMonitor perfMonitor;
    common::PerformanceMonitor initMonitor;

    // Setup terminal for immediate input using common utilities
    if (!common::TerminalManager::setupImmediate())
    {
        common::ErrorReporter::reportWarning("MPU6050-Test", "Failed to setup immediate terminal input");
    }

    // Initialize hexapod with retry logic and performance monitoring
    Hexapod hexapod;
    bool initialized = false;
    int retries = 3;

    std::cout << "MPU6050 Test - Press Ctrl+C to exit" << std::endl;
    std::cout << "================================" << std::endl;
    std::cout << "Note: MPU6050 sensor automatically wakes from sleep mode when needed\n"
              << std::endl;

    while (!initialized && retries-- > 0)
    {
        common::ErrorReporter::reportInfo("MPU6050-Test", "Initializing hexapod hardware (attempt " + std::to_string(3 - retries) + ")");

        initMonitor.startFrame();
        if (hexapod.init())
        {
            initialized = true;
            initMonitor.endFrame();
            common::ErrorReporter::reportInfo("MPU6050-Test", "Hexapod initialized successfully in " +
                                                                  common::StringUtils::formatNumber(initMonitor.getAverageFrameTime()) + "ms");
        }
        else
        {
            initMonitor.endFrame();
            common::ErrorReporter::reportError("MPU6050-Test", "Initialization", hexapod.getLastErrorMessage());
            if (retries > 0)
            {
                std::cout << "Retrying in 1 second..." << std::endl;
                common::sleepMs(1000);
            }
            else
            {
                common::ErrorReporter::reportError("MPU6050-Test", "Initialization", "Maximum retries reached, giving up");
                common::TerminalManager::restore();
                return 1;
            }
        }
    }

    // Main loop with timeout detection and statistics gathering using common utilities
    double last_successful_read = common::getCurrentTime();
    double read_intervals[100] = {0}; // Store last 100 read intervals
    int interval_idx = 0;
    int total_reads = 0;
    int successful_reads = 0;
    double last_stats_time = common::getCurrentTime();

    std::cout << "\nReading IMU data..." << std::endl;
    std::cout << "Press 'q' to quit, 's' to show statistics\n"
              << std::endl;

    while (running.load())
    {
        ImuData imuData;
        char key;

        perfMonitor.startFrame();
        double start_time = common::getCurrentTime();

        // Check for key press using common terminal utilities
        if (common::TerminalManager::readChar(key))
        {
            if (key == 'q' || key == 'Q')
            {
                common::ErrorReporter::reportInfo("MPU6050-Test", "User requested exit");
                break;
            }
            else if (key == 's' || key == 'S')
            {
                print_timing_stats(read_intervals, std::min(total_reads, 100));
            }
        }

        // Get IMU data with timeout detection
        bool success = hexapod.getImuData(imuData);
        double now = common::getCurrentTime();
        double elapsed = now - start_time;

        perfMonitor.endFrame();
        total_reads++;

        if (success)
        {
            // Record timing interval for statistics
            read_intervals[interval_idx] = elapsed;
            interval_idx = (interval_idx + 1) % 100;

            // Print formatted IMU data using common string utilities
            std::cout << "\rAccel: X=" << std::showpos << std::setw(6)
                      << common::StringUtils::formatNumber(imuData.getAccelX(), 2) << "g Y=" << std::setw(6)
                      << common::StringUtils::formatNumber(imuData.getAccelY(), 2)
                      << "g Z=" << std::setw(6) << common::StringUtils::formatNumber(imuData.getAccelZ(), 2)
                      << "g | Gyro: X=" << std::setw(7) << common::StringUtils::formatNumber(imuData.getGyroX(), 1)
                      << "° Y=" << std::setw(7) << common::StringUtils::formatNumber(imuData.getGyroY(), 1)
                      << "° Z=" << std::setw(7) << common::StringUtils::formatNumber(imuData.getGyroZ(), 1)
                      << "°/s" << std::flush;
            last_successful_read = now;
            successful_reads++;
        }
        else
        {
            if (now - last_successful_read > 5.0) // 5 second timeout
            {
                common::ErrorReporter::reportError("MPU6050-Test", "Timeout", "No response from IMU for 5 seconds");
                break;
            }
            std::cerr << "\rFailed to read IMU: " << hexapod.getLastErrorMessage() << std::flush;
        }

        // Show statistics periodically using common time utilities
        if (now - last_stats_time > 10.0)
        {
            double success_rate = (successful_reads * 100.0) / total_reads;
            std::cout << "\n\nSuccess rate: " << common::StringUtils::formatNumber(success_rate, 1)
                      << "% (" << successful_reads << "/"
                      << total_reads << ") in last 10 seconds" << std::endl;
            last_stats_time = now;
        }

        // Brief sleep to prevent excessive CPU usage - dynamically adjust based on performance
        common::sleepUs(elapsed < 0.005 ? 50000 : 10000); // Sleep longer if reads are fast
    }

    std::cout << "\n\nExiting MPU6050 test program..." << std::endl;

    // Print final statistics using common string utilities
    print_timing_stats(read_intervals, std::min(total_reads, 100));

    if (total_reads > 0)
    {
        double overall_success_rate = (successful_reads * 100.0) / total_reads;
        std::cout << "Overall success rate: " << common::StringUtils::formatNumber(overall_success_rate, 1)
                  << "% (" << successful_reads << "/"
                  << total_reads << ")" << std::endl;
    }

    // Print final performance report
    perfMonitor.printReport("IMU read operations ");

    // Explicitly cleanup
    hexapod.cleanup();

    // Restore terminal settings using common utilities
    common::TerminalManager::restore();

    return 0;
}
