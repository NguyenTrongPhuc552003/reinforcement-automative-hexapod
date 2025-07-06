#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <atomic>
#include <cmath>
#include "hexapod.hpp"
#include "controller.hpp"
#include "common.hpp"

// Global running flag for signal handling
static std::atomic<bool> running(true);

// Signal handler using common utilities
void signalHandler(int signal)
{
    running.store(false);
    common::ErrorReporter::reportInfo("Balance-Test", "Received termination signal " + std::to_string(signal));
}

// Display test menu
static void print_menu()
{
    std::cout << "\nBalance Test Program - Commands\n"
              << "===============================\n"
              << "  1: Test IMU sensor readings\n"
              << "  2: Test balance adjustments\n"
              << "  3: Continuous balance mode\n"
              << "  4: Manual tilt test\n"
              << "  5: Response sensitivity test\n"
              << "  6: Deadzone calibration\n"
              << "  7: Balance stress test\n"
              << "  8: IMU calibration\n"
              << "  b: Toggle balance mode\n"
              << "  +/-: Adjust response factor\n"
              << "  [/]: Adjust deadzone\n"
              << "  s: Show balance status\n"
              << "  c: Center all legs\n"
              << "  h: Show this help\n"
              << "  q: Quit\n\n";
}

// Test IMU sensor readings
bool test_imu_readings(hexapod::Hexapod &hexapod, double duration)
{
    common::ErrorReporter::reportInfo("Balance-Test", "Testing IMU readings for " +
                                                          common::StringUtils::formatDuration(duration));

    common::PerformanceMonitor perfMonitor;
    auto start_time = common::getCurrentTime();
    auto end_time = start_time + duration;

    int successful_reads = 0;
    int total_reads = 0;
    std::vector<double> roll_readings, pitch_readings;

    std::cout << "Reading IMU data for " << common::StringUtils::formatDuration(duration) << "..." << std::endl;
    std::cout << "Press 'q' to abort early" << std::endl;

    while (running.load() && common::getCurrentTime() < end_time)
    {
        perfMonitor.startFrame();

        hexapod::ImuData imuData;
        if (hexapod.getImuData(imuData))
        {
            successful_reads++;

            // Calculate roll and pitch from accelerometer data
            double ax = imuData.getAccelX();
            double ay = imuData.getAccelY();
            double az = imuData.getAccelZ();

            double roll = atan2(ay, az) * 180.0 / M_PI;
            double pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

            roll_readings.push_back(roll);
            pitch_readings.push_back(pitch);

            // Display real-time data
            std::cout << "\rRoll: " << std::setw(6) << common::StringUtils::formatNumber(roll, 1) << "°"
                      << " Pitch: " << std::setw(6) << common::StringUtils::formatNumber(pitch, 1) << "°"
                      << " | Accel: " << common::StringUtils::formatNumber(imuData.getAccelX(), 2) << "g "
                      << common::StringUtils::formatNumber(imuData.getAccelY(), 2) << "g "
                      << common::StringUtils::formatNumber(imuData.getAccelZ(), 2) << "g" << std::flush;
        }
        else
        {
            std::cerr << "\rIMU read failed: " << hexapod.getLastErrorMessage() << std::flush;
        }

        perfMonitor.endFrame();
        total_reads++;

        // Check for user abort
        char key;
        if (common::TerminalManager::readChar(key) && (key == 'q' || key == 27))
        {
            std::cout << "\nIMU test aborted by user" << std::endl;
            return false;
        }

        common::sleepMs(50); // 20Hz update rate
    }

    std::cout << std::endl;
    perfMonitor.printReport("IMU reading ");

    // Calculate statistics
    if (!roll_readings.empty())
    {
        double roll_avg = 0, pitch_avg = 0;
        for (size_t i = 0; i < roll_readings.size(); i++)
        {
            roll_avg += roll_readings[i];
            pitch_avg += pitch_readings[i];
        }
        roll_avg /= roll_readings.size();
        pitch_avg /= pitch_readings.size();

        double roll_std = 0, pitch_std = 0;
        for (size_t i = 0; i < roll_readings.size(); i++)
        {
            roll_std += pow(roll_readings[i] - roll_avg, 2);
            pitch_std += pow(pitch_readings[i] - pitch_avg, 2);
        }
        roll_std = sqrt(roll_std / roll_readings.size());
        pitch_std = sqrt(pitch_std / pitch_readings.size());

        std::cout << "\nIMU Statistics:" << std::endl;
        std::cout << "  Success rate: " << common::StringUtils::formatNumber((successful_reads * 100.0) / total_reads, 1)
                  << "% (" << successful_reads << "/" << total_reads << ")" << std::endl;
        std::cout << "  Roll: avg=" << common::StringUtils::formatNumber(roll_avg, 1) << "° std="
                  << common::StringUtils::formatNumber(roll_std, 2) << "°" << std::endl;
        std::cout << "  Pitch: avg=" << common::StringUtils::formatNumber(pitch_avg, 1) << "° std="
                  << common::StringUtils::formatNumber(pitch_std, 2) << "°" << std::endl;
    }

    return successful_reads > 0;
}

// Test balance adjustments
bool test_balance_adjustments(controller::Controller &controller, double duration)
{
    common::ErrorReporter::reportInfo("Balance-Test", "Testing balance adjustments for " +
                                                          common::StringUtils::formatDuration(duration));

    // Enable balance mode
    controller.setBalanceEnabled(true);

    common::PerformanceMonitor perfMonitor;
    auto start_time = common::getCurrentTime();
    auto end_time = start_time + duration;

    std::cout << "Balance mode enabled. Tilt the robot to test adjustments..." << std::endl;
    std::cout << "Press 'q' to stop test" << std::endl;

    while (running.load() && common::getCurrentTime() < end_time)
    {
        perfMonitor.startFrame();

        if (!controller.update())
        {
            common::ErrorReporter::reportError("Balance-Test", "Controller Update", "Failed to update controller");
            return false;
        }

        perfMonitor.endFrame();

        // Show progress
        double progress = (common::getCurrentTime() - start_time) / duration;
        if (static_cast<int>(progress * 20) % 5 == 0) // Update every 25%
        {
            common::ProgressBar::displayPercent(progress, 30, "Balance test");
        }

        // Check for user abort
        char key;
        if (common::TerminalManager::readChar(key) && (key == 'q' || key == 27))
        {
            std::cout << "\nBalance test aborted by user" << std::endl;
            break;
        }

        common::sleepMs(20); // 50Hz update rate
    }

    common::ProgressBar::clear();
    perfMonitor.printReport("Balance adjustment ");
    std::cout << "Balance adjustment test completed" << std::endl;
    return true;
}

// Manual tilt test
bool test_manual_tilt(controller::Controller &controller)
{
    common::ErrorReporter::reportInfo("Balance-Test", "Running manual tilt test");

    std::vector<std::pair<double, double>> test_tilts = {
        {0.0, 0.0},   // Center
        {10.0, 0.0},  // Forward tilt
        {-10.0, 0.0}, // Backward tilt
        {0.0, 10.0},  // Right tilt
        {0.0, -10.0}, // Left tilt
        {5.0, 5.0},   // Forward-right
        {-5.0, -5.0}, // Backward-left
        {0.0, 0.0}    // Return to center
    };

    std::cout << "Testing manual tilt positions..." << std::endl;

    common::PerformanceMonitor perfMonitor;

    for (size_t i = 0; i < test_tilts.size(); i++)
    {
        double tiltX = test_tilts[i].first;
        double tiltY = test_tilts[i].second;

        std::cout << "Setting tilt: X=" << common::StringUtils::formatNumber(tiltX, 1)
                  << "° Y=" << common::StringUtils::formatNumber(tiltY, 1) << "°..." << std::flush;

        perfMonitor.startFrame();
        controller.setTilt(tiltX, tiltY);

        // Update controller to apply the tilt
        for (int j = 0; j < 10; j++) // Allow time for tilt to be applied
        {
            if (!controller.update())
            {
                std::cout << " FAILED" << std::endl;
                common::ErrorReporter::reportError("Balance-Test", "Manual Tilt", "Controller update failed");
                return false;
            }
            common::sleepMs(50);
        }
        perfMonitor.endFrame();

        std::cout << " OK (" << common::StringUtils::formatNumber(perfMonitor.getAverageFrameTime()) << "ms)" << std::endl;

        // Check for user abort
        char key;
        if (common::TerminalManager::readChar(key) && (key == 'q' || key == 27))
        {
            std::cout << "Manual tilt test aborted by user" << std::endl;
            return false;
        }

        // Pause between positions
        common::sleepMs(1000);
    }

    std::cout << "Manual tilt test completed successfully" << std::endl;
    return true;
}

// Response sensitivity test
bool test_response_sensitivity(controller::Controller &controller)
{
    common::ErrorReporter::reportInfo("Balance-Test", "Testing response sensitivity");

    std::vector<double> response_factors = {0.2, 0.5, 0.8, 1.0};

    std::cout << "Testing different response sensitivity levels..." << std::endl;

    for (double factor : response_factors)
    {
        std::cout << "Testing response factor: " << common::StringUtils::formatNumber(factor, 1) << std::endl;

        controller.setBalanceResponseFactor(factor);
        controller.setBalanceEnabled(true);

        // Run for a short period with this setting
        auto start_time = common::getCurrentTime();
        auto end_time = start_time + 3.0; // 3 seconds per test

        common::PerformanceMonitor perfMonitor;
        int updates = 0;

        while (running.load() && common::getCurrentTime() < end_time)
        {
            perfMonitor.startFrame();
            if (!controller.update())
            {
                common::ErrorReporter::reportError("Balance-Test", "Sensitivity Test", "Controller update failed");
                return false;
            }
            perfMonitor.endFrame();
            updates++;
            common::sleepMs(50);
        }

        perfMonitor.printReport("  Response factor " + common::StringUtils::formatNumber(factor, 1) + " ");
        std::cout << "  Updates: " << updates << std::endl;

        // Brief pause between tests
        common::sleepMs(500);
    }

    std::cout << "Response sensitivity test completed" << std::endl;
    return true;
}

// Deadzone calibration test
bool test_deadzone_calibration(controller::Controller &controller)
{
    common::ErrorReporter::reportInfo("Balance-Test", "Testing deadzone calibration");

    std::vector<double> deadzone_values = {0.5, 1.0, 2.0, 5.0};

    std::cout << "Testing different deadzone values..." << std::endl;

    for (double deadzone : deadzone_values)
    {
        std::cout << "Testing deadzone: " << common::StringUtils::formatNumber(deadzone, 1) << "°" << std::endl;

        controller.setBalanceDeadzone(deadzone);
        controller.setBalanceEnabled(true);

        // Run for a short period with this setting
        auto start_time = common::getCurrentTime();
        auto end_time = start_time + 3.0;

        while (running.load() && common::getCurrentTime() < end_time)
        {
            if (!controller.update())
            {
                common::ErrorReporter::reportError("Balance-Test", "Deadzone Test", "Controller update failed");
                return false;
            }
            common::sleepMs(50);
        }

        std::cout << "  Deadzone " << common::StringUtils::formatNumber(deadzone, 1) << "° completed" << std::endl;
        common::sleepMs(500);
    }

    std::cout << "Deadzone calibration test completed" << std::endl;
    return true;
}

// Balance stress test
bool test_balance_stress(controller::Controller &controller, double duration)
{
    common::ErrorReporter::reportInfo("Balance-Test", "Running balance stress test for " +
                                                          common::StringUtils::formatDuration(duration));

    controller.setBalanceEnabled(true);

    common::PerformanceMonitor perfMonitor;
    auto start_time = common::getCurrentTime();
    auto end_time = start_time + duration;

    int successful_updates = 0;
    int total_updates = 0;

    std::cout << "Running continuous balance updates for " << common::StringUtils::formatDuration(duration) << "..." << std::endl;
    std::cout << "Press 'q' to abort early" << std::endl;

    while (running.load() && common::getCurrentTime() < end_time)
    {
        perfMonitor.startFrame();

        if (controller.update())
        {
            successful_updates++;
        }
        total_updates++;

        perfMonitor.endFrame();

        // Show progress every 100 updates
        if (total_updates % 100 == 0)
        {
            double progress = (common::getCurrentTime() - start_time) / duration;
            common::ProgressBar::displayPercent(progress, 40, "Stress test");
        }

        // Check for user abort
        char key;
        if (common::TerminalManager::readChar(key) && (key == 'q' || key == 27))
        {
            std::cout << "\nStress test aborted by user" << std::endl;
            break;
        }

        common::sleepMs(10); // 100Hz update rate for stress test
    }

    common::ProgressBar::clear();
    perfMonitor.printReport("Balance stress test ");

    std::cout << "Stress test results:" << std::endl;
    std::cout << "  Total updates: " << total_updates << std::endl;
    std::cout << "  Successful: " << successful_updates << std::endl;
    std::cout << "  Success rate: " << common::StringUtils::formatNumber((successful_updates * 100.0) / total_updates, 1)
              << "%" << std::endl;

    return (successful_updates * 100.0) / total_updates > 95.0; // 95% success rate threshold
}

// Show balance status
void show_balance_status(controller::Controller &controller)
{
    std::cout << "\nBalance System Status" << std::endl;
    std::cout << "=====================" << std::endl;

    auto config = controller.getBalanceConfig();

    std::cout << "Balance mode: " << (controller.isBalanceEnabled() ? "ENABLED" : "disabled") << std::endl;
    std::cout << "Response factor: " << common::StringUtils::formatNumber(config.response_factor, 2) << std::endl;
    std::cout << "Deadzone: " << common::StringUtils::formatNumber(config.deadzone, 1) << "°" << std::endl;
    std::cout << "Max adjustment: " << common::StringUtils::formatNumber(config.max_tilt_adjustment, 1) << "°" << std::endl;
    std::cout << std::endl;
}

int main()
{
    // Setup graceful shutdown handling using common utilities
    common::SignalManager::setupGracefulShutdown(running, signalHandler);

    // Initialize performance monitoring
    common::PerformanceMonitor perfMonitor;

    // Setup terminal for immediate input using common utilities
    if (!common::TerminalManager::setupImmediate())
    {
        common::ErrorReporter::reportWarning("Balance-Test", "Failed to setup immediate terminal input");
    }

    std::cout << "Balance Test Program" << std::endl;
    std::cout << "====================" << std::endl;
    std::cout << "Testing hexapod balance and IMU systems" << std::endl;

    // Initialize hexapod
    hexapod::Hexapod hexapod;

    perfMonitor.startFrame();
    if (!hexapod.init())
    {
        common::ErrorReporter::reportError("Balance-Test", "Initialization", hexapod.getLastErrorMessage());
        common::TerminalManager::restore();
        return 1;
    }
    perfMonitor.endFrame();

    common::ErrorReporter::reportInfo("Balance-Test", "Hexapod initialized successfully in " +
                                                          common::StringUtils::formatNumber(perfMonitor.getAverageFrameTime()) + "ms");

    // Initialize controller
    controller::Controller controller(hexapod);
    if (!controller.init())
    {
        common::ErrorReporter::reportError("Balance-Test", "Controller Init", "Failed to initialize controller");
        common::TerminalManager::restore();
        return 1;
    }

    // Center all legs for safety
    std::cout << "Centering all legs for safety..." << std::endl;
    if (!hexapod.centerAll())
    {
        common::ErrorReporter::reportWarning("Balance-Test", "Failed to center legs");
    }

    print_menu();

    // Main test loop
    while (running.load())
    {
        std::cout << "Enter command (h for help): ";
        char command;

        if (common::TerminalManager::readChar(command))
        {
            switch (command)
            {
            case '1': // IMU readings test
                test_imu_readings(hexapod, 10.0);
                break;

            case '2': // Balance adjustments test
                test_balance_adjustments(controller, 15.0);
                break;

            case '3': // Continuous balance mode
            {
                std::cout << "Continuous balance mode activated. Tilt the robot to see adjustments." << std::endl;
                std::cout << "Press any key to stop..." << std::endl;

                controller.setBalanceEnabled(true);

                while (running.load())
                {
                    if (!controller.update())
                    {
                        common::ErrorReporter::reportError("Balance-Test", "Continuous Mode", "Controller update failed");
                        break;
                    }

                    char key;
                    if (common::TerminalManager::readChar(key))
                    {
                        break;
                    }

                    common::sleepMs(20);
                }

                controller.setBalanceEnabled(false);
                std::cout << "Continuous balance mode stopped" << std::endl;
                break;
            }

            case '4': // Manual tilt test
                test_manual_tilt(controller);
                break;

            case '5': // Response sensitivity test
                test_response_sensitivity(controller);
                break;

            case '6': // Deadzone calibration
                test_deadzone_calibration(controller);
                break;

            case '7': // Stress test
                test_balance_stress(controller, 30.0);
                break;

            case '8': // IMU calibration
                std::cout << "IMU calibration not yet implemented" << std::endl;
                break;

            case 'b': // Toggle balance mode
                controller.setBalanceEnabled(!controller.isBalanceEnabled());
                break;

            case '+': // Increase response factor
            {
                auto config = controller.getBalanceConfig();
                controller.setBalanceResponseFactor(std::min(1.0, config.response_factor + 0.1));
                break;
            }

            case '-': // Decrease response factor
            {
                auto config = controller.getBalanceConfig();
                controller.setBalanceResponseFactor(std::max(0.1, config.response_factor - 0.1));
                break;
            }

            case '[': // Decrease deadzone
            {
                auto config = controller.getBalanceConfig();
                controller.setBalanceDeadzone(std::max(0.0, config.deadzone - 0.5));
                break;
            }

            case ']': // Increase deadzone
            {
                auto config = controller.getBalanceConfig();
                controller.setBalanceDeadzone(std::min(10.0, config.deadzone + 0.5));
                break;
            }

            case 's': // Show status
                show_balance_status(controller);
                break;

            case 'c': // Center legs
                std::cout << "Centering all legs..." << std::endl;
                if (hexapod.centerAll())
                {
                    std::cout << "All legs centered successfully" << std::endl;
                }
                else
                {
                    common::ErrorReporter::reportError("Balance-Test", "Center", hexapod.getLastErrorMessage());
                }
                break;

            case 'h': // Help
                print_menu();
                break;

            case 'q': // Quit
                std::cout << "Exiting balance test program..." << std::endl;
                running.store(false);
                break;

            default:
                std::cout << "Unknown command. Press 'h' for help." << std::endl;
                break;
            }
        }

        // Small delay to prevent excessive CPU usage
        common::sleepMs(50);
    }

    // Final safety - disable balance and center legs before exit
    std::cout << "Disabling balance mode and centering legs for safe shutdown..." << std::endl;
    controller.setBalanceEnabled(false);
    hexapod.centerAll();

    // Restore terminal settings
    common::TerminalManager::restore();

    std::cout << "Balance test program completed." << std::endl;
    return 0;
}
