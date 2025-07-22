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
#include <iomanip>
#include <string>
#include <vector>
#include <atomic>
#include <cmath>
#include "hexapod.hpp"
#include "cpg/controller.hpp"
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
    std::cout << "\nCPG Balance Test Program - Commands\n"
              << "===================================\n"
              << "  1: Test IMU sensor readings\n"
              << "  2: Test balance adjustments with CPG\n"
              << "  3: Continuous balance mode\n"
              << "  4: Manual tilt test with IMU\n"
              << "  5: Autonomous walking test\n"
              << "  6: Obstacle avoidance test\n"
              << "  s: Show CPG controller status\n"
              << "  c: Center all legs\n"
              << "  h: Show this help\n"
              << "  q: Quit\n"
              << std::endl;
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

// Test balance adjustments with CPG
bool test_balance_adjustments(cpg::Controller &controller, double duration)
{
    common::ErrorReporter::reportInfo("Balance-Test", "Testing balance adjustments for " +
                                                          common::StringUtils::formatDuration(duration));

    hexapod::Hexapod hexapod;
    if (!hexapod.init())
    {
        common::ErrorReporter::reportError("Balance-Test", "Hexapod Init", hexapod.getLastErrorMessage());
        return false;
    }

    common::PerformanceMonitor perfMonitor;
    auto start_time = common::getCurrentTime();
    auto end_time = start_time + duration;

    std::cout << "CPG balance feedback test. Tilt the robot to test adjustments..." << std::endl;
    std::cout << "Press 'q' to stop test" << std::endl;

    while (running.load() && common::getCurrentTime() < end_time)
    {
        perfMonitor.startFrame();

        // Read IMU data and provide balance feedback to CPG controller
        hexapod::ImuData imuData;
        if (hexapod.getImuData(imuData))
        {
            double roll = atan2(imuData.getAccelY(), imuData.getAccelZ());
            double pitch = atan2(-imuData.getAccelX(), sqrt(imuData.getAccelY() * imuData.getAccelY() + imuData.getAccelZ() * imuData.getAccelZ()));

            std::vector<double> angular_velocity = {imuData.getGyroX(), imuData.getGyroY(), imuData.getGyroZ()};
            controller.updateBalanceFeedback(roll, pitch, angular_velocity);
        }

        if (!controller.update(0.01)) // 10ms update rate
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

// Autonomous walking test
bool test_autonomous_walking(cpg::Controller &controller, double duration)
{
    common::ErrorReporter::reportInfo("Balance-Test", "Testing autonomous walking for " +
                                                          common::StringUtils::formatDuration(duration));

    hexapod::Hexapod hexapod;
    if (!hexapod.init())
    {
        common::ErrorReporter::reportError("Balance-Test", "Hexapod Init", hexapod.getLastErrorMessage());
        return false;
    }

    auto start_time = common::getCurrentTime();
    auto end_time = start_time + duration;

    std::cout << "Starting autonomous walking with IMU balance feedback..." << std::endl;
    std::cout << "Press 'q' to stop test" << std::endl;

    // Start walking with CPG
    cpg::LocomotionCommand walkCommand(0.1, 0.0, "tripod"); // Slow forward walk
    if (!controller.startLocomotion(walkCommand))
    {
        common::ErrorReporter::reportError("Balance-Test", "Walking", "Failed to start walking");
        return false;
    }

    while (running.load() && common::getCurrentTime() < end_time)
    {
        // Read IMU and provide balance feedback
        hexapod::ImuData imuData;
        if (hexapod.getImuData(imuData))
        {
            double roll = atan2(imuData.getAccelY(), imuData.getAccelZ());
            double pitch = atan2(-imuData.getAccelX(), sqrt(imuData.getAccelY() * imuData.getAccelY() + imuData.getAccelZ() * imuData.getAccelZ()));

            std::vector<double> angular_velocity = {imuData.getGyroX(), imuData.getGyroY(), imuData.getGyroZ()};
            controller.updateBalanceFeedback(roll, pitch, angular_velocity);
        }

        // Update CPG controller
        if (!controller.update(0.02))
        {
            common::ErrorReporter::reportError("Balance-Test", "Walking Update", "Failed to update controller");
            break;
        }

        // Show controller state
        auto state = controller.getControllerState();
        std::cout << "\rWalking | Gait: " << state.current_gait
                  << " | Freq: " << common::StringUtils::formatNumber(state.current_frequency, 2) << "Hz"
                  << " | Vel: " << common::StringUtils::formatNumber(state.actual_linear_velocity, 2) << "m/s" << std::flush;

        // Check for user abort
        char key;
        if (common::TerminalManager::readChar(key) && (key == 'q' || key == 27))
        {
            std::cout << "\nWalking test aborted by user" << std::endl;
            break;
        }

        common::sleepMs(20);
    }

    // Stop walking
    controller.stopLocomotion();
    std::cout << "\nAutonomous walking test completed" << std::endl;
    return true;
}

// Show CPG controller status
void show_controller_status(cpg::Controller &controller)
{
    auto state = controller.getControllerState();

    std::cout << "\nCPG Controller Status:" << std::endl;
    std::cout << "=====================" << std::endl;
    std::cout << "  Active: " << (state.is_active ? "Yes" : "No") << std::endl;
    std::cout << "  Walking: " << (state.is_walking ? "Yes" : "No") << std::endl;
    std::cout << "  Current Gait: " << state.current_gait << std::endl;
    std::cout << "  Frequency: " << common::StringUtils::formatNumber(state.current_frequency, 2) << " Hz" << std::endl;
    std::cout << "  Linear Velocity: " << common::StringUtils::formatNumber(state.actual_linear_velocity, 2) << " m/s" << std::endl;
    std::cout << "  Angular Velocity: " << common::StringUtils::formatNumber(state.actual_angular_velocity, 2) << " rad/s" << std::endl;
    std::cout << "  Stability Margin: " << common::StringUtils::formatNumber(state.stability_margin, 3) << std::endl;
    std::cout << "  Energy Consumption: " << common::StringUtils::formatNumber(state.energy_consumption, 2) << " W" << std::endl;
    std::cout << std::endl;
}

int main()
{
    // Setup graceful shutdown handling using common utilities
    common::SignalManager::setupGracefulShutdown(running, signalHandler);

    // Setup terminal for immediate input using common utilities
    if (!common::TerminalManager::setupImmediate())
    {
        common::ErrorReporter::reportWarning("Balance-Test", "Failed to setup immediate terminal input");
    }

    std::cout << "CPG Balance Test Program" << std::endl;
    std::cout << "========================" << std::endl;
    std::cout << "Testing hexapod CPG balance and IMU systems" << std::endl;

    // Initialize hexapod
    hexapod::Hexapod hexapod;
    if (!hexapod.init())
    {
        common::ErrorReporter::reportError("Balance-Test", "Initialization", hexapod.getLastErrorMessage());
        common::TerminalManager::restore();
        return 1;
    }

    // Initialize CPG controller
    cpg::Controller controller;
    if (!controller.initialize())
    {
        common::ErrorReporter::reportError("Balance-Test", "Controller Init", "Failed to initialize CPG controller");
        common::TerminalManager::restore();
        return 1;
    }

    // Center all legs for safety
    std::cout << "Centering all legs for safety..." << std::endl;
    if (!hexapod.centerAll())
    {
        common::ErrorReporter::reportWarning("Balance-Test", "Failed to center legs");
    }

    // Show main menu
    print_menu();

    // Main interactive loop
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

                while (running.load())
                {
                    // Read IMU and update balance feedback
                    hexapod::ImuData imuData;
                    if (hexapod.getImuData(imuData))
                    {
                        double roll = atan2(imuData.getAccelY(), imuData.getAccelZ());
                        double pitch = atan2(-imuData.getAccelX(), sqrt(imuData.getAccelY() * imuData.getAccelY() + imuData.getAccelZ() * imuData.getAccelZ()));

                        std::vector<double> angular_velocity = {imuData.getGyroX(), imuData.getGyroY(), imuData.getGyroZ()};
                        controller.updateBalanceFeedback(roll, pitch, angular_velocity);
                    }

                    if (!controller.update(0.02)) // 20ms update
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

                std::cout << "Continuous balance mode stopped" << std::endl;
                break;
            }

            case '4': // Manual tilt test with IMU
            {
                std::cout << "Manual tilt test - tilt the robot and observe CPG response" << std::endl;
                auto start_time = common::getCurrentTime();
                auto end_time = start_time + 10.0;

                while (running.load() && common::getCurrentTime() < end_time)
                {
                    hexapod::ImuData imuData;
                    if (hexapod.getImuData(imuData))
                    {
                        double roll = atan2(imuData.getAccelY(), imuData.getAccelZ()) * 180.0 / M_PI;
                        double pitch = atan2(-imuData.getAccelX(), sqrt(imuData.getAccelY() * imuData.getAccelY() + imuData.getAccelZ() * imuData.getAccelZ())) * 180.0 / M_PI;

                        std::cout << "\rRoll: " << std::setw(6) << common::StringUtils::formatNumber(roll, 1) << "°"
                                  << " Pitch: " << std::setw(6) << common::StringUtils::formatNumber(pitch, 1) << "°" << std::flush;

                        std::vector<double> angular_velocity = {imuData.getGyroX(), imuData.getGyroY(), imuData.getGyroZ()};
                        controller.updateBalanceFeedback(roll * M_PI / 180.0, pitch * M_PI / 180.0, angular_velocity);
                    }

                    controller.update(0.05);
                    common::sleepMs(50);
                }
                std::cout << "\nManual tilt test completed" << std::endl;
                break;
            }

            case '5': // Autonomous walking test
                test_autonomous_walking(controller, 20.0);
                break;

            case '6': // Obstacle avoidance test
                std::cout << "Obstacle avoidance test - not yet implemented for CPG" << std::endl;
                break;

            case 's': // Show status
                show_controller_status(controller);
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
                std::cout << "Exiting CPG balance test program..." << std::endl;
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

    // Final safety - center legs before exit
    std::cout << "Centering legs for safe shutdown..." << std::endl;
    hexapod.centerAll();

    // Restore terminal settings
    common::TerminalManager::restore();

    std::cout << "CPG balance test program completed." << std::endl;
    return 0;
}
