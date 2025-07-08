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
#include "common.hpp"

// Global running flag for signal handling
static std::atomic<bool> running(true);

// Signal handler using common utilities
void signalHandler(int signal)
{
    running.store(false);
    common::ErrorReporter::reportInfo("Servo-Test", "Received termination signal " + std::to_string(signal));
}

// Display test menu
static void print_menu()
{
    std::cout << "\nServo Test Program - Commands\n"
              << "=============================\n"
              << "  1: Test individual servo\n"
              << "  2: Test all servos sequentially\n"
              << "  3: Sweep test (smooth movement)\n"
              << "  4: Center all servos\n"
              << "  5: Leg coordination test\n"
              << "  6: Range of motion test\n"
              << "  7: Load test (stress test)\n"
              << "  8: Calibration test\n"
              << "  s: Show servo status\n"
              << "  h: Show this help\n"
              << "  q: Quit\n\n";
}

// Test individual servo movement
bool test_individual_servo(Hexapod &hexapod, int leg_num, int joint_type)
{
    common::ErrorReporter::reportInfo("Servo-Test", "Testing leg " + std::to_string(leg_num) + 
        " joint " + std::to_string(joint_type));

    LegPosition position;
    position.leg_num = leg_num;
    
    // Get current position
    if (!hexapod.getLegPosition(leg_num, position))
    {
        common::ErrorReporter::reportError("Servo-Test", "Get Position", hexapod.getLastErrorMessage());
        return false;
    }

    // Test movement in small increments
    std::vector<int> test_angles = {-30, -15, 0, 15, 30, 0}; // Return to center
    
    for (int angle : test_angles)
    {
        // Set the specific joint angle
        switch (joint_type)
        {
        case 0: // Hip
            position.setHip(angle);
            break;
        case 1: // Knee
            position.setKnee(angle);
            break;
        case 2: // Ankle
            position.setAnkle(angle);
            break;
        default:
            common::ErrorReporter::reportError("Servo-Test", "Joint Type", "Invalid joint type");
            return false;
        }

        std::cout << "Moving to angle: " << angle << "°..." << std::flush;
        
        if (hexapod.setLegPosition(leg_num, position))
        {
            std::cout << " OK" << std::endl;
            common::sleepMs(500); // Wait for movement to complete
        }
        else
        {
            std::cout << " FAILED" << std::endl;
            common::ErrorReporter::reportError("Servo-Test", "Set Position", hexapod.getLastErrorMessage());
            return false;
        }
    }

    return true;
}

// Test all servos in sequence
bool test_all_servos(Hexapod &hexapod)
{
    common::ErrorReporter::reportInfo("Servo-Test", "Testing all servos sequentially");
    
    common::PerformanceMonitor perfMonitor;
    int successful_tests = 0;
    int total_tests = hexapod::Config::NUM_LEGS * hexapod::Config::SERVOS_PER_LEG;

    for (int leg = 0; leg < hexapod::Config::NUM_LEGS; leg++)
    {
        for (int joint = 0; joint < hexapod::Config::SERVOS_PER_LEG; joint++)
        {
            std::cout << "Testing Leg " << leg << ", Joint " << joint << " ";
            
            const char* joint_names[] = {"(Hip)", "(Knee)", "(Ankle)"};
            std::cout << joint_names[joint] << "..." << std::endl;

            perfMonitor.startFrame();
            bool success = test_individual_servo(hexapod, leg, joint);
            perfMonitor.endFrame();

            if (success)
            {
                successful_tests++;
                std::cout << "✓ Test passed in " << 
                    common::StringUtils::formatNumber(perfMonitor.getAverageFrameTime()) << "ms" << std::endl;
            }
            else
            {
                std::cout << "✗ Test failed" << std::endl;
            }

            // Check if user wants to abort
            char key;
            if (common::TerminalManager::readChar(key) && (key == 'q' || key == 27))
            {
                std::cout << "Test aborted by user" << std::endl;
                break;
            }
        }
    }

    std::cout << "\nTest Summary: " << successful_tests << "/" << total_tests 
              << " servos passed (" << 
              common::StringUtils::formatNumber((successful_tests * 100.0) / total_tests, 1) 
              << "%)" << std::endl;

    return successful_tests == total_tests;
}

// Smooth sweep test
bool sweep_test(Hexapod &hexapod)
{
    common::ErrorReporter::reportInfo("Servo-Test", "Running smooth sweep test");

    // Test smooth movement across range
    const int steps = 20;
    const int delay_ms = 100;
    
    for (int leg = 0; leg < hexapod::Config::NUM_LEGS; leg++)
    {
        std::cout << "Sweeping leg " << leg << "..." << std::endl;
        
        LegPosition position;
        position.leg_num = leg;
        
        // Sweep through range of motion
        for (int step = 0; step <= steps; step++)
        {
            // Calculate smooth sine wave motion
            double t = (double)step / steps * 2.0 * common::Constants::PI;
            int hip_angle = (int)(30.0 * sin(t));
            int knee_angle = (int)(20.0 * sin(t + common::Constants::PI / 3));
            int ankle_angle = (int)(25.0 * sin(t + 2.0 * common::Constants::PI / 3));

            position.setHip(hip_angle);
            position.setKnee(knee_angle);
            position.setAnkle(ankle_angle);

            if (!hexapod.setLegPosition(leg, position))
            {
                common::ErrorReporter::reportError("Servo-Test", "Sweep Movement", 
                    hexapod.getLastErrorMessage());
                return false;
            }

            common::sleepMs(delay_ms);

            // Show progress
            if (step % 5 == 0)
            {
                common::ProgressBar::display(step, steps, 20, "Progress");
            }

            // Check for abort
            char key;
            if (common::TerminalManager::readChar(key) && (key == 'q' || key == 27))
            {
                std::cout << "\nSweep test aborted by user" << std::endl;
                return false;
            }
        }
        
        common::ProgressBar::clear();
        std::cout << "Leg " << leg << " sweep completed" << std::endl;
    }

    return true;
}

// Range of motion test
bool range_of_motion_test(Hexapod &hexapod)
{
    common::ErrorReporter::reportInfo("Servo-Test", "Testing range of motion limits");

    struct JointLimits {
        const char* name;
        int min_angle;
        int max_angle;
    };

    JointLimits limits[] = {
        {"Hip", hexapod::AngleLimits::HIP_MIN, hexapod::AngleLimits::HIP_MAX},
        {"Knee", hexapod::AngleLimits::KNEE_MIN, hexapod::AngleLimits::KNEE_MAX},
        {"Ankle", hexapod::AngleLimits::ANKLE_MIN, hexapod::AngleLimits::ANKLE_MAX}
    };

    for (int leg = 0; leg < hexapod::Config::NUM_LEGS; leg++)
    {
        std::cout << "Testing range of motion for leg " << leg << std::endl;

        for (int joint = 0; joint < 3; joint++)
        {
            std::cout << "  " << limits[joint].name << " joint: ";

            LegPosition position;
            position.leg_num = leg;

            // Test minimum position
            switch (joint)
            {
            case 0: position.setHip(limits[joint].min_angle); break;
            case 1: position.setKnee(limits[joint].min_angle); break;
            case 2: position.setAnkle(limits[joint].min_angle); break;
            }

            if (hexapod.setLegPosition(leg, position))
            {
                common::sleepMs(300);
                std::cout << "Min(" << limits[joint].min_angle << "°) ";

                // Test maximum position
                switch (joint)
                {
                case 0: position.setHip(limits[joint].max_angle); break;
                case 1: position.setKnee(limits[joint].max_angle); break;
                case 2: position.setAnkle(limits[joint].max_angle); break;
                }

                if (hexapod.setLegPosition(leg, position))
                {
                    common::sleepMs(300);
                    std::cout << "Max(" << limits[joint].max_angle << "°) ✓" << std::endl;

                    // Return to center
                    switch (joint)
                    {
                    case 0: position.setHip(0); break;
                    case 1: position.setKnee(0); break;
                    case 2: position.setAnkle(0); break;
                    }
                    hexapod.setLegPosition(leg, position);
                    common::sleepMs(200);
                }
                else
                {
                    std::cout << "Max FAILED ✗" << std::endl;
                }
            }
            else
            {
                std::cout << "Min FAILED ✗" << std::endl;
            }
        }
    }

    return true;
}

// Load test - stress test servos
bool load_test(Hexapod &hexapod)
{
    common::ErrorReporter::reportInfo("Servo-Test", "Running servo load test");

    const int test_duration_seconds = 30;
    const int movements_per_second = 5;
    const int total_movements = test_duration_seconds * movements_per_second;

    std::cout << "Running " << test_duration_seconds << " second load test..." << std::endl;
    std::cout << "Press 'q' to abort early" << std::endl;

    common::PerformanceMonitor perfMonitor;
    int successful_movements = 0;

    auto start_time = common::getCurrentTimeMs();

    for (int movement = 0; movement < total_movements && running.load(); movement++)
    {
        perfMonitor.startFrame();

        // Generate random leg and position
        int leg = movement % hexapod::Config::NUM_LEGS;
        LegPosition position;
        position.leg_num = leg;

        // Generate random angles within safe range
        position.setHip((movement * 7) % 61 - 30);     // -30 to +30
        position.setKnee((movement * 11) % 41 - 20);   // -20 to +20
        position.setAnkle((movement * 13) % 51 - 25);  // -25 to +25

        if (hexapod.setLegPosition(leg, position))
        {
            successful_movements++;
        }

        perfMonitor.endFrame();

        // Show progress every 10 movements
        if (movement % 10 == 0)
        {
            common::ProgressBar::display(movement, total_movements, 40, "Load test");
        }

        // Check for user abort
        char key;
        if (common::TerminalManager::readChar(key) && (key == 'q' || key == 27))
        {
            std::cout << "\nLoad test aborted by user" << std::endl;
            break;
        }

        // Maintain target rate
        common::sleepMs(1000 / movements_per_second);
    }

    common::ProgressBar::clear();

    auto end_time = common::getCurrentTimeMs();
    double actual_duration = (end_time - start_time) / 1000.0;

    perfMonitor.printReport("Load test ");

    std::cout << "Load test completed:" << std::endl;
    std::cout << "  Duration: " << common::StringUtils::formatDuration(actual_duration) << std::endl;
    std::cout << "  Successful movements: " << successful_movements << "/" << total_movements 
              << " (" << common::StringUtils::formatNumber((successful_movements * 100.0) / total_movements, 1) 
              << "%)" << std::endl;

    return successful_movements == total_movements;
}

// Show servo status
void show_servo_status(Hexapod &hexapod)
{
    std::cout << "\nServo Status Report" << std::endl;
    std::cout << "===================" << std::endl;

    for (int leg = 0; leg < hexapod::Config::NUM_LEGS; leg++)
    {
        LegPosition position;
        if (hexapod.getLegPosition(leg, position))
        {
            std::cout << "Leg " << leg << ": Hip=" << std::setw(4) << position.getHip()
                      << "° Knee=" << std::setw(4) << position.getKnee()
                      << "° Ankle=" << std::setw(4) << position.getAnkle() << "°" << std::endl;
        }
        else
        {
            std::cout << "Leg " << leg << ": ERROR - " << hexapod.getLastErrorMessage() << std::endl;
        }
    }
    std::cout << std::endl;
}

int main()
{
    // Setup graceful shutdown handling using common utilities
    common::SignalManager::setupGracefulShutdown(running, signalHandler);

    // Initialize performance monitoring
    common::PerformanceMonitor perfMonitor;

    // Setup terminal for immediate input using common utilities
    if (!common::TerminalManager::setupImmediate()) {
        common::ErrorReporter::reportWarning("Servo-Test", "Failed to setup immediate terminal input");
    }

    std::cout << "Servo Test Program" << std::endl;
    std::cout << "==================" << std::endl;
    std::cout << "Testing hexapod servo functionality" << std::endl;

    // Initialize hexapod
    Hexapod hexapod;
    
    perfMonitor.startFrame();
    if (!hexapod.init())
    {
        common::ErrorReporter::reportError("Servo-Test", "Initialization", hexapod.getLastErrorMessage());
        common::TerminalManager::restore();
        return 1;
    }
    perfMonitor.endFrame();

    common::ErrorReporter::reportInfo("Servo-Test", "Hexapod initialized successfully in " +
        common::StringUtils::formatNumber(perfMonitor.getAverageFrameTime()) + "ms");

    // Center all servos for safety
    std::cout << "Centering all servos for safety..." << std::endl;
    if (!hexapod.centerAll())
    {
        common::ErrorReporter::reportWarning("Servo-Test", "Failed to center servos: " + hexapod.getLastErrorMessage());
    }

    print_menu();

    // Main test loop
    while (running.load())
    {
        std::cout << "Enter command: ";
        char command;
        
        if (common::TerminalManager::readChar(command))
        {
            switch (command)
            {
            case '1': // Individual servo test
            {
                int leg, joint;
                std::cout << "\nEnter leg number (0-" << (hexapod::Config::NUM_LEGS-1) << "): ";
                std::cin >> leg;
                std::cout << "Enter joint (0=Hip, 1=Knee, 2=Ankle): ";
                std::cin >> joint;

                if (common::Validator::validateLegNumber(leg) && joint >= 0 && joint < 3)
                {
                    test_individual_servo(hexapod, leg, joint);
                }
                else
                {
                    common::ErrorReporter::reportError("Servo-Test", "Input", "Invalid leg or joint number");
                }
                break;
            }

            case '2': // Test all servos
                test_all_servos(hexapod);
                break;

            case '3': // Sweep test
                sweep_test(hexapod);
                break;

            case '4': // Center all
                std::cout << "Centering all servos..." << std::endl;
                if (hexapod.centerAll())
                {
                    std::cout << "All servos centered successfully" << std::endl;
                }
                else
                {
                    common::ErrorReporter::reportError("Servo-Test", "Center", hexapod.getLastErrorMessage());
                }
                break;

            case '5': // Leg coordination test
                std::cout << "Leg coordination test not yet implemented" << std::endl;
                break;

            case '6': // Range of motion test
                range_of_motion_test(hexapod);
                break;

            case '7': // Load test
                load_test(hexapod);
                break;

            case '8': // Calibration test
                std::cout << "Calibration test not yet implemented" << std::endl;
                break;

            case 's': // Show status
                show_servo_status(hexapod);
                break;

            case 'h': // Help
                print_menu();
                break;

            case 'q': // Quit
                std::cout << "Exiting servo test program..." << std::endl;
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

    // Final safety - center all servos before exit
    std::cout << "Centering servos for safe shutdown..." << std::endl;
    hexapod.centerAll();

    // Restore terminal settings
    common::TerminalManager::restore();

    std::cout << "Servo test program completed." << std::endl;
    return 0;
}
