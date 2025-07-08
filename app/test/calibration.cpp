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
#include <algorithm>
#include <fstream>
#include "hexapod.hpp"
#include "calibration.hpp"
#include "common.hpp"

// Global running flag for signal handling
static std::atomic<bool> running(true);

// Signal handler using common utilities
void signalHandler(int signal)
{
    running.store(false);
    common::ErrorReporter::reportInfo("Calibration-Test", "Received termination signal " + std::to_string(signal));
}

// Display test menu
static void print_menu()
{
    std::cout << "\nCalibration Test Program - Commands\n"
              << "===================================\n"
              << "  1: Load calibration from file\n"
              << "  2: Save calibration to file\n"
              << "  3: Create new calibration\n"
              << "  4: Edit calibration values\n"
              << "  5: Apply current calibration\n"
              << "  6: Reset to default calibration\n"
              << "  7: Validate calibration\n"
              << "  8: Test leg movement with calibration\n"
              << "  9: Interactive calibration wizard\n"
              << "  s: Show current calibration\n"
              << "  c: Center all legs\n"
              << "  h: Show this help\n"
              << "  q: Quit\n\n";
}

// Display current calibration values
void show_calibration(const std::vector<calibration::Calibration> &calibrations)
{
    std::cout << "\nCurrent Calibration Values:\n";
    std::cout << "===========================\n";
    std::cout << "Leg | Hip  | Knee | Ankle\n";
    std::cout << "----+------+------+------\n";

    for (const auto &cal : calibrations)
    {
        std::cout << std::setw(3) << static_cast<int>(cal.leg_num) << " | "
                  << std::setw(4) << cal.hip_offset << " | "
                  << std::setw(4) << cal.knee_offset << " | "
                  << std::setw(5) << cal.ankle_offset << std::endl;
    }
    std::cout << std::endl;
}

// Interactive calibration wizard
bool run_calibration_wizard(hexapod::Hexapod &hexapod, std::vector<calibration::Calibration> &calibrations)
{
    common::ErrorReporter::reportInfo("Calibration-Test", "Starting interactive calibration wizard");

    std::cout << "\nInteractive Calibration Wizard\n";
    std::cout << "==============================\n";
    std::cout << "This wizard will help you calibrate each leg individually.\n";
    std::cout << "For each leg, you'll be able to adjust joint offsets until\n";
    std::cout << "the leg is properly aligned.\n\n";

    for (int leg = 0; leg < hexapod::Config::NUM_LEGS; leg++)
    {
        std::cout << "Calibrating Leg " << leg << "...\n";
        std::cout << "Current position will be set to neutral (0,0,0)\n";

        // Set leg to neutral position first
        hexapod::LegPosition neutral;
        neutral.leg_num = leg;
        neutral.setHip(0);
        neutral.setKnee(0);
        neutral.setAnkle(0);

        if (!hexapod.setLegPosition(leg, neutral))
        {
            common::ErrorReporter::reportError("Calibration-Test", "Leg Movement",
                                               "Failed to move leg " + std::to_string(leg) + " to neutral position");
            continue;
        }

        // Wait for movement to complete
        common::sleepMs(1000);

        auto &cal = calibrations[leg];
        cal.leg_num = leg;

        std::cout << "Observe leg " << leg << " and enter offset adjustments:\n";

        // Hip calibration
        std::cout << "Hip offset (current: " << cal.hip_offset << "): ";
        int new_hip;
        if (std::cin >> new_hip)
        {
            cal.hip_offset = common::Validator::clamp(new_hip, -30, 30);
        }

        // Knee calibration
        std::cout << "Knee offset (current: " << cal.knee_offset << "): ";
        int new_knee;
        if (std::cin >> new_knee)
        {
            cal.knee_offset = common::Validator::clamp(new_knee, -30, 30);
        }

        // Ankle calibration
        std::cout << "Ankle offset (current: " << cal.ankle_offset << "): ";
        int new_ankle;
        if (std::cin >> new_ankle)
        {
            cal.ankle_offset = common::Validator::clamp(new_ankle, -30, 30);
        }

        // Apply calibration and test
        if (hexapod.setCalibration(leg, cal.hip_offset, cal.knee_offset, cal.ankle_offset))
        {
            std::cout << "Applied calibration to leg " << leg << std::endl;

            // Test the calibrated leg
            if (hexapod.setLegPosition(leg, neutral))
            {
                std::cout << "Test movement successful\n";
            }
            else
            {
                common::ErrorReporter::reportWarning("Calibration-Test",
                                                     "Test movement failed for leg " + std::to_string(leg));
            }
        }
        else
        {
            common::ErrorReporter::reportError("Calibration-Test", "Calibration Apply",
                                               "Failed to apply calibration to leg " + std::to_string(leg));
        }

        std::cout << "Press Enter to continue to next leg...";
        std::cin.ignore();
        std::cin.get();
    }

    std::cout << "Calibration wizard completed!\n";
    return true;
}

// Test leg movement with current calibration
bool test_leg_movement(hexapod::Hexapod &hexapod, int leg_num)
{
    common::ErrorReporter::reportInfo("Calibration-Test", "Testing movement for leg " + std::to_string(leg_num));

    std::vector<hexapod::LegPosition> test_positions;

    // Create test positions
    for (int i = 0; i < 5; i++)
    {
        hexapod::LegPosition pos;
        pos.leg_num = leg_num;

        switch (i)
        {
        case 0: // Neutral
            pos.setHip(0);
            pos.setKnee(0);
            pos.setAnkle(0);
            break;
        case 1: // Hip forward
            pos.setHip(30);
            pos.setKnee(0);
            pos.setAnkle(0);
            break;
        case 2: // Hip backward
            pos.setHip(-30);
            pos.setKnee(0);
            pos.setAnkle(0);
            break;
        case 3: // Knee bend
            pos.setHip(0);
            pos.setKnee(45);
            pos.setAnkle(0);
            break;
        case 4: // Ankle flex
            pos.setHip(0);
            pos.setKnee(0);
            pos.setAnkle(30);
            break;
        }

        test_positions.push_back(pos);
    }

    common::PerformanceMonitor perfMonitor;
    bool success = true;

    for (size_t i = 0; i < test_positions.size(); i++)
    {
        std::cout << "Moving to test position " << (i + 1) << "..." << std::flush;

        perfMonitor.startFrame();
        bool result = hexapod.setLegPosition(leg_num, test_positions[i]);
        perfMonitor.endFrame();

        if (result)
        {
            std::cout << " OK (" << common::StringUtils::formatNumber(perfMonitor.getAverageFrameTime()) << "ms)" << std::endl;
            common::sleepMs(800); // Wait for movement
        }
        else
        {
            std::cout << " FAILED" << std::endl;
            common::ErrorReporter::reportError("Calibration-Test", "Movement Test",
                                               hexapod.getLastErrorMessage());
            success = false;
        }

        // Check for user abort
        char key;
        if (common::TerminalManager::readChar(key) && (key == 'q' || key == 27))
        {
            std::cout << "\nTest aborted by user" << std::endl;
            return false;
        }
    }

    // Return to neutral
    std::cout << "Returning to neutral position..." << std::endl;
    hexapod::LegPosition neutral;
    neutral.leg_num = leg_num;
    hexapod.setLegPosition(leg_num, neutral);

    return success;
}

// Edit calibration values for a specific leg
void edit_calibration_values(std::vector<calibration::Calibration> &calibrations)
{
    int leg_num;
    std::cout << "Enter leg number (0-" << (hexapod::Config::NUM_LEGS - 1) << "): ";

    if (!(std::cin >> leg_num) || !common::Validator::validateLegNumber(leg_num))
    {
        common::ErrorReporter::reportError("Calibration-Test", "Input", "Invalid leg number");
        return;
    }

    auto &cal = calibrations[leg_num];

    std::cout << "Current calibration for leg " << leg_num << ":\n";
    std::cout << "  Hip: " << cal.hip_offset << "°\n";
    std::cout << "  Knee: " << cal.knee_offset << "°\n";
    std::cout << "  Ankle: " << cal.ankle_offset << "°\n\n";

    int new_value;

    std::cout << "New hip offset (-30 to 30, current " << cal.hip_offset << "): ";
    if (std::cin >> new_value)
    {
        cal.hip_offset = common::Validator::clamp(new_value, -30, 30);
    }

    std::cout << "New knee offset (-30 to 30, current " << cal.knee_offset << "): ";
    if (std::cin >> new_value)
    {
        cal.knee_offset = common::Validator::clamp(new_value, -30, 30);
    }

    std::cout << "New ankle offset (-30 to 30, current " << cal.ankle_offset << "): ";
    if (std::cin >> new_value)
    {
        cal.ankle_offset = common::Validator::clamp(new_value, -30, 30);
    }

    std::cout << "Calibration updated for leg " << leg_num << std::endl;
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
        common::ErrorReporter::reportWarning("Calibration-Test", "Failed to setup immediate terminal input");
    }

    std::cout << "Calibration Test Program" << std::endl;
    std::cout << "========================" << std::endl;
    std::cout << "Testing hexapod calibration system" << std::endl;

    // Initialize hexapod
    hexapod::Hexapod hexapod;

    perfMonitor.startFrame();
    if (!hexapod.init())
    {
        common::ErrorReporter::reportError("Calibration-Test", "Initialization", hexapod.getLastErrorMessage());
        common::TerminalManager::restore();
        return 1;
    }
    perfMonitor.endFrame();

    common::ErrorReporter::reportInfo("Calibration-Test", "Hexapod initialized successfully in " +
                                                              common::StringUtils::formatNumber(perfMonitor.getAverageFrameTime()) + "ms");

    // Initialize calibration data
    std::vector<calibration::Calibration> calibrations = calibration::CalibrationManager::getDefaultCalibration();
    std::string current_filename;

    // Try to load existing calibration
    if (calibration::CalibrationManager::loadCalibration(calibrations))
    {
        common::ErrorReporter::reportInfo("Calibration-Test", "Loaded existing calibration from default file");
        current_filename = calibration::CalibrationManager::getDefaultCalibrationPath();
    }
    else
    {
        common::ErrorReporter::reportInfo("Calibration-Test", "Using default calibration (all zeros)");
    }

    // Center all legs for safety
    std::cout << "Centering all legs for safety..." << std::endl;
    if (!hexapod.centerAll())
    {
        common::ErrorReporter::reportWarning("Calibration-Test", "Failed to center legs");
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
            case '1': // Load calibration
            {
                std::cout << "\nEnter filename (or press Enter for default): ";
                std::string filename;
                std::getline(std::cin, filename);

                if (calibration::CalibrationManager::loadCalibration(calibrations, filename))
                {
                    current_filename = filename.empty() ? calibration::CalibrationManager::getDefaultCalibrationPath() : filename;
                    std::cout << "Calibration loaded successfully from: " << current_filename << std::endl;
                }
                else
                {
                    common::ErrorReporter::reportError("Calibration-Test", "Load", "Failed to load calibration file");
                }
                break;
            }

            case '2': // Save calibration
            {
                std::cout << "\nEnter filename (or press Enter for default): ";
                std::string filename;
                std::getline(std::cin, filename);

                if (calibration::CalibrationManager::saveCalibration(calibrations, filename))
                {
                    current_filename = filename.empty() ? calibration::CalibrationManager::getDefaultCalibrationPath() : filename;
                    std::cout << "Calibration saved successfully to: " << current_filename << std::endl;
                }
                else
                {
                    common::ErrorReporter::reportError("Calibration-Test", "Save", "Failed to save calibration file");
                }
                break;
            }

            case '3': // Create new calibration
                calibrations = calibration::CalibrationManager::getDefaultCalibration();
                std::cout << "Created new default calibration (all zeros)" << std::endl;
                current_filename.clear();
                break;

            case '4': // Edit calibration values
                edit_calibration_values(calibrations);
                break;

            case '5': // Apply calibration
            {
                std::cout << "Applying calibration to hexapod..." << std::endl;
                common::ProgressBar::display(0, hexapod::Config::NUM_LEGS, 30, "Applying");

                if (calibration::CalibrationManager::applyCalibration(hexapod, calibrations))
                {
                    common::ProgressBar::display(hexapod::Config::NUM_LEGS, hexapod::Config::NUM_LEGS, 30, "Complete");
                    std::cout << "\nCalibration applied successfully" << std::endl;
                }
                else
                {
                    common::ProgressBar::clear();
                    common::ErrorReporter::reportError("Calibration-Test", "Apply", "Failed to apply calibration");
                }
                break;
            }

            case '6': // Reset to default
                calibrations = calibration::CalibrationManager::getDefaultCalibration();
                if (calibration::CalibrationManager::applyCalibration(hexapod, calibrations))
                {
                    std::cout << "Reset to default calibration and applied" << std::endl;
                }
                else
                {
                    common::ErrorReporter::reportError("Calibration-Test", "Reset", "Failed to apply default calibration");
                }
                break;

            case '7': // Validate calibration
                if (calibration::CalibrationManager::validateCalibration(calibrations))
                {
                    std::cout << "✓ Calibration validation passed" << std::endl;
                }
                else
                {
                    std::cout << "✗ Calibration validation failed" << std::endl;
                }
                break;

            case '8': // Test leg movement
            {
                int leg_num;
                std::cout << "Enter leg number to test (0-" << (hexapod::Config::NUM_LEGS - 1) << "): ";
                if (std::cin >> leg_num && common::Validator::validateLegNumber(leg_num))
                {
                    test_leg_movement(hexapod, leg_num);
                }
                else
                {
                    common::ErrorReporter::reportError("Calibration-Test", "Input", "Invalid leg number");
                }
                break;
            }

            case '9': // Interactive calibration wizard
                run_calibration_wizard(hexapod, calibrations);
                break;

            case 's': // Show calibration
                show_calibration(calibrations);
                break;

            case 'c': // Center legs
                std::cout << "Centering all legs..." << std::endl;
                if (hexapod.centerAll())
                {
                    std::cout << "All legs centered successfully" << std::endl;
                }
                else
                {
                    common::ErrorReporter::reportError("Calibration-Test", "Center", hexapod.getLastErrorMessage());
                }
                break;

            case 'h': // Help
                print_menu();
                break;

            case 'q': // Quit
                std::cout << "Exiting calibration test program..." << std::endl;
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

    // Final safety - center all legs before exit
    std::cout << "Centering legs for safe shutdown..." << std::endl;
    hexapod.centerAll();

    // Final performance report
    std::cout << "\nCalibration Test Summary:" << std::endl;
    if (!current_filename.empty())
    {
        std::cout << "  Last file: " << current_filename << std::endl;
    }
    show_calibration(calibrations);

    // Restore terminal settings
    common::TerminalManager::restore();

    std::cout << "Calibration test program completed." << std::endl;
    return 0;
}
