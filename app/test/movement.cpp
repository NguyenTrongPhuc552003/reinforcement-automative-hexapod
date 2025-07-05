#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <atomic>
#include <cmath>
#include "hexapod.hpp"
#include "gait.hpp"
#include "kinematics.hpp"
#include "common.hpp"

// Global running flag for signal handling
static std::atomic<bool> running(true);

// Signal handler using common utilities
void signalHandler(int signal)
{
    running.store(false);
    common::ErrorReporter::reportInfo("Movement-Test", "Received termination signal " + std::to_string(signal));
}

// Display test menu
static void print_menu()
{
    std::cout << "\nMovement Test Program - Commands\n"
              << "================================\n"
              << "  1: Basic forward movement\n"
              << "  2: Backward movement\n"
              << "  3: Left turn movement\n"
              << "  4: Right turn movement\n"
              << "  5: Figure-8 pattern\n"
              << "  6: Square pattern\n"
              << "  7: Circle pattern\n"
              << "  8: Custom speed test\n"
              << "  9: Gait comparison test\n"
              << "  0: Stress test (continuous movement)\n"
              << "  g: Toggle gait type\n"
              << "  +/-: Adjust speed\n"
              << "  c: Center all legs\n"
              << "  s: Show status\n"
              << "  h: Show this help\n"
              << "  q: Quit\n\n";
}

// Test basic forward movement
bool test_forward_movement(Hexapod &hexapod, gait::Gait &gait_controller, double duration, double speed)
{
    // Mark hexapod as potentially unused since we only use gait_controller for movement
    (void)hexapod; // Suppress unused parameter warning

    common::ErrorReporter::reportInfo("Movement-Test", "Testing forward movement for " +
                                                           common::StringUtils::formatDuration(duration) + " at speed " +
                                                           common::StringUtils::formatNumber(speed, 1));

    common::PerformanceMonitor perfMonitor;
    auto start_time = common::getCurrentTime();
    auto end_time = start_time + duration;

    while (running.load() && common::getCurrentTime() < end_time)
    {
        perfMonitor.startFrame();

        double current_time = common::getCurrentTime();
        if (!gait_controller.update(current_time, 0.0, speed)) // 0 degrees = forward
        {
            common::ErrorReporter::reportError("Movement-Test", "Gait Update", "Failed to update gait");
            return false;
        }

        perfMonitor.endFrame();

        // Check for user abort
        char key;
        if (common::TerminalManager::readChar(key) && (key == 'q' || key == 27))
        {
            std::cout << "\nMovement test aborted by user" << std::endl;
            return false;
        }

        // Update progress
        double progress = (current_time - start_time) / duration;
        if (static_cast<int>(progress * 20) % 5 == 0) // Update every 25%
        {
            common::ProgressBar::display(static_cast<int>(progress * 100), 100, 30, "Forward");
        }

        common::sleepMs(50); // 20Hz update rate
    }

    common::ProgressBar::clear();
    perfMonitor.printReport("Forward movement ");
    std::cout << "Forward movement test completed successfully" << std::endl;
    return true;
}

// Test turning movement
bool test_turn_movement(Hexapod &hexapod, gait::Gait &gait_controller, double direction, double duration, double speed)
{
    // Mark hexapod as potentially unused since we only use gait_controller for movement
    (void)hexapod; // Suppress unused parameter warning

    std::string direction_name = (direction > 0) ? "right" : "left";
    common::ErrorReporter::reportInfo("Movement-Test", "Testing " + direction_name + " turn for " +
                                                           common::StringUtils::formatDuration(duration));

    common::PerformanceMonitor perfMonitor;
    auto start_time = common::getCurrentTime();
    auto end_time = start_time + duration;

    while (running.load() && common::getCurrentTime() < end_time)
    {
        perfMonitor.startFrame();

        double current_time = common::getCurrentTime();
        if (!gait_controller.update(current_time, direction, speed))
        {
            common::ErrorReporter::reportError("Movement-Test", "Gait Update", "Failed to update gait during turn");
            return false;
        }

        perfMonitor.endFrame();

        // Check for user abort
        char key;
        if (common::TerminalManager::readChar(key) && (key == 'q' || key == 27))
        {
            std::cout << "\nTurn test aborted by user" << std::endl;
            return false;
        }

        common::sleepMs(50);
    }

    perfMonitor.printReport(direction_name + " turn ");
    std::cout << direction_name << " turn test completed successfully" << std::endl;
    return true;
}

// Test figure-8 pattern
bool test_figure_eight(Hexapod &hexapod, gait::Gait &gait_controller, double speed)
{
    // Mark hexapod as potentially unused since we only use gait_controller for movement
    (void)hexapod; // Suppress unused parameter warning

    common::ErrorReporter::reportInfo("Movement-Test", "Testing figure-8 pattern");

    const double total_time = 16.0; // 8 seconds per circle
    const double half_time = total_time / 2.0;

    common::PerformanceMonitor perfMonitor;
    auto start_time = common::getCurrentTime();
    auto end_time = start_time + total_time;

    std::cout << "Executing figure-8 pattern (16 seconds)..." << std::endl;

    while (running.load() && common::getCurrentTime() < end_time)
    {
        perfMonitor.startFrame();

        double current_time = common::getCurrentTime();
        double elapsed = current_time - start_time;

        // First half: turn right, second half: turn left
        double direction = (elapsed < half_time) ? 90.0 : -90.0;

        if (!gait_controller.update(current_time, direction, speed))
        {
            common::ErrorReporter::reportError("Movement-Test", "Figure-8", "Failed to update gait");
            return false;
        }

        perfMonitor.endFrame();

        // Show progress
        double progress = elapsed / total_time;
        common::ProgressBar::displayPercent(progress, 40, "Figure-8");

        // Check for abort
        char key;
        if (common::TerminalManager::readChar(key) && (key == 'q' || key == 27))
        {
            std::cout << "\nFigure-8 test aborted by user" << std::endl;
            return false;
        }

        common::sleepMs(50);
    }

    common::ProgressBar::clear();
    perfMonitor.printReport("Figure-8 pattern ");
    std::cout << "Figure-8 pattern completed successfully" << std::endl;
    return true;
}

// Test square pattern
bool test_square_pattern(Hexapod &hexapod, gait::Gait &gait_controller, double speed)
{
    // Mark hexapod as potentially unused since we only use gait_controller for movement
    (void)hexapod; // Suppress unused parameter warning

    common::ErrorReporter::reportInfo("Movement-Test", "Testing square movement pattern");

    const double side_duration = 3.0; // 3 seconds per side
    const double turn_duration = 2.0; // 2 seconds per turn

    common::PerformanceMonitor perfMonitor;
    auto start_time = common::getCurrentTime();

    std::cout << "Executing square pattern (20 seconds)..." << std::endl;

    for (int side = 0; side < 4 && running.load(); side++)
    {
        // Move forward for one side
        std::cout << "Side " << (side + 1) << "/4: Moving forward..." << std::endl;
        if (!test_forward_movement(hexapod, gait_controller, side_duration, speed))
        {
            return false;
        }

        // Pause briefly
        gait_controller.centerLegs();
        common::sleepMs(500);

        // Turn 90 degrees right
        std::cout << "Turning right 90°..." << std::endl;
        if (!test_turn_movement(hexapod, gait_controller, 90.0, turn_duration, speed))
        {
            return false;
        }

        // Pause briefly
        gait_controller.centerLegs();
        common::sleepMs(500);
    }

    double total_elapsed = common::getCurrentTime() - start_time;
    std::cout << "Square pattern completed in " << common::StringUtils::formatDuration(total_elapsed) << std::endl;
    return true;
}

// Test circle pattern
bool test_circle_pattern(Hexapod &hexapod, gait::Gait &gait_controller, double speed)
{
    // Mark hexapod as potentially unused since we only use gait_controller for movement
    (void)hexapod; // Suppress unused parameter warning

    common::ErrorReporter::reportInfo("Movement-Test", "Testing circular movement pattern");

    const double total_time = 10.0; // 10 seconds for full circle

    common::PerformanceMonitor perfMonitor;
    auto start_time = common::getCurrentTime();
    auto end_time = start_time + total_time;

    std::cout << "Executing circle pattern (10 seconds)..." << std::endl;

    while (running.load() && common::getCurrentTime() < end_time)
    {
        perfMonitor.startFrame();

        double current_time = common::getCurrentTime();

        // Constant turn rate for smooth circle
        if (!gait_controller.update(current_time, 45.0, speed)) // 45° turn rate
        {
            common::ErrorReporter::reportError("Movement-Test", "Circle", "Failed to update gait");
            return false;
        }

        perfMonitor.endFrame();

        // Show progress
        double progress = (current_time - start_time) / total_time;
        common::ProgressBar::displayPercent(progress, 40, "Circle");

        // Check for abort
        char key;
        if (common::TerminalManager::readChar(key) && (key == 'q' || key == 27))
        {
            std::cout << "\nCircle test aborted by user" << std::endl;
            return false;
        }

        common::sleepMs(50);
    }

    common::ProgressBar::clear();
    perfMonitor.printReport("Circle pattern ");
    std::cout << "Circle pattern completed successfully" << std::endl;
    return true;
}

// Test different gaits
bool test_gait_comparison(Hexapod &hexapod, gait::Gait &gait_controller)
{
    // Mark hexapod as potentially unused since we only use gait_controller for movement
    (void)hexapod; // Suppress unused parameter warning

    common::ErrorReporter::reportInfo("Movement-Test", "Testing gait comparison");

    std::vector<gait::GaitType> gaits = {
        gait::GaitType::TRIPOD,
        gait::GaitType::WAVE,
        gait::GaitType::RIPPLE};

    std::vector<std::string> gait_names = {
        "Tripod", "Wave", "Ripple"};

    const double test_duration = 5.0; // 5 seconds per gait
    const double speed = 0.6;

    for (size_t i = 0; i < gaits.size(); i++)
    {
        std::cout << "Testing " << gait_names[i] << " gait..." << std::endl;

        // Set gait parameters
        gait::GaitParameters params;
        params.type = gaits[i];
        params.stepHeight = 30.0;
        params.stepLength = 60.0;
        params.cycleTime = (gaits[i] == gait::GaitType::TRIPOD) ? 1.0 : 1.5;
        params.dutyFactor = (gaits[i] == gait::GaitType::TRIPOD) ? 0.5 : 0.7;

        if (!gait_controller.setParameters(params))
        {
            common::ErrorReporter::reportError("Movement-Test", "Gait Setup",
                                               "Failed to set parameters for " + gait_names[i] + " gait");
            continue;
        }

        // Test forward movement with this gait
        common::PerformanceMonitor perfMonitor;
        auto start_time = common::getCurrentTime();
        auto end_time = start_time + test_duration;

        while (running.load() && common::getCurrentTime() < end_time)
        {
            perfMonitor.startFrame();

            double current_time = common::getCurrentTime();
            if (!gait_controller.update(current_time, 0.0, speed))
            {
                common::ErrorReporter::reportError("Movement-Test", "Gait Update",
                                                   "Failed during " + gait_names[i] + " gait test");
                break;
            }

            perfMonitor.endFrame();

            // Check for abort
            char key;
            if (common::TerminalManager::readChar(key) && (key == 'q' || key == 27))
            {
                std::cout << "\nGait comparison aborted by user" << std::endl;
                return false;
            }

            common::sleepMs(50);
        }

        perfMonitor.printReport(gait_names[i] + " gait ");

        // Center legs between tests
        gait_controller.centerLegs();
        common::sleepMs(1000);
    }

    std::cout << "Gait comparison test completed" << std::endl;
    return true;
}

// Stress test with continuous movement
bool test_stress_movement(Hexapod &hexapod, gait::Gait &gait_controller, double duration)
{
    // Mark hexapod as potentially unused since we only use gait_controller for movement
    (void)hexapod; // Suppress unused parameter warning

    common::ErrorReporter::reportInfo("Movement-Test", "Running stress test for " +
                                                           common::StringUtils::formatDuration(duration));

    common::PerformanceMonitor perfMonitor;
    auto start_time = common::getCurrentTime();
    auto end_time = start_time + duration;

    int successful_updates = 0;
    int total_updates = 0;

    std::vector<double> directions = {0.0, 90.0, 180.0, -90.0}; // Forward, right, back, left
    std::vector<std::string> direction_names = {"Forward", "Right", "Backward", "Left"};

    std::cout << "Running stress test with random movements..." << std::endl;

    while (running.load() && common::getCurrentTime() < end_time)
    {
        perfMonitor.startFrame();

        double current_time = common::getCurrentTime();
        double elapsed = current_time - start_time;

        // Change direction every 3 seconds
        int cycle_index = (static_cast<int>(elapsed / 3.0)) % directions.size();
        double direction = directions[cycle_index];

        // Vary speed between 0.3 and 0.8
        double speed = 0.3 + 0.5 * sin(elapsed * 0.5);

        if (!gait_controller.update(current_time, direction, speed))
        {
            common::ErrorReporter::reportError("Movement-Test", "Stress Test", "Gait update failed");
            return false;
        }

        perfMonitor.endFrame();

        if (gait_controller.update(current_time, direction, speed))
        {
            successful_updates++;
        }
        total_updates++;

        // Show progress and current status
        if (static_cast<int>(elapsed) % 3 == 0) // Update every 3 seconds
        {
            std::cout << "\rStress test: " << direction_names[cycle_index]
                      << " at speed " << common::StringUtils::formatNumber(speed, 1)
                      << " (" << common::StringUtils::formatNumber((elapsed / duration) * 100, 0) << "%)" << std::flush;
        }

        // Check for abort
        char key;
        if (common::TerminalManager::readChar(key) && (key == 'q' || key == 27))
        {
            std::cout << "\nStress test aborted by user" << std::endl;
            return false;
        }

        common::sleepMs(20); // 50Hz update rate for stress test
    }

    std::cout << std::endl;
    perfMonitor.printReport("Stress test ");
    std::cout << "Stress test completed successfully" << std::endl;
    return true;
}

// Show current status
void show_status(Hexapod &hexapod, gait::Gait &gait_controller)
{
    std::cout << "\nMovement Test Status" << std::endl;
    std::cout << "====================" << std::endl;

    // Show gait parameters
    auto params = gait_controller.getParameters();
    std::cout << "Current gait: ";
    switch (params.type)
    {
    case gait::GaitType::TRIPOD:
        std::cout << "Tripod";
        break;
    case gait::GaitType::WAVE:
        std::cout << "Wave";
        break;
    case gait::GaitType::RIPPLE:
        std::cout << "Ripple";
        break;
    }
    std::cout << std::endl;

    std::cout << "Step height: " << common::StringUtils::formatNumber(params.stepHeight) << "mm" << std::endl;
    std::cout << "Step length: " << common::StringUtils::formatNumber(params.stepLength) << "mm" << std::endl;
    std::cout << "Cycle time: " << common::StringUtils::formatNumber(params.cycleTime) << "s" << std::endl;
    std::cout << "Duty factor: " << common::StringUtils::formatNumber(params.dutyFactor, 2) << std::endl;

    // Show leg positions
    std::cout << "\nLeg positions:" << std::endl;
    for (int leg = 0; leg < hexapod::Config::NUM_LEGS; leg++)
    {
        LegPosition position;
        if (hexapod.getLegPosition(leg, position))
        {
            std::cout << "  Leg " << leg << ": Hip=" << std::setw(3) << position.getHip()
                      << "° Knee=" << std::setw(3) << position.getKnee()
                      << "° Ankle=" << std::setw(3) << position.getAnkle() << "°" << std::endl;
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
    if (!common::TerminalManager::setupImmediate())
    {
        common::ErrorReporter::reportWarning("Movement-Test", "Failed to setup immediate terminal input");
    }

    std::cout << "Movement Test Program" << std::endl;
    std::cout << "=====================" << std::endl;
    std::cout << "Testing hexapod movement patterns and gaits" << std::endl;

    // Initialize hexapod
    Hexapod hexapod;

    perfMonitor.startFrame();
    if (!hexapod.init())
    {
        common::ErrorReporter::reportError("Movement-Test", "Initialization", hexapod.getLastErrorMessage());
        common::TerminalManager::restore();
        return 1;
    }
    perfMonitor.endFrame();

    common::ErrorReporter::reportInfo("Movement-Test", "Hexapod initialized successfully in " +
                                                           common::StringUtils::formatNumber(perfMonitor.getAverageFrameTime()) + "ms");

    // Initialize gait controller
    gait::Gait gait_controller;
    gait::GaitParameters default_params;
    default_params.type = gait::GaitType::TRIPOD;
    default_params.stepHeight = 30.0;
    default_params.stepLength = 60.0;
    default_params.cycleTime = 1.0;
    default_params.dutyFactor = 0.5;

    if (!gait_controller.init(hexapod, default_params))
    {
        common::ErrorReporter::reportError("Movement-Test", "Gait Init", "Failed to initialize gait controller");
        common::TerminalManager::restore();
        return 1;
    }

    // Center all legs for safety
    std::cout << "Centering all legs for safety..." << std::endl;
    if (!gait_controller.centerLegs())
    {
        common::ErrorReporter::reportWarning("Movement-Test", "Failed to center legs");
    }

    print_menu();

    double current_speed = 0.5;
    gait::GaitType current_gait = gait::GaitType::TRIPOD;

    // Main test loop
    while (running.load())
    {
        std::cout << "Enter command (h for help): ";
        char command;

        if (common::TerminalManager::readChar(command))
        {
            switch (command)
            {
            case '1': // Forward movement
                test_forward_movement(hexapod, gait_controller, 5.0, current_speed);
                gait_controller.centerLegs();
                break;

            case '2':                                                                // Backward movement
                test_forward_movement(hexapod, gait_controller, 5.0, current_speed); // Use 180° direction
                gait_controller.centerLegs();
                break;

            case '3': // Left turn
                test_turn_movement(hexapod, gait_controller, -90.0, 3.0, current_speed);
                gait_controller.centerLegs();
                break;

            case '4': // Right turn
                test_turn_movement(hexapod, gait_controller, 90.0, 3.0, current_speed);
                gait_controller.centerLegs();
                break;

            case '5': // Figure-8
                test_figure_eight(hexapod, gait_controller, current_speed);
                gait_controller.centerLegs();
                break;

            case '6': // Square pattern
                test_square_pattern(hexapod, gait_controller, current_speed);
                gait_controller.centerLegs();
                break;

            case '7': // Circle pattern
                test_circle_pattern(hexapod, gait_controller, current_speed);
                gait_controller.centerLegs();
                break;

            case '8': // Custom speed test
            {
                std::cout << "Enter speed (0.1-1.0): ";
                double speed;
                std::cin >> speed;
                if (common::Validator::validateSpeed(speed))
                {
                    current_speed = speed;
                    test_forward_movement(hexapod, gait_controller, 3.0, speed);
                }
                else
                {
                    common::ErrorReporter::reportError("Movement-Test", "Input", "Invalid speed value");
                }
                gait_controller.centerLegs();
                break;
            }

            case '9': // Gait comparison
                test_gait_comparison(hexapod, gait_controller);
                break;

            case '0': // Stress test
                test_stress_movement(hexapod, gait_controller, 30.0);
                gait_controller.centerLegs();
                break;

            case 'g': // Toggle gait
            {
                // Cycle through gait types
                switch (current_gait)
                {
                case gait::GaitType::TRIPOD:
                    current_gait = gait::GaitType::WAVE;
                    break;
                case gait::GaitType::WAVE:
                    current_gait = gait::GaitType::RIPPLE;
                    break;
                case gait::GaitType::RIPPLE:
                    current_gait = gait::GaitType::TRIPOD;
                    break;
                }

                gait::GaitParameters params = gait_controller.getParameters();
                params.type = current_gait;
                gait_controller.setParameters(params);

                std::cout << "Switched to ";
                switch (current_gait)
                {
                case gait::GaitType::TRIPOD:
                    std::cout << "Tripod";
                    break;
                case gait::GaitType::WAVE:
                    std::cout << "Wave";
                    break;
                case gait::GaitType::RIPPLE:
                    std::cout << "Ripple";
                    break;
                }
                std::cout << " gait" << std::endl;
                break;
            }

            case '+': // Increase speed
                current_speed = std::min(1.0, current_speed + 0.1);
                std::cout << "Speed: " << common::StringUtils::formatNumber(current_speed, 1) << std::endl;
                break;

            case '-': // Decrease speed
                current_speed = std::max(0.1, current_speed - 0.1);
                std::cout << "Speed: " << common::StringUtils::formatNumber(current_speed, 1) << std::endl;
                break;

            case 'c': // Center legs
                std::cout << "Centering all legs..." << std::endl;
                if (gait_controller.centerLegs())
                {
                    std::cout << "All legs centered successfully" << std::endl;
                }
                else
                {
                    common::ErrorReporter::reportError("Movement-Test", "Center", "Failed to center legs");
                }
                break;

            case 's': // Show status
                show_status(hexapod, gait_controller);
                break;

            case 'h': // Help
                print_menu();
                break;

            case 'q': // Quit
                std::cout << "Exiting movement test program..." << std::endl;
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
    gait_controller.centerLegs();

    // Restore terminal settings
    common::TerminalManager::restore();

    std::cout << "Movement test program completed." << std::endl;
    return 0;
}