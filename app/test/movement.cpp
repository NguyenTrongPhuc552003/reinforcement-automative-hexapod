#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <chrono>
#include "gait.hpp"

#define DIRECTION 0.0 // Forward direction (0.0 - 180.0 degrees)
#define SPEED 0.5    // Half speed (0.0 - 1.0)

// Signal flag for graceful termination
static volatile bool running = true;

// Signal handler
static void handle_signal(int sig)
{
    std::cout << "\nReceived signal " << sig << ", shutting down..." << std::endl;
    running = false;
}

// Get current time in seconds
static double get_time()
{
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

// Function to test leg movement
static bool test_leg_movement(Hexapod &hexapod)
{
    std::cout << "Testing individual leg movement..." << std::endl;

    // Center all legs first
    std::cout << "Centering all legs..." << std::endl;
    hexapod.centerAll();
    sleep(2);

    // Reuse a single LegPosition object
    LegPosition pos(0, 0, 0);

    // Test each leg
    for (int leg = 0; leg < NUM_LEGS && running; leg++)
    {
        std::cout << "Testing leg " << leg << "..." << std::endl;

        // Move hip joint
        std::cout << "  Moving hip joint" << std::endl;
        for (int angle = -30; angle <= 30 && running; angle += 10)
        {
            pos.setHip(angle);
            pos.setKnee(0);
            pos.setAnkle(0);

            if (!hexapod.setLegPosition(leg, pos))
            {
                std::cerr << "Failed to move hip: "
                          << hexapod.getLastErrorMessage() << std::endl;
                return false;
            }
            usleep(200000); // 200ms delay
        }

        // Reset hip and test knee
        pos.setHip(0);
        pos.setKnee(0);
        pos.setAnkle(0);

        if (!hexapod.setLegPosition(leg, pos))
        {
            std::cerr << "Failed to reset hip: "
                      << hexapod.getLastErrorMessage() << std::endl;
            return false;
        }
        usleep(500000); // 500ms delay

        // Move knee joint
        std::cout << "  Moving knee joint" << std::endl;
        for (int angle = 0; angle <= 45 && running; angle += 10)
        {
            pos.setHip(0);
            pos.setKnee(angle);
            pos.setAnkle(0);

            if (!hexapod.setLegPosition(leg, pos))
            {
                std::cerr << "Failed to move knee: "
                          << hexapod.getLastErrorMessage() << std::endl;
                return false;
            }
            usleep(200000); // 200ms delay
        }

        // Reset knee and test ankle
        pos.setHip(0);
        pos.setKnee(0);
        pos.setAnkle(0);

        if (!hexapod.setLegPosition(leg, pos))
        {
            std::cerr << "Failed to reset knee: "
                      << hexapod.getLastErrorMessage() << std::endl;
            return false;
        }
        usleep(500000); // 500ms delay

        // Move ankle joint
        std::cout << "  Moving ankle joint" << std::endl;
        for (int angle = -30; angle <= 30 && running; angle += 10)
        {
            pos.setHip(0);
            pos.setKnee(0);
            pos.setAnkle(angle);

            if (!hexapod.setLegPosition(leg, pos))
            {
                std::cerr << "Failed to move ankle: "
                          << hexapod.getLastErrorMessage() << std::endl;
                return false;
            }
            usleep(200000); // 200ms delay
        }

        // Reset ankle
        pos.setHip(0);
        pos.setKnee(0);
        pos.setAnkle(0);

        if (!hexapod.setLegPosition(leg, pos))
        {
            std::cerr << "Failed to reset ankle: "
                      << hexapod.getLastErrorMessage() << std::endl;
            return false;
        }
        usleep(500000); // 500ms delay
    }

    // Center all legs again
    std::cout << "Centering all legs..." << std::endl;
    hexapod.centerAll();

    return true;
}

// Test gait patterns
static bool test_gait(Hexapod &hexapod, GaitType gaitType)
{
    Gait gait;
    GaitParameters params;
    double start_time, current_time, elapsed_time;
    double direction = 0.0; // Forward
    double speed = 0.5;     // Half-speed
    bool success = true;

    // Initialize gait parameters with LARGER values for more noticeable movement
    params.type = gaitType;
    params.stepHeight = 40.0; // Increased from 30.0 to 40.0 mm
    params.stepLength = 80.0; // Increased from 60.0 to 80.0 mm
    params.cycleTime = 3.0;   // Increased from 2.0 to 3.0 seconds for slower, more visible movement

    // Set duty factor according to gait type
    switch (gaitType)
    {
    case GaitType::TRIPOD:
        std::cout << "Testing tripod gait pattern" << std::endl;
        params.dutyFactor = 0.5; // 50% stance phase
        break;

    case GaitType::WAVE:
        std::cout << "Testing wave gait pattern" << std::endl;
        params.dutyFactor = 0.8; // 80% stance phase
        break;

    case GaitType::RIPPLE:
        std::cout << "Testing ripple gait pattern" << std::endl;
        params.dutyFactor = 0.65; // 65% stance phase
        break;

    default:
        std::cerr << "Invalid gait type" << std::endl;
        return false;
    }

    std::cout << "Initializing gait with parameters:" << std::endl;
    std::cout << "- Step Height: " << params.stepHeight << " mm" << std::endl;
    std::cout << "- Step Length: " << params.stepLength << " mm" << std::endl;
    std::cout << "- Cycle Time: " << params.cycleTime << " seconds" << std::endl;
    std::cout << "- Duty Factor: " << params.dutyFactor << std::endl;

    // Initialize gait controller
    if (!gait.init(hexapod, params))
    {
        std::cerr << "Failed to initialize gait" << std::endl;
        return false;
    }

    // Center legs before starting
    std::cout << "Centering legs..." << std::endl;
    if (!gait.centerLegs())
    {
        std::cerr << "Failed to center legs" << std::endl;
        return false;
    }
    sleep(1);

    // Run gait pattern for 15 seconds or until Ctrl+C
    std::cout << "Running gait pattern for 15 seconds (press Ctrl+C to stop)..." << std::endl;
    start_time = get_time();
    elapsed_time = 0.0;

    std::cout << "Starting movement loop..." << std::endl;
    std::cout << "Using fixed direction " << direction << "° and speed " << speed << std::endl;

    while (elapsed_time < 15.0 && running)
    {
        current_time = get_time();
        elapsed_time = current_time - start_time;

        // Update gait
        if (!gait.update(elapsed_time, direction, speed))
        {
            std::cerr << "Gait update error at time " << std::fixed << std::setprecision(2)
                      << elapsed_time << std::endl;
            success = false;
            break;
        }

        // Print status every second - using a static variable to avoid duplicates
        static int last_logged_second = -1;
        int current_second = static_cast<int>(elapsed_time);

        if (current_second != last_logged_second)
        {
            std::cout << "Time: " << std::fixed << std::setprecision(1) << elapsed_time
                      << " s, Direction: " << direction << "°, Speed: " << speed << std::endl;

            // Every 5 seconds, switch direction to confirm servos respond
            if (current_second % 5 == 0)
            {
                direction = (current_second % 10 == 0) ? 0.0 : 180.0; // alternate forward/backward
                std::cout << "Switching direction to " << direction << "°" << std::endl;
            }

            // Update last logged second
            last_logged_second = current_second;
        }

        // Reduce delay for smoother motion - 10ms (100Hz update rate)
        usleep(10000);
    }

    // Clean up - gait will clean itself up in destructor

    // Center legs again
    std::cout << "Centering legs..." << std::endl;
    gait.centerLegs();

    return success;
}

// Main test program
int main(int argc, char *argv[])
{
    // Set up signal handler
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    std::cout << "Hexapod Movement Test Program" << std::endl;
    std::cout << "----------------------------" << std::endl;

    // Initialize hexapod
    Hexapod hexapod;
    if (!hexapod.init())
    {
        ErrorInfo err = hexapod.getLastError();
        std::cerr << "Failed to initialize hexapod: [" << err.getCode() << "] "
                  << err.getMessage() << std::endl;

        // Provide specific recommendations based on error category
        switch (err.getCategory())
        {
        case ErrorCategory::DEVICE:
            std::cerr << "Device issue: Check if the device exists and the driver is loaded" << std::endl;
            break;
        case ErrorCategory::SYSTEM:
            std::cerr << "System issue: Check permissions and system resources" << std::endl;
            break;
        case ErrorCategory::HARDWARE:
            std::cerr << "Hardware issue: Check physical connections" << std::endl;
            break;
        default:
            std::cerr << "Try running with sudo privileges" << std::endl;
        }
        return 1;
    }
    std::cout << "Connected to hexapod device\n"
              << std::endl;

    // Test a simple leg movement to verify servo communication
    std::cout << "Testing basic leg movement..." << std::endl;
    LegPosition testPos(15, 15, 15); // small movement to verify servos respond
    if (hexapod.setLegPosition(0, testPos))
    {
        std::cout << "Servo test command sent successfully" << std::endl;
    }
    else
    {
        std::cout << "Warning: Servo test command failed: "
                  << hexapod.getLastErrorMessage() << std::endl;
    }
    sleep(1);
    hexapod.centerAll();
    sleep(1);

    // Run tests (or specific test if specified)
    if (argc > 1)
    {
        std::string test_type = argv[1];

        if (test_type == "leg")
        {
            test_leg_movement(hexapod);
        }
        else if (test_type == "tripod")
        {
            test_gait(hexapod, GaitType::TRIPOD);
        }
        else if (test_type == "wave")
        {
            test_gait(hexapod, GaitType::WAVE);
        }
        else if (test_type == "ripple")
        {
            test_gait(hexapod, GaitType::RIPPLE);
        }
        else
        {
            std::cout << "Unknown test: " << test_type << std::endl;
            std::cout << "Available tests: leg, tripod, wave, ripple" << std::endl;
        }
    }
    else
    {
        // Run all tests
        std::cout << "Running all tests. Press Ctrl+C to skip to next test.\n"
                  << std::endl;
        std::cout << "\n=== Testing Individual Leg Movement ===" << std::endl;
        test_leg_movement(hexapod);
        if (!running)
            goto cleanup;
        sleep(1);

        std::cout << "\n=== Testing Tripod Gait ===" << std::endl;
        test_gait(hexapod, GaitType::TRIPOD);
        if (!running)
            goto cleanup;
        sleep(1);

        std::cout << "\n=== Testing Wave Gait ===" << std::endl;
        test_gait(hexapod, GaitType::WAVE);
        if (!running)
            goto cleanup;
        sleep(1);

        std::cout << "\n=== Testing Ripple Gait ===" << std::endl;
        test_gait(hexapod, GaitType::RIPPLE);
    }

cleanup:
    // Center all legs before exit
    std::cout << "\nCentering all legs..." << std::endl;
    hexapod.centerAll();

    std::cout << "Test program completed." << std::endl;
    return 0;
}