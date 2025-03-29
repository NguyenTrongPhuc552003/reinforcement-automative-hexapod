#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <chrono>
#include <termios.h>
#include "hexapod.hpp"

// Global flags for program control
static volatile bool running = true;
static volatile bool emergency_stop = false;

// Configuration parameters
static struct
{
    uint8_t leg_num;   // Current leg to test
    int sweep_range;   // Range of motion in degrees (+/-)
    int sweep_steps;   // Number of steps in the sweep motion
    int delay_ms;      // Delay between position updates in ms
    int transition_ms; // Delay between joint transitions in ms
} config = {
    0,   // Default to leg 0
    45,  // Default to +/- 45 degrees
    100, // Default to 100 steps
    30,  // Default to 30ms between updates
    300  // Default to 300ms between transitions
};

// Signal handler for graceful termination
void handle_signal(int sig)
{
    if (sig == SIGINT)
    {
        std::cout << "\nReceived interrupt signal, initiating shutdown..." << std::endl;
        running = false;
    }
    else if (sig == SIGTERM)
    {
        std::cout << "\nReceived termination signal, shutting down immediately..." << std::endl;
        emergency_stop = true;
        running = false;
    }

    // Force cleanup to happen faster on second signal
    static int signal_count = 0;
    if (++signal_count >= 2)
    {
        std::cerr << "\nForced exit - multiple signals received" << std::endl;
        exit(1); // Force immediate exit if user is impatient
    }
}

// Print progress bar
void print_progress(int current, int total, const char *joint_name)
{
    const int bar_width = 40;
    float progress = static_cast<float>(current) / total;
    int pos = static_cast<int>(bar_width * progress);

    std::cout << joint_name << " [";
    for (int i = 0; i < bar_width; ++i)
    {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << std::setw(3) << static_cast<int>(progress * 100) << "%\r";
    std::cout << std::flush;
}

// Get current time for performance measurements
double get_current_time()
{
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

// Test a specific joint with improved error handling and feedback
bool test_joint(Hexapod &hexapod, uint8_t leg_num, int joint_index, const char *joint_name)
{
    std::cout << "\nTesting " << joint_name << " joint of leg " << static_cast<int>(leg_num) << "..." << std::endl;

    double start_time = get_current_time();
    int success_count = 0;
    int error_count = 0;
    int consecutive_errors = 0; // Track consecutive errors for early bailout

    // Create a single LegPosition object and reuse it
    LegPosition pos(0, 0, 0);

    // Perform the sweep motion
    for (int i = 0; i < config.sweep_steps && running && !emergency_stop; i++)
    {
        // Calculate angle using sine wave: smooth transition from -range to +range
        double progress = static_cast<double>(i) / config.sweep_steps;
        int16_t angle = static_cast<int16_t>(sin(progress * M_PI) * config.sweep_range);

        // Only set the target joint's angle - more efficient than creating a new object each time
        if (joint_index == 0)
            pos.setHip(angle);
        else if (joint_index == 1)
            pos.setKnee(angle);
        else
            pos.setAnkle(angle);

        // Update progress bar every 5th step
        if (i % 5 == 0)
            print_progress(i, config.sweep_steps, joint_name);

        if (hexapod.setLegPosition(leg_num, pos))
        {
            success_count++;
            consecutive_errors = 0; // Reset consecutive error counter
        }
        else
        {
            error_count++;
            consecutive_errors++;

            std::cerr << "\nError setting " << joint_name << " angle to " << angle
                      << ": " << hexapod.getLastErrorMessage() << std::endl;

            // Bail out if too many consecutive errors
            if (consecutive_errors >= 5)
            {
                std::cerr << "Too many consecutive errors, aborting joint test" << std::endl;
                return false;
            }

            // Brief delay after error before continuing
            usleep(100000);
        }

        // Sleep for specified delay between updates
        usleep(config.delay_ms * 1000);
    }

    // Complete the progress bar
    print_progress(config.sweep_steps, config.sweep_steps, joint_name);
    std::cout << std::endl;

    // Print performance statistics
    double elapsed = get_current_time() - start_time;
    double success_rate = (success_count > 0) ? (static_cast<double>(success_count) / (success_count + error_count) * 100.0) : 0.0;

    std::cout << "Joint test completed in " << std::fixed << std::setprecision(2)
              << elapsed << "s with " << success_rate << "% success rate" << std::endl;

    return (success_count > 0 && success_rate >= 80.0);
}

// Test movement on a specific leg
bool test_leg(Hexapod &hexapod, uint8_t leg_num)
{
    bool success = true;

    std::cout << "\n=== Testing Leg " << static_cast<int>(leg_num) << " ===" << std::endl;

    // Center the leg before testing
    LegPosition center(0, 0, 0);
    if (!hexapod.setLegPosition(leg_num, center))
    {
        std::cerr << "Failed to center leg before test: "
                  << hexapod.getLastErrorMessage() << std::endl;
        return false;
    }

    usleep(config.transition_ms * 1000);

    // Test each joint
    success &= test_joint(hexapod, leg_num, 0, "Hip  ");
    if (!running || emergency_stop)
        return false;

    usleep(config.transition_ms * 1000);

    success &= test_joint(hexapod, leg_num, 1, "Knee ");
    if (!running || emergency_stop)
        return false;

    usleep(config.transition_ms * 1000);

    success &= test_joint(hexapod, leg_num, 2, "Ankle");
    if (!running || emergency_stop)
        return false;

    // Re-center the leg after testing
    if (!hexapod.setLegPosition(leg_num, center))
    {
        std::cerr << "Failed to center leg after test: "
                  << hexapod.getLastErrorMessage() << std::endl;
        success = false;
    }

    std::cout << "\nLeg " << static_cast<int>(leg_num) << " test "
              << (success ? "PASSED" : "FAILED") << std::endl;

    return success;
}

// Test each leg sequentially
void test_all_legs(Hexapod &hexapod)
{
    int passed = 0;
    int failed = 0;

    std::cout << "\n=== Testing All Legs ===" << std::endl;

    for (uint8_t leg = 0; leg < NUM_LEGS && running && !emergency_stop; leg++)
    {
        if (test_leg(hexapod, leg))
            passed++;
        else
            failed++;

        // Brief delay between legs
        usleep(500000);
    }

    std::cout << "\nTest Summary: " << passed << " legs passed, " << failed << " legs failed" << std::endl;
}

// Test synchronized movement of all legs
void test_synchronized_movement(Hexapod &hexapod)
{
    std::cout << "\n=== Testing Synchronized Movement ===" << std::endl;

    // Center all legs first
    std::cout << "Centering all legs..." << std::endl;
    if (!hexapod.centerAll())
    {
        std::cerr << "Failed to center all legs" << std::endl;
        return;
    }

    sleep(1);

    std::cout << "Moving all legs simultaneously..." << std::endl;

    // Perform synchronized sweep
    for (int i = 0; i < config.sweep_steps && running && !emergency_stop; i++)
    {
        double progress = static_cast<double>(i) / config.sweep_steps;
        int16_t hip_angle = static_cast<int16_t>(sin(progress * M_PI) * 20);         // ±20°
        int16_t knee_angle = static_cast<int16_t>(sin(progress * M_PI * 2) * 30);    // ±30°
        int16_t ankle_angle = static_cast<int16_t>(sin(progress * M_PI * 0.5) * 25); // ±25°

        // Update all legs with the same angles
        for (uint8_t leg = 0; leg < NUM_LEGS; leg++)
        {
            LegPosition pos(hip_angle, knee_angle, ankle_angle);
            hexapod.setLegPosition(leg, pos);
        }

        // Update progress every 5th step
        if (i % 5 == 0)
        {
            print_progress(i, config.sweep_steps, "Sync");
        }

        // Shorter delay for synchronized movement
        usleep(config.delay_ms * 1000);
    }

    print_progress(config.sweep_steps, config.sweep_steps, "Sync");
    std::cout << std::endl;

    // Center all legs again
    std::cout << "Centering all legs..." << std::endl;
    hexapod.centerAll();
}

// Parse command line arguments and set configuration
void parse_arguments(int argc, char *argv[])
{
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];

        if (arg == "--leg" && i + 1 < argc)
        {
            config.leg_num = static_cast<uint8_t>(std::stoi(argv[++i]) % NUM_LEGS);
        }
        else if (arg == "--range" && i + 1 < argc)
        {
            config.sweep_range = std::stoi(argv[++i]);
            config.sweep_range = std::min(90, std::max(10, config.sweep_range));
        }
        else if (arg == "--steps" && i + 1 < argc)
        {
            config.sweep_steps = std::stoi(argv[++i]);
            config.sweep_steps = std::min(500, std::max(20, config.sweep_steps));
        }
        else if (arg == "--delay" && i + 1 < argc)
        {
            config.delay_ms = std::stoi(argv[++i]);
            config.delay_ms = std::min(500, std::max(10, config.delay_ms));
        }
        else if (arg == "--help")
        {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --leg N      Test only leg N (0-5)" << std::endl;
            std::cout << "  --range N    Set sweep range to ±N degrees (10-90)" << std::endl;
            std::cout << "  --steps N    Set sweep steps to N (20-500)" << std::endl;
            std::cout << "  --delay N    Set delay between updates to N ms (10-500)" << std::endl;
            std::cout << "  --help       Show this help message" << std::endl;
            exit(0);
        }
    }
}

// Main program
int main(int argc, char *argv[])
{
    // Register signal handler
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    // Parse command line arguments
    parse_arguments(argc, argv);

    std::cout << "Hexapod Servo Test Utility" << std::endl;
    std::cout << "=========================" << std::endl;
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Leg: " << static_cast<int>(config.leg_num) << std::endl;
    std::cout << "  Sweep Range: ±" << config.sweep_range << " degrees" << std::endl;
    std::cout << "  Sweep Steps: " << config.sweep_steps << std::endl;
    std::cout << "  Update Delay: " << config.delay_ms << " ms" << std::endl;
    std::cout << "  Transition Delay: " << config.transition_ms << " ms" << std::endl;

    // Initialize hexapod with retry logic
    Hexapod hexapod;
    bool initialized = false;
    int retries = 3;

    while (!initialized && retries-- > 0)
    {
        std::cout << "\nInitializing hexapod hardware (attempt " << 3 - retries << ")..." << std::endl;
        if (hexapod.init())
        {
            initialized = true;
            std::cout << "Hexapod initialized successfully!" << std::endl;
        }
        else
        {
            std::cerr << "Failed to initialize: " << hexapod.getLastErrorMessage() << std::endl;
            if (retries > 0)
            {
                std::cout << "Retrying in 1 second..." << std::endl;
                sleep(1);
            }
            else
            {
                std::cerr << "Failed to initialize after multiple attempts." << std::endl;
                return 1;
            }
        }
    }

    // Center all legs initially
    if (!hexapod.centerAll())
    {
        std::cerr << "Warning: Failed to center all legs at startup" << std::endl;
    }

    if (argc > 1 && std::string(argv[1]) == "all")
    {
        // Test all legs sequentially
        test_all_legs(hexapod);

        if (running && !emergency_stop)
        {
            // Test synchronized movement after individual tests
            test_synchronized_movement(hexapod);
        }
    }
    else
    {
        // Test specific leg from configuration
        test_leg(hexapod, config.leg_num);
    }

    // Center all legs before exit for safety
    std::cout << "\nCentering all legs before exit..." << std::endl;
    hexapod.centerAll();

    // Clean exit
    std::cout << "\nTest completed." << std::endl;
    return 0;
}
