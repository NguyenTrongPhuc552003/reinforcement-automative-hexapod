#include <iostream>
#include <iomanip>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <ctime>
#include "hexapod.hpp"
#include "kinematics.hpp"

// Global control flags
static volatile bool running = true;
static volatile bool pause_balance = false;
static volatile bool debug_mode = false;

// Default parameters
struct BalanceConfig
{
    double max_tilt_adjustment = 30.0; // Maximum angle adjustment in degrees
    double base_height = -120.0;       // Base height for standing (mm)
    double response_factor = 0.8;      // How responsive the balance is (0.0-1.0)
    double deadzone = 2.0;             // Minimum tilt (degrees) before reacting
    double update_rate = 20.0;         // Target updates per second
    int display_decimation = 5;        // Only update display every N iterations
} config;

// Terminal state handling
static struct termios orig_termios;

// Signal handler for graceful termination
void handle_signal(int sig)
{
    std::cout << "\nReceived signal " << sig << ", exiting..." << std::endl;
    running = false;
}

// Setup terminal for non-blocking input
void setup_terminal()
{
    // Get current terminal settings
    tcgetattr(STDIN_FILENO, &orig_termios);

    // Modified settings for non-canonical mode
    struct termios new_termios = orig_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN] = 0;
    new_termios.c_cc[VTIME] = 0;

    // Apply new settings
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

// Restore terminal to original state
void restore_terminal()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
}

// Process keyboard input
void process_input()
{
    char c;
    if (read(STDIN_FILENO, &c, 1) <= 0)
        return;

    switch (c)
    {
    case 'q':
        running = false;
        break;
    case 'p':
        pause_balance = !pause_balance;
        std::cout << (pause_balance ? "Balance paused" : "Balance resumed") << std::endl;
        break;
    case 'd':
        debug_mode = !debug_mode;
        std::cout << (debug_mode ? "Debug mode ON" : "Debug mode OFF") << std::endl;
        break;
    case '+':
        config.response_factor = std::min(1.0, config.response_factor + 0.1);
        std::cout << "Response factor: " << config.response_factor << std::endl;
        break;
    case '-':
        config.response_factor = std::max(0.1, config.response_factor - 0.1);
        std::cout << "Response factor: " << config.response_factor << std::endl;
        break;
    case 'r':
        // Reset to default height and orientation
        std::cout << "Resetting to default position" << std::endl;
        break;
    case 'h':
        // Show help
        std::cout << "\nBalance Test Controls:\n"
                  << "  q: Quit the test\n"
                  << "  p: Pause/resume balance\n"
                  << "  d: Toggle debug mode\n"
                  << "  +: Increase response factor\n"
                  << "  -: Decrease response factor\n"
                  << "  r: Reset to default position\n"
                  << "  h: Show this help\n"
                  << std::endl;
        break;
    }
}

// Calculate tilt angles from accelerometer data
void calculate_tilt(const ImuData &imu_data, double &roll, double &pitch)
{
    // Conversion for accessing accelerometer data in g units
    double ax = imu_data.getAccelX();
    double ay = imu_data.getAccelY();
    double az = imu_data.getAccelZ();

    // Calculate roll and pitch in degrees
    // Roll: rotation around X-axis (side-to-side tilt)
    // Pitch: rotation around Y-axis (front-to-back tilt)
    roll = atan2(ay, az) * 180.0 / M_PI;
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
}

// Apply tilt compensation to leg positions
bool apply_balance_adjustments(Hexapod &hexapod, double roll, double pitch)
{
    // Apply deadzone - ignore small tilts
    if (fabs(roll) < config.deadzone)
        roll = 0;
    if (fabs(pitch) < config.deadzone)
        pitch = 0;

    // Cap maximum adjustment
    roll = std::max(-config.max_tilt_adjustment, std::min(config.max_tilt_adjustment, roll));
    pitch = std::max(-config.max_tilt_adjustment, std::min(config.max_tilt_adjustment, pitch));

    // Scale by response factor
    roll *= config.response_factor;
    pitch *= config.response_factor;

    // No need to adjust if tilt is negligible
    if (fabs(roll) < 0.1 && fabs(pitch) < 0.1)
    {
        return true;
    }

    // Prepare for inverse kinematics calculations
    bool success = true;

    // Loop through each leg to calculate and apply adjustments
    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        // Define base position for this leg
        double baseX, baseY, baseZ = config.base_height;

        // Default leg positions (when standing) - these depend on leg position around body
        // Note: These are approximate hexapod "home" positions and would be adjusted
        // based on the actual robot's geometry
        switch (leg)
        {
        case 0: // Front right
            baseX = 100.0;
            baseY = 100.0;
            break;
        case 1: // Middle right
            baseX = 0.0;
            baseY = 120.0;
            break;
        case 2: // Rear right
            baseX = -100.0;
            baseY = 100.0;
            break;
        case 3: // Front left
            baseX = 100.0;
            baseY = -100.0;
            break;
        case 4: // Middle left
            baseX = 0.0;
            baseY = -120.0;
            break;
        case 5: // Rear left
            baseX = -100.0;
            baseY = -100.0;
            break;
        }

        // Calculate leg height adjustment based on tilt
        // This creates a plane that compensates for the tilt
        double zAdjustment = sin(roll * M_PI / 180.0) * baseY - sin(pitch * M_PI / 180.0) * baseX;

        // Apply adjustment to position
        kinematics::Point3D targetPos(baseX, baseY, baseZ + zAdjustment);

        // Use inverse kinematics to get joint angles
        LegPosition legPos;
        legPos.leg_num = leg;

        if (Kinematics::getInstance().inverseKinematics(targetPos, legPos))
        {
            // Apply the calculated position
            if (!hexapod.setLegPosition(leg, legPos))
            {
                std::cerr << "Failed to set position for leg " << leg << ": "
                          << hexapod.getLastErrorMessage() << std::endl;
                success = false;
            }
        }
        else
        {
            std::cerr << "Inverse kinematics failed for leg " << leg << std::endl;
            success = false;
        }
    }

    return success;
}

// Display current status
void display_status(const ImuData &imu_data, double roll, double pitch, int iter)
{
    // Only update display periodically to avoid console flicker
    if (iter % config.display_decimation != 0)
        return;

    std::cout << "\r"
              << "Roll: " << std::fixed << std::setprecision(1) << std::setw(6) << roll << "° | "
              << "Pitch: " << std::setw(6) << pitch << "° | "
              << "Accel: X=" << std::setw(5) << imu_data.getAccelX() << "g "
              << "Y=" << std::setw(5) << imu_data.getAccelY() << "g "
              << "Z=" << std::setw(5) << imu_data.getAccelZ() << "g | "
              << (pause_balance ? "PAUSED" : "ACTIVE") << std::flush;
}

// Parse command line arguments
void parse_arguments(int argc, char *argv[], BalanceConfig &config)
{
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h")
        {
            std::cout << "Hexapod Balance Test - Usage:\n"
                      << "  " << argv[0] << " [options]\n\n"
                      << "Options:\n"
                      << "  -r, --response FACTOR   Set response factor (0.1-1.0, default: 0.8)\n"
                      << "  -d, --deadzone DEGREES  Set deadzone in degrees (0.0-10.0, default: 2.0)\n"
                      << "  -z, --height HEIGHT     Set base height in mm (default: -120.0)\n"
                      << "  -m, --max-tilt DEGREES  Set maximum tilt adjustment (default: 30.0)\n"
                      << "  -u, --update-rate HZ    Set target update frequency (default: 20.0)\n"
                      << "  -h, --help              Show this help message\n"
                      << std::endl;
            exit(0);
        }
        else if ((arg == "--response" || arg == "-r") && i + 1 < argc)
        {
            config.response_factor = std::stod(argv[++i]);
            // Clamp to valid range
            config.response_factor = std::max(0.1, std::min(1.0, config.response_factor));
        }
        else if ((arg == "--deadzone" || arg == "-d") && i + 1 < argc)
        {
            config.deadzone = std::stod(argv[++i]);
            // Clamp to valid range
            config.deadzone = std::max(0.0, std::min(10.0, config.deadzone));
        }
        else if ((arg == "--height" || arg == "-z") && i + 1 < argc)
        {
            config.base_height = std::stod(argv[++i]);
            // Clamp to reasonable range
            config.base_height = std::max(-150.0, std::min(-50.0, config.base_height));
        }
        else if ((arg == "--max-tilt" || arg == "-m") && i + 1 < argc)
        {
            config.max_tilt_adjustment = std::stod(argv[++i]);
            // Clamp to reasonable range
            config.max_tilt_adjustment = std::max(5.0, std::min(45.0, config.max_tilt_adjustment));
        }
        else if ((arg == "--update-rate" || arg == "-u") && i + 1 < argc)
        {
            config.update_rate = std::stod(argv[++i]);
            // Clamp to reasonable range
            config.update_rate = std::max(5.0, std::min(50.0, config.update_rate));
        }
    }
}

// Display current configuration
void display_config(const BalanceConfig &config)
{
    std::cout << "Current configuration:\n"
              << "  Response factor: " << config.response_factor << "\n"
              << "  Deadzone: " << config.deadzone << " degrees\n"
              << "  Base height: " << config.base_height << " mm\n"
              << "  Maximum tilt: " << config.max_tilt_adjustment << " degrees\n"
              << "  Update rate: " << config.update_rate << " Hz\n"
              << std::endl;
}

int main(int argc, char *argv[])
{
    // Register signal handler
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    // Parse command line arguments
    parse_arguments(argc, argv, config);

    // Set up terminal for immediate input
    setup_terminal();

    std::cout << "Hexapod Balance Test" << std::endl;
    std::cout << "===================" << std::endl;
    std::cout << "This test uses IMU data to maintain balance when the hexapod is tilted." << std::endl;

    // Display configuration after parsing arguments
    display_config(config);

    std::cout << "Press 'h' for help, 'q' to quit." << std::endl
              << std::endl;

    // Initialize hexapod
    Hexapod hexapod;
    if (!hexapod.init())
    {
        std::cerr << "Failed to initialize hexapod: "
                  << hexapod.getLastErrorMessage() << std::endl;
        restore_terminal();
        return 1;
    }

    std::cout << "Hexapod initialized. Centering legs..." << std::endl;

    // Center all legs to starting position
    if (!hexapod.centerAll())
    {
        std::cerr << "Failed to center legs: "
                  << hexapod.getLastErrorMessage() << std::endl;
        hexapod.cleanup();
        restore_terminal();
        return 1;
    }

    std::cout << "Starting balance control loop. Press 'q' to quit." << std::endl;

    // Main balance loop
    int iterations = 0;
    double last_update_time = hexapod.getCurrentTime();
    double update_interval = 1.0 / config.update_rate;

    while (running)
    {
        // Process any keyboard input
        process_input();

        // Check if it's time for an update
        double current_time = hexapod.getCurrentTime();
        if ((current_time - last_update_time) < update_interval)
        {
            // Sleep for a bit to avoid consuming 100% CPU
            usleep(1000);
            continue;
        }

        // Update timing
        last_update_time = current_time;
        iterations++;

        // Read IMU data
        ImuData imu_data;
        if (!hexapod.getImuData(imu_data))
        {
            std::cerr << "\nFailed to read IMU data: "
                      << hexapod.getLastErrorMessage() << std::endl;
            continue;
        }

        // Calculate tilt angles from accelerometer data
        double roll, pitch;
        calculate_tilt(imu_data, roll, pitch);

        // Display status
        display_status(imu_data, roll, pitch, iterations);

        // Skip adjustment if paused
        if (pause_balance)
        {
            continue;
        }

        // Apply balance adjustments
        apply_balance_adjustments(hexapod, roll, pitch);

        // Extra debug information if enabled
        if (debug_mode && iterations % config.display_decimation == 0)
            hexapod.printImuData(imu_data);
    }

    // Clean shutdown
    std::cout << "\nShutting down..." << std::endl;

    // Center all legs before exit
    hexapod.centerAll();
    hexapod.cleanup();

    // Restore terminal settings
    restore_terminal();

    std::cout << "Balance test completed." << std::endl;
    return 0;
}
