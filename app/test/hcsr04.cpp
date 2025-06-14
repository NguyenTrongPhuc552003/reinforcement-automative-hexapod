#include <iostream>
#include <iomanip>
#include <string>
#include <csignal>
#include <unistd.h>
#include <termios.h>
#include <chrono>
#include <cmath>
#include "ultrasonic.hpp"

// Global flag for program termination
static volatile bool running = true;
static struct termios orig_termios;
static bool terminal_modified = false;

// Add these global counters at the top of the file after includes
static int read_count = 0;  // Track total number of readings attempted 
static int error_count = 0; // Track number of failed/error readings

// Signal handler for graceful termination
static void handle_signal(int sig)
{
    std::cout << "\nReceived signal " << sig << ", shutting down..." << std::endl;
    running = false;
}

// Setup terminal for non-blocking input
static void setup_terminal()
{
    struct termios new_termios;

    // Get current terminal settings
    if (tcgetattr(STDIN_FILENO, &orig_termios) < 0)
    {
        std::cerr << "Warning: Failed to get terminal attributes" << std::endl;
        return;
    }

    terminal_modified = true;
    new_termios = orig_termios;

    // Disable canonical mode and echo
    new_termios.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | IEXTEN);
    new_termios.c_iflag &= ~(ISTRIP | INLCR | ICRNL | IGNCR | IXON | IXOFF);
    new_termios.c_cflag &= ~CSIZE;
    new_termios.c_cflag |= CS8;

    // Set input options for non-blocking reads
    new_termios.c_cc[VMIN] = 0;  // Return immediately with what is available
    new_termios.c_cc[VTIME] = 0; // No timeout

    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &new_termios) < 0)
    {
        std::cerr << "Warning: Failed to set terminal attributes" << std::endl;
        terminal_modified = false;
    }
}

// Restore terminal to original state
static void restore_terminal()
{
    if (terminal_modified)
    {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
        terminal_modified = false;
    }
}

// Display statistics for distance readings
static void print_stats(const std::vector<float> &readings)
{
    if (readings.empty())
        return;

    float min = readings[0];
    float max = readings[0];
    float sum = 0.0f;
    int valid_count = 0;

    for (float reading : readings)
    {
        if (reading > 0)
        { // Only count valid readings
            min = std::min(min, reading);
            max = std::max(max, reading);
            sum += reading;
            valid_count++;
        }
    }

    if (valid_count == 0)
        return;

    float avg = sum / valid_count;

    // Calculate standard deviation
    float variance = 0.0f;
    for (float reading : readings)
    {
        if (reading > 0)
        {
            variance += (reading - avg) * (reading - avg);
        }
    }
    float std_dev = std::sqrt(variance / valid_count);

    std::cout << "\nDistance Statistics (cm):" << std::endl;
    std::cout << std::fixed << std::setprecision(1)
              << "  Min: " << min
              << " | Max: " << max
              << " | Avg: " << avg
              << " | Std Dev: " << std_dev
              << " | Valid Readings: " << valid_count << "/" << readings.size()
              << std::endl;
}

int main(void)
{
    // Setup signal handlers
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // Configure terminal
    setup_terminal();

    std::cout << "HC-SR04 Ultrasonic Sensor Test\n"
              << "============================\n\n"
              << "Press 's' for statistics, 'r' to reset stats, 'q' to quit\n"
              << std::endl;

    // Initialize sensor with BeagleBone AI GPIO pins
    ultrasonic::PinConfig config{
        .trigger_chip = ultrasonic::DefaultPins::TRIGGER_CHIP,
        .trigger_line = ultrasonic::DefaultPins::TRIGGER_LINE,
        .echo_chip = ultrasonic::DefaultPins::ECHO_CHIP,
        .echo_line = ultrasonic::DefaultPins::ECHO_LINE};

    ultrasonic::Ultrasonic sensor(config);

    if (!sensor.init())
    {
        std::cerr << "Failed to initialize sensor: " << sensor.getLastError() << std::endl;
        restore_terminal();
        return 1;
    }

    // Initialize the hex  apod
    std::vector<float> readings;
    bool running = true;

    // Configure terminal for immediate character reading
    struct termios orig_termios;
    tcgetattr(STDIN_FILENO, &orig_termios);
    terminal_modified = true;

    // Set non-blocking non-canonical mode
    setup_terminal();

    // Install signal handler
    signal(SIGINT, handle_signal);

    // Create ultrasonic sensor instance
    ultrasonic::PinConfig config{
        .trigger_chip = ultrasonic::DefaultPins::TRIGGER_CHIP,
        .trigger_line = ultrasonic::DefaultPins::TRIGGER_LINE,
        .echo_chip = ultrasonic::DefaultPins::ECHO_CHIP,
        .echo_line = ultrasonic::DefaultPins::ECHO_LINE};

    auto ultrasonic = std::make_unique<ultrasonic::Ultrasonic>(config);

    if (!ultrasonic->init())
    {
        std::cerr << "Failed to initialize ultrasonic sensor: "
                  << ultrasonic->getLastError() << std::endl;
        restore_terminal();
        return 1;
    }

    std::cout << "\nUltrasonic Distance Test\n"
              << "Press Ctrl+C to end test\n"
              << std::endl;

    // Main test loop
    while (running)
    {
        // Only take reading when sensor is ready
        if (sensor.isReady())
        {
            read_count++; // Increment read attempts
            float distance = sensor.getDistance();

            if (distance < 0)
            {
                error_count++; // Increment error count on invalid reading
                std::cerr << "Error getting distance: "
                          << sensor.getLastError() << "\r";
            }
            else
            {
                readings.push_back(distance);
                std::cout << "Distance: " << std::fixed << std::setprecision(1)
                          << distance << " cm           \r" << std::flush;
            }
        }

        // Check for keyboard input to exit
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0)
        {
            if (c == 'q' || c == 'Q')
            {
                running = false;
            }
        }

        usleep(50000); // 50ms delay between readings
    }

    restore_terminal();
    std::cout << "\nTest completed." << std::endl;

    // Show final statistics
    if (!readings.empty())
    {
        std::cout << "\nFinal Statistics:" << std::endl;
        print_stats(readings);

        float success_rate = (read_count > 0) ? (1.0f - (float)error_count / read_count) * 100.0f : 0.0f;

        std::cout << "Overall success rate: " << std::fixed << std::setprecision(1)
                  << success_rate << "% ("
                  << (read_count - error_count) << "/" << read_count
                  << " readings)" << std::endl;
    }

    return 0;
}
