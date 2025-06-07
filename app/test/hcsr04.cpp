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
    ultrasonic::Ultrasonic::PinConfig config{
        .trigger_pin = ultrasonic::DefaultPins::TRIGGER_PIN, // P9_16
        .echo_pin = ultrasonic::DefaultPins::ECHO_PIN,       // P9_41
    };

    ultrasonic::Ultrasonic sensor(config);

    if (!sensor.init())
    {
        std::cerr << "Failed to initialize sensor: " << sensor.getLastError() << std::endl;
        restore_terminal();
        return 1;
    }

    std::vector<float> readings;
    readings.reserve(1000); // Pre-allocate space for efficiency

    int read_count = 0;
    int error_count = 0;
    auto last_stats_time = std::chrono::steady_clock::now();

    while (running)
    {
        auto start = std::chrono::steady_clock::now();

        // Process any keypress
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0)
        {
            switch (c)
            {
            case 'q':
                running = false;
                break;
            case 's':
                print_stats(readings);
                break;
            case 'r':
                readings.clear();
                read_count = 0;
                error_count = 0;
                std::cout << "Statistics reset" << std::endl;
                break;
            }
        }

        // Get distance measurement
        float distance = sensor.getDistance();
        read_count++;

        if (distance > 0)
        {
            readings.push_back(distance);

            // Print current reading with color coding based on distance
            std::cout << "\rDistance: ";
            if (distance < 20.0f)
            {
                std::cout << "\033[31m"; // Red for close objects
            }
            else if (distance < 50.0f)
            {
                std::cout << "\033[33m"; // Yellow for medium range
            }
            else
            {
                std::cout << "\033[32m"; // Green for clear path
            }
            std::cout << std::fixed << std::setprecision(1) << std::setw(6)
                      << distance << " cm\033[0m" << std::flush;
        }
        else
        {
            error_count++;
            std::cout << "\rError: " << sensor.getLastError() << std::flush;
        }

        // Show stats every 5 seconds
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_time).count() >= 5)
        {
            float success_rate = (read_count > 0) ? (1.0f - (float)error_count / read_count) * 100.0f : 0.0f;

            std::cout << "\nSuccess rate: " << std::fixed << std::setprecision(1)
                      << success_rate << "% ("
                      << (read_count - error_count) << "/" << read_count
                      << " readings)\n"
                      << std::endl;

            last_stats_time = now;
        }

        // Calculate how long to sleep
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start);

        // Maintain ~10Hz sampling rate
        if (elapsed < std::chrono::milliseconds(100))
        {
            usleep((100 - elapsed.count()) * 1000);
        }
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
