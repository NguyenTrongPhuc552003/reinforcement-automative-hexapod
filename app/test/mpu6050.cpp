#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <ctime>
#include <algorithm>
#include "hexapod.hpp"

// Global flag for termination
static volatile bool running = true;

// Terminal state
static struct termios orig_termios;

// Signal handler
static void test_mpu6050_signal_handler(int sig)
{
    running = false;
    std::cout << "\nReceived signal " << sig << ", exiting..." << std::endl;
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

    // Save a copy for modifications
    new_termios = orig_termios;

    // Disable canonical mode and echo
    new_termios.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | IEXTEN);

    // Disable implementation-defined input processing
    new_termios.c_iflag &= ~(ISTRIP | INLCR | ICRNL | IGNCR | IXON | IXOFF);

    // Set input character size
    new_termios.c_cflag &= ~CSIZE;
    new_termios.c_cflag |= CS8;

    // One input byte is enough to return from read()
    new_termios.c_cc[VMIN] = 0;

    // Inter-character timer unused (timeout immediately)
    new_termios.c_cc[VTIME] = 0;

    // Apply the new settings
    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &new_termios) < 0)
    {
        std::cerr << "Warning: Failed to set terminal attributes" << std::endl;
    }

    // Set stdin to non-blocking mode
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags == -1)
    {
        std::cerr << "Warning: Failed to get file flags" << std::endl;
    }
    else if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        std::cerr << "Warning: Failed to set non-blocking mode" << std::endl;
    }
}

// Restore terminal to original state
static void restore_terminal()
{
    // Restore original terminal settings
    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios) < 0)
    {
        std::cerr << "Warning: Failed to restore terminal attributes" << std::endl;
    }
}

// Display timing statistics for IMU readings
static void print_timing_stats(const double *intervals, int count)
{
    if (count < 2)
        return;

    double min = intervals[0];
    double max = intervals[0];
    double sum = 0.0;
    double avg, std_dev = 0.0;

    // Calculate min, max, sum
    for (int i = 0; i < count; i++)
    {
        min = std::min(min, intervals[i]);
        max = std::max(max, intervals[i]);
        sum += intervals[i];
    }

    // Calculate average
    avg = sum / count;

    // Calculate standard deviation
    for (int i = 0; i < count; i++)
    {
        std_dev += (intervals[i] - avg) * (intervals[i] - avg);
    }
    std_dev = std::sqrt(std_dev / count);

    std::cout << "\nIMU Timing Statistics (ms):" << std::endl;
    std::cout << "  Min: " << std::fixed << std::setprecision(2) << min * 1000.0
              << " | Max: " << max * 1000.0
              << " | Avg: " << avg * 1000.0
              << " | Std Dev: " << std_dev * 1000.0
              << " | Samples: " << count << std::endl;
}

int main()
{
    // Set up signal handler for clean termination
    std::signal(SIGINT, test_mpu6050_signal_handler);
    std::signal(SIGTERM, test_mpu6050_signal_handler);

    // Set up terminal
    setup_terminal();

    // Initialize hexapod with retry logic
    Hexapod hexapod;
    bool initialized = false;
    int retries = 3;

    std::cout << "MPU6050 Test - Press Ctrl+C to exit" << std::endl;
    std::cout << "================================" << std::endl;
    std::cout << "Note: MPU6050 sensor automatically wakes from sleep mode when needed\n"
              << std::endl;

    while (!initialized && retries-- > 0)
    {
        std::cout << "Initializing hexapod hardware (attempt " << (3 - retries) << ")..." << std::endl;
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
                std::cerr << "Maximum retries reached, giving up." << std::endl;
                restore_terminal();
                return 1;
            }
        }
    }

    // Main loop with timeout detection and statistics gathering
    double last_successful_read = hexapod.getCurrentTime();
    double read_intervals[100] = {0}; // Store last 100 read intervals
    int interval_idx = 0;
    int total_reads = 0;
    int successful_reads = 0;
    double last_stats_time = hexapod.getCurrentTime();

    std::cout << "\nReading IMU data..." << std::endl;
    std::cout << "Press 'q' to quit, 's' to show statistics\n"
              << std::endl;

    while (running)
    {
        ImuData imuData;
        char key;
        double start_time = hexapod.getCurrentTime();

        // Check for key press
        if (read(STDIN_FILENO, &key, 1) > 0)
        {
            if (key == 'q' || key == 'Q')
            {
                std::cout << "\nUser requested exit" << std::endl;
                break;
            }
            else if (key == 's' || key == 'S')
            {
                print_timing_stats(read_intervals, std::min(total_reads, 100));
            }
        }

        // Get IMU data with timeout detection
        bool success = hexapod.getImuData(imuData);
        double now = hexapod.getCurrentTime();
        double elapsed = now - start_time;

        total_reads++;

        if (success)
        {
            // Record timing interval for statistics
            read_intervals[interval_idx] = elapsed;
            interval_idx = (interval_idx + 1) % 100;

            // Print formatted IMU data
            std::cout << "\rAccel: X=" << std::fixed << std::showpos << std::setprecision(2) << std::setw(6)
                      << imuData.getAccelX() << "g Y=" << std::setw(6) << imuData.getAccelY()
                      << "g Z=" << std::setw(6) << imuData.getAccelZ() << "g | Gyro: X="
                      << std::setw(7) << imuData.getGyroX() << "° Y=" << std::setw(7)
                      << imuData.getGyroY() << "° Z=" << std::setw(7) << imuData.getGyroZ() << "°/s" << std::flush;
            last_successful_read = now;
            successful_reads++;
        }
        else
        {
            if (now - last_successful_read > 5.0) // 5 second timeout
            {
                std::cerr << "\nNo response from IMU for 5 seconds, exiting..." << std::endl;
                break;
            }
            std::cerr << "\rFailed to read IMU: " << hexapod.getLastErrorMessage() << std::flush;
        }

        // Show statistics periodically
        if (now - last_stats_time > 10.0)
        {
            double success_rate = (successful_reads * 100.0) / total_reads;
            std::cout << "\n\nSuccess rate: " << std::fixed << std::setprecision(1)
                      << success_rate << "% (" << successful_reads << "/"
                      << total_reads << ") in last 10 seconds" << std::endl;
            last_stats_time = now;
        }

        // Brief sleep to prevent excessive CPU usage - dynamically adjust based on performance
        usleep(elapsed < 0.005 ? 50000 : 10000); // Sleep longer if reads are fast
    }

    std::cout << "\n\nExiting MPU6050 test program..." << std::endl;

    // Print final statistics
    print_timing_stats(read_intervals, std::min(total_reads, 100));

    if (total_reads > 0)
    {
        double overall_success_rate = (successful_reads * 100.0) / total_reads;
        std::cout << "Overall success rate: " << std::fixed << std::setprecision(1)
                  << overall_success_rate << "% (" << successful_reads << "/"
                  << total_reads << ")" << std::endl;
    }

    // Explicitly cleanup
    hexapod.cleanup();

    // Restore terminal settings
    restore_terminal();

    return 0;
}
