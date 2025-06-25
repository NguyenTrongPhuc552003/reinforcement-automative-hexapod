#include <iostream>
#include <iomanip>
#include <csignal>
#include <chrono>
#include <thread>
#include <cmath>
#include <string>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <vector>
#include "sharpsensor.hpp"

// Global flag for program termination
static volatile bool running = true;

// Terminal state
static struct termios orig_termios;
static bool terminal_modified = false;

// Signal handler
static void signal_handler(int sig)
{
    running = false;
    std::cout << "\nReceived signal " << sig << ", exiting..." << std::endl;
}

// Setup terminal for non-blocking input
static void setup_terminal()
{
    // Get current terminal attributes
    if (tcgetattr(STDIN_FILENO, &orig_termios) < 0)
    {
        std::cerr << "Warning: Failed to get terminal attributes" << std::endl;
        return;
    }

    terminal_modified = true;

    // Modify for non-canonical, non-echo mode
    struct termios new_termios = orig_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | IEXTEN);
    new_termios.c_iflag &= ~(ISTRIP | INLCR | ICRNL | IGNCR | IXON | IXOFF);
    new_termios.c_cflag &= ~CSIZE;
    new_termios.c_cflag |= CS8;
    new_termios.c_cc[VMIN] = 0;
    new_termios.c_cc[VTIME] = 0;

    // Apply the modified attributes
    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &new_termios) < 0)
    {
        std::cerr << "Warning: Failed to set terminal attributes" << std::endl;
        terminal_modified = false;
    }

    // Set stdin to non-blocking mode
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags != -1)
    {
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
}

// Restore terminal to original state
static void restore_terminal()
{
    if (terminal_modified)
    {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
    }
}

// Render a distance visualization bar
static std::string render_distance_bar(float distance, float max_range, int width)
{
    std::string bar = "[";
    int position = std::min(width - 1, static_cast<int>((distance / max_range) * width));

    for (int i = 0; i < width; i++)
    {
        if (i == position)
            bar += "O";
        else
            bar += "-";
    }

    bar += "] ";
    return bar;
}

// Display help information
static void print_help()
{
    std::cout << "\nSharp GP2Y0A21YK0F IR Distance Sensor Test - Commands\n"
              << "====================================================\n"
              << "  1: Single measurement mode\n"
              << "  2: Continuous measurement mode\n"
              << "  3: Average measurement mode\n"
              << "  f: Toggle filtering\n"
              << "  +: Increase samples (in average mode)\n"
              << "  -: Decrease samples (in average mode)\n"
              << "  r: Recalibrate sensor\n"
              << "  v: Toggle verbose output\n"
              << "  h: Show this help\n"
              << "  q: Quit\n\n";
}

enum class TestMode
{
    SINGLE,     // Single measurement
    CONTINUOUS, // Continuous measurement
    AVERAGE     // Average of multiple measurements
};

int main(int argc, char *argv[])
{
    // Set up signal handlers for clean termination
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Set up terminal
    setup_terminal();

    // Parse command line arguments for ADC input
    std::string ain_path = "in_voltage0_raw"; // Default ADC input

    if (argc > 1)
    {
        ain_path = argv[1];
    }

    std::cout << "Sharp GP2Y0A21YK0F IR Distance Sensor Test\n"
              << "=========================================\n"
              << "Using ADC input: " << ain_path << "\n";

    std::cout << "\nWiring Instructions for BeagleBone AI:\n"
              << "  - Connect VCC to 5V power (P9.05 or P9.06)\n"
              << "  - Connect GND to ground (P9.01 or P9.02)\n"
              << "  - Connect OUT to ADC input (AIN0-AIN6 on the BeagleBone)\n"
              << "  - Make sure ADC is enabled in device tree\n"
              << "  - Run with sudo if permission errors occur\n"
              << "  - Press Ctrl+C to exit\n";

    // Initialize the sensor
    SharpSensor sensor(ain_path);
    if (!sensor.init())
    {
        std::cerr << "Failed to initialize sensor. Exiting.\n";
        restore_terminal();
        return 1;
    }

    std::cout << "Sensor initialized successfully!\n";

    // Test variables
    TestMode current_mode = TestMode::SINGLE;
    int num_samples = 5;
    bool verbose_mode = false;
    bool filtering_enabled = true;
    char key = 0;

    // Enable filtering by default
    sensor.enableFiltering(filtering_enabled);

    // Print help initially
    print_help();
    std::cout << "\nCurrent mode: Single measurement. Press '1-3' to change modes, 'h' for help.\n";

    // Initial single measurement test
    std::cout << "\nPerforming initial test measurement...\n";
    float distance = sensor.readDistanceCm();
    if (distance > 0)
    {
        std::cout << "Test measurement successful! Distance: " << distance << " cm\n";
    }
    else
    {
        std::cout << "Test measurement failed! Check sensor connections.\n";
        std::cout << "This might indicate wiring issues or incorrect ADC configuration.\n";
    }

    // Statistics tracking
    std::vector<float> measurements;
    float min_distance = 999.0f;
    float max_distance = 0.0f;
    float sum_distance = 0.0f;
    int valid_readings = 0;
    int error_readings = 0;

    // Main loop
    while (running)
    {
        // Handle key input
        if (read(STDIN_FILENO, &key, 1) > 0)
        {
            switch (key)
            {
            case '1':
                current_mode = TestMode::SINGLE;
                std::cout << "\nSwitched to Single measurement mode\n";
                std::cout << "Press Enter to take a single measurement\n";
                break;

            case '2':
                current_mode = TestMode::CONTINUOUS;
                std::cout << "\nSwitched to Continuous measurement mode\n";
                break;

            case '3':
                current_mode = TestMode::AVERAGE;
                std::cout << "\nSwitched to Average measurement mode\n";
                std::cout << "Using " << num_samples << " samples per measurement\n";
                break;

            case 'f':
                filtering_enabled = !filtering_enabled;
                sensor.enableFiltering(filtering_enabled);
                std::cout << "\nFiltering " << (filtering_enabled ? "enabled" : "disabled") << "\n";
                break;

            case '+':
                if (num_samples < 20)
                {
                    num_samples++;
                    std::cout << "\nIncreased samples to " << num_samples << "\n";
                }
                break;

            case '-':
                if (num_samples > 1)
                {
                    num_samples--;
                    std::cout << "\nDecreased samples to " << num_samples << "\n";
                }
                break;

            case 'r':
            { // Add braces to create a scope for the case
                std::cout << "\nRecalibrating sensor...\n";
                // Simple calibration by taking an average reading at known distance
                std::cout << "Place the sensor at exactly 20cm from an object\n";
                std::cout << "Press Enter when ready...";
                while (read(STDIN_FILENO, &key, 1) <= 0 || key != '\n')
                {
                    if (!running)
                        break;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                if (!running)
                    break;

                std::cout << "\nCalibrating...";
                float cal_avg = 0;
                for (int i = 0; i < 10; i++)
                {
                    cal_avg += sensor.readRawValue();
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                cal_avg /= 10;

                // Set scale based on this reading for 20cm
                float expected_value = 20.0f;
                float new_scale = cal_avg * expected_value / 20.0f;
                sensor.setScale(new_scale);
                std::cout << "\nCalibration complete\n";
            } // Close the scope for this case
            break;

            case 'v':
                verbose_mode = !verbose_mode;
                std::cout << "\nVerbose output " << (verbose_mode ? "enabled" : "disabled") << "\n";
                break;

            case 'h':
                print_help();
                break;

            case 'q':
                std::cout << "\nExiting...\n";
                running = false;
                break;

            case '\n':
                // Only trigger measurement in SINGLE mode
                if (current_mode == TestMode::SINGLE)
                {
                    float single_distance = sensor.readDistanceCm();
                    if (single_distance > 0)
                    {
                        std::cout << "Distance: " << std::fixed << std::setprecision(1)
                                  << single_distance << " cm   "
                                  << render_distance_bar(single_distance, 80.0f, 40) << "\n";

                        // Update statistics
                        min_distance = std::min(min_distance, single_distance);
                        max_distance = std::max(max_distance, single_distance);
                        sum_distance += single_distance;
                        valid_readings++;
                        measurements.push_back(single_distance);
                    }
                    else
                    {
                        std::cout << "Error reading sensor\n";
                        error_readings++;
                    }
                }
                break;

            default:
                // Ignore other keys
                break;
            }
        }

        // Process current mode
        switch (current_mode)
        {
        case TestMode::SINGLE:
            // Handled above with Enter key
            break;

        case TestMode::CONTINUOUS:
        {
            float distance = sensor.readDistanceCm();
            if (distance > 0)
            {
                // Update statistics
                min_distance = std::min(min_distance, distance);
                max_distance = std::max(max_distance, distance);
                sum_distance += distance;
                valid_readings++;
                measurements.push_back(distance);

                // Maintain a reasonable buffer size
                if (measurements.size() > 100)
                {
                    measurements.erase(measurements.begin());
                }

                std::cout << "\rDistance: " << std::fixed << std::setprecision(1)
                          << distance << " cm   "
                          << render_distance_bar(distance, 80.0f, 40);

                if (verbose_mode)
                {
                    int raw_value = sensor.readRawValue();
                    std::cout << " RAW: " << raw_value;
                }

                std::cout << std::flush;
            }
            else
            {
                std::cout << "\rError reading sensor                       " << std::flush;
                error_readings++;
            }

            // Use a small delay to prevent flooding
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            break;
        }

        case TestMode::AVERAGE:
        {
            float avg_distance = sensor.readAverageDistanceCm(num_samples);
            if (avg_distance > 0)
            {
                // Update statistics
                min_distance = std::min(min_distance, avg_distance);
                max_distance = std::max(max_distance, avg_distance);
                sum_distance += avg_distance;
                valid_readings++;
                measurements.push_back(avg_distance);

                // Maintain a reasonable buffer size
                if (measurements.size() > 100)
                {
                    measurements.erase(measurements.begin());
                }

                std::cout << "\rAvg Distance (" << num_samples << " samples): " << std::fixed
                          << std::setprecision(1) << avg_distance << " cm   "
                          << render_distance_bar(avg_distance, 80.0f, 40) << std::flush;
            }
            else
            {
                std::cout << "\rError reading sensor                       " << std::flush;
                error_readings++;
            }

            // Use a larger delay for average mode to give feedback time
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            break;
        }
        }
    }

    // Print final statistics
    std::cout << "\n\nSensor Statistics:";
    std::cout << "\n  Total readings: " << (valid_readings + error_readings);
    std::cout << "\n  Valid readings: " << valid_readings;
    std::cout << "\n  Error readings: " << error_readings;

    if (valid_readings > 0)
    {
        float avg_distance = sum_distance / valid_readings;

        // Calculate standard deviation
        float variance = 0.0f;
        for (float m : measurements)
        {
            variance += (m - avg_distance) * (m - avg_distance);
        }
        float std_dev = std::sqrt(variance / measurements.size());

        std::cout << "\n  Min distance: " << min_distance << " cm";
        std::cout << "\n  Max distance: " << max_distance << " cm";
        std::cout << "\n  Avg distance: " << avg_distance << " cm";
        std::cout << "\n  Std deviation: " << std_dev << " cm";
    }

    std::cout << "\n\nExiting GP2Y0A21YK0F test program...\n";
    restore_terminal();
    return 0;
}
