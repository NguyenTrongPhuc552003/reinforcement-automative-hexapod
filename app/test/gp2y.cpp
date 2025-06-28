#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <string>
#include <vector>
#include <atomic>
#include "sharpsensor.hpp"
#include "common.hpp"

// Global running flag for signal handling
static std::atomic<bool> running(true);

// Signal handler using common utilities
void signalHandler(int signal)
{
    running.store(false);
    common::ErrorReporter::reportInfo("GP2Y-Test", "Received termination signal " + std::to_string(signal));
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

    bar += "]";
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
    SINGLE,     // Single measurement mode
    CONTINUOUS, // Continuous measurement mode
    AVERAGE     // Average of multiple measurements
};

// Program state machine
enum class ProgramState
{
    NORMAL,               // Normal operation (handling commands)
    AWAITING_MEASUREMENT, // Waiting for Enter to measure in SINGLE mode
    CALIBRATION_PROMPT,   // Showing calibration prompt
    CALIBRATION_ACTIVE,   // Actively calibrating
    EXIT_REQUESTED        // Program is about to exit
};

/**
 * @brief Process input in normal mode
 */
void processNormalInput(char key, TestMode &current_mode, ProgramState &program_state,
                        int &num_samples, bool &filtering_enabled, bool &verbose_mode,
                        std::string &status_message, SharpSensor &sensor);

/**
 * @brief Take a single measurement
 */
void takeSingleMeasurement(SharpSensor &sensor, TestMode &current_mode, ProgramState &program_state,
                           std::vector<float> &measurements, float &min_distance,
                           float &max_distance, float &sum_distance,
                           int &valid_readings, int &error_readings);

/**
 * @brief Update the current measurement mode
 */
void updateCurrentMode(TestMode current_mode, SharpSensor &sensor,
                       std::vector<float> &measurements, float &min_distance,
                       float &max_distance, float &sum_distance,
                       int &valid_readings, int &error_readings,
                       bool verbose_mode, int num_samples);

/**
 * @brief Process sensor calibration
 *
 * @return true if status needs to be shown
 */
bool processSensorCalibration(SharpSensor &sensor, int &cal_samples, float &cal_avg,
                              const int REQUIRED_SAMPLES, ProgramState &program_state,
                              std::string &status_message);

int main(int argc, char *argv[])
{
    // Setup graceful shutdown handling using common utilities
    common::SignalManager::setupGracefulShutdown(running, signalHandler);

    // Initialize performance monitoring
    common::PerformanceMonitor perfMonitor;

    // Setup terminal for non-blocking input using common utilities
    if (!common::TerminalManager::setupNonBlocking())
    {
        common::ErrorReporter::reportWarning("GP2Y-Test", "Failed to setup non-blocking terminal");
    }

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
    perfMonitor.startFrame();
    if (!sensor.init())
    {
        common::ErrorReporter::reportError("GP2Y-Test", "Sensor Initialization", "Failed to initialize sensor");
        common::TerminalManager::restore();
        return 1;
    }
    perfMonitor.endFrame();

    common::ErrorReporter::reportInfo("GP2Y-Test", "Sensor initialized successfully in " +
                                                       common::StringUtils::formatNumber(perfMonitor.getAverageFrameTime()) + "ms");

    // Improved state variables
    TestMode current_mode = TestMode::SINGLE;
    ProgramState program_state = ProgramState::NORMAL;
    int num_samples = 5;
    bool verbose_mode = false;
    bool filtering_enabled = true;
    char key = 0;

    // Calibration variables
    float cal_avg = 0;
    int cal_samples = 0;
    const int REQUIRED_CAL_SAMPLES = 10;

    // Timing variables using common utilities
    unsigned long current_time = 0;
    unsigned long last_prompt_time = 0;
    unsigned long last_blink_time = 0;
    bool blink_state = false;

    // Message display variables
    std::string status_message = "";
    bool show_status = false;

    // Enable filtering by default
    sensor.enableFiltering(filtering_enabled);

    // Print help initially
    print_help();
    std::cout << "\nCurrent mode: Single measurement. Press '1-3' to change modes, 'h' for help.\n";

    // Initial single measurement test with performance monitoring
    std::cout << "\nPerforming initial test measurement...\n";
    perfMonitor.startFrame();
    float distance = sensor.readDistanceCm();
    perfMonitor.endFrame();

    if (distance > 0)
    {
        std::cout << "Test measurement successful! Distance: " << distance << " cm\n";
        std::cout << "Measurement took " << common::StringUtils::formatNumber(perfMonitor.getAverageFrameTime()) << "ms\n";
        status_message = "Ready for measurements. Press Enter to measure in SINGLE mode.";
        show_status = true;
    }
    else
    {
        common::ErrorReporter::reportError("GP2Y-Test", "Initial Test", "Sensor not responding. Check connections.");
        status_message = "Sensor not responding. Check connections and try again.";
        show_status = true;
    }

    // Statistics tracking
    std::vector<float> measurements;
    float min_distance = 999.0f;
    float max_distance = 0.0f;
    float sum_distance = 0.0f;
    int valid_readings = 0;
    int error_readings = 0;

    // Performance monitoring for main loop
    common::PerformanceMonitor loopMonitor;
    unsigned long frame_count = 0;

    // Main loop
    while (running.load())
    {
        loopMonitor.startFrame();

        // Update current time using common utilities
        current_time = common::getCurrentTimeMs();

        // Process user input using common terminal utilities
        if (common::TerminalManager::readChar(key))
        {
            // Process input based on current program state
            switch (program_state)
            {
            case ProgramState::NORMAL:
                processNormalInput(key, current_mode, program_state, num_samples,
                                   filtering_enabled, verbose_mode, status_message, sensor);
                break;

            case ProgramState::AWAITING_MEASUREMENT:
                if (key == '\n')
                {
                    takeSingleMeasurement(sensor, current_mode, program_state, measurements,
                                          min_distance, max_distance, sum_distance,
                                          valid_readings, error_readings);
                }
                else if (key == 'q')
                {
                    running.store(false);
                }
                else
                {
                    program_state = ProgramState::NORMAL;
                    processNormalInput(key, current_mode, program_state, num_samples,
                                       filtering_enabled, verbose_mode, status_message, sensor);
                }
                break;

            case ProgramState::CALIBRATION_PROMPT:
                if (key == '\n')
                {
                    program_state = ProgramState::CALIBRATION_ACTIVE;
                    status_message = "Calibrating... hold sensor still at 20cm";
                    show_status = true;
                    cal_samples = 0;
                    cal_avg = 0;
                }
                else if (key == 'q' || key == 27)
                {
                    program_state = ProgramState::NORMAL;
                    status_message = "Calibration cancelled";
                    show_status = true;
                }
                break;

            case ProgramState::CALIBRATION_ACTIVE:
                if (key == 'q' || key == 27)
                {
                    program_state = ProgramState::NORMAL;
                    status_message = "Calibration cancelled";
                    show_status = true;
                }
                break;

            case ProgramState::EXIT_REQUESTED:
                break;
            }

            if (key == 'q' && program_state != ProgramState::EXIT_REQUESTED)
            {
                std::cout << "\nExiting...\n";
                program_state = ProgramState::EXIT_REQUESTED;
                running.store(false);
            }
        }

        // Update program based on current state
        switch (program_state)
        {
        case ProgramState::NORMAL:
            updateCurrentMode(current_mode, sensor, measurements, min_distance, max_distance,
                              sum_distance, valid_readings, error_readings, verbose_mode, num_samples);

            if (current_mode == TestMode::SINGLE &&
                (current_time - last_prompt_time > 3000))
            {
                std::cout << "\rPress Enter to take a measurement...  " << std::flush;
                last_prompt_time = current_time;
                program_state = ProgramState::AWAITING_MEASUREMENT;
            }
            break;

        case ProgramState::AWAITING_MEASUREMENT:
            if (current_time - last_blink_time > 500)
            {
                blink_state = !blink_state;
                last_blink_time = current_time;
                std::cout << "\rPress Enter to measure" << (blink_state ? "..." : "   ") << std::flush;
            }
            break;

        case ProgramState::CALIBRATION_PROMPT:
            if (current_time - last_blink_time > 500)
            {
                blink_state = !blink_state;
                last_blink_time = current_time;
                std::cout << "\rPlace sensor at 20cm and press Enter" << (blink_state ? "..." : "   ") << std::flush;
            }
            break;

        case ProgramState::CALIBRATION_ACTIVE:
            if (processSensorCalibration(sensor, cal_samples, cal_avg, REQUIRED_CAL_SAMPLES,
                                         program_state, status_message))
            {
                show_status = true;
            }
            break;

        case ProgramState::EXIT_REQUESTED:
            break;
        }

        // Show status message if needed
        if (show_status)
        {
            std::cout << "\n"
                      << status_message << std::endl;
            show_status = false;
        }

        loopMonitor.endFrame();
        frame_count++;

        // Report performance every 1000 frames
        if (frame_count % 1000 == 0)
        {
            loopMonitor.printReport("Main loop ");
        }

        // Use common sleep utility
        common::sleepMs(30);
    }

    // Print final statistics using common string utilities
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

        std::cout << "\n  Min distance: " << common::StringUtils::formatNumber(min_distance, 1) << " cm";
        std::cout << "\n  Max distance: " << common::StringUtils::formatNumber(max_distance, 1) << " cm";
        std::cout << "\n  Avg distance: " << common::StringUtils::formatNumber(avg_distance, 1) << " cm";
        std::cout << "\n  Std deviation: " << common::StringUtils::formatNumber(std_dev, 2) << " cm";
    }

    // Final performance report
    loopMonitor.printReport("Final performance ");

    std::cout << "\n\nExiting GP2Y0A21YK0F test program...\n";

    // Restore terminal using common utilities
    common::TerminalManager::restore();
    return 0;
}

/**
 * @brief Process input in normal mode
 */
void processNormalInput(char key, TestMode &current_mode, ProgramState &program_state,
                        int &num_samples, bool &filtering_enabled, bool &verbose_mode,
                        std::string &status_message, SharpSensor &sensor)
{
    // Validate inputs using common utilities
    if (!common::Validator::clamp(num_samples, 1, 20))
    {
        common::ErrorReporter::reportWarning("GP2Y-Test", "Invalid sample count, clamping to valid range");
    }

    switch (key)
    {
    case '1':
        current_mode = TestMode::SINGLE;
        status_message = "Switched to Single measurement mode. Press Enter to measure.";
        program_state = ProgramState::AWAITING_MEASUREMENT;
        break;

    case '2':
        current_mode = TestMode::CONTINUOUS;
        status_message = "Switched to Continuous measurement mode.";
        break;

    case '3':
        current_mode = TestMode::AVERAGE;
        status_message = "Switched to Average measurement mode. Using " +
                         std::to_string(num_samples) + " samples.";
        break;

    case 'f':
        filtering_enabled = !filtering_enabled;
        sensor.enableFiltering(filtering_enabled);
        status_message = "Filtering " + std::string(filtering_enabled ? "enabled" : "disabled");
        break;

    case '+':
        if (num_samples < 20)
        {
            num_samples++;
            status_message = "Increased samples to " + std::to_string(num_samples);
        }
        break;

    case '-':
        if (num_samples > 1)
        {
            num_samples--;
            status_message = "Decreased samples to " + std::to_string(num_samples);
        }
        break;

    case 'r':
        status_message = "Recalibrating sensor. Place sensor at exactly 20cm from object.";
        program_state = ProgramState::CALIBRATION_PROMPT;
        break;

    case 'v':
        verbose_mode = !verbose_mode;
        status_message = "Verbose output " + std::string(verbose_mode ? "enabled" : "disabled");
        break;

    case 'h':
        print_help();
        status_message = "Help displayed. Press key to continue.";
        break;

    case '\n':
        if (current_mode == TestMode::SINGLE)
        {
            program_state = ProgramState::AWAITING_MEASUREMENT;
        }
        break;
    }
}

/**
 * @brief Take a single measurement
 */
void takeSingleMeasurement(SharpSensor &sensor, TestMode &current_mode, ProgramState &program_state,
                           std::vector<float> &measurements, float &min_distance,
                           float &max_distance, float &sum_distance,
                           int &valid_readings, int &error_readings)
{
    // Only for single mode
    if (current_mode == TestMode::SINGLE)
    {
        float single_distance = sensor.readDistanceCm();
        if (single_distance > 0)
        {
            std::cout << "\nDistance: " << std::fixed << std::setprecision(1)
                      << single_distance << " cm   "
                      << render_distance_bar(single_distance, 80.0f, 40) << "\n";
            // Update statistics
            min_distance = std::min(min_distance, single_distance);
            max_distance = std::max(max_distance, single_distance);
            sum_distance += single_distance;
            valid_readings++;
            measurements.push_back(single_distance);

            // Limit measurements buffer size
            if (measurements.size() > 100)
            {
                measurements.erase(measurements.begin());
            }
        }
        else
        {
            std::cout << "\nError reading sensor\n";
            error_readings++;
        }

        // Return to normal state with prompt to measure again
        program_state = ProgramState::NORMAL;
        std::cout << "\nPress Enter for another measurement, or select another mode." << std::endl;
    }
}

/**
 * @brief Update the current measurement mode
 */
void updateCurrentMode(TestMode current_mode, SharpSensor &sensor,
                       std::vector<float> &measurements, float &min_distance,
                       float &max_distance, float &sum_distance,
                       int &valid_readings, int &error_readings,
                       bool verbose_mode, int num_samples)
{
    // Process current mode (except SINGLE which is handled via key input)
    switch (current_mode)
    {
    case TestMode::SINGLE:
        // Handled through key input
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
    }
    break;

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

        // Use a larger delay for average mode
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    break;
    }
}

/**
 * @brief Process sensor calibration
 *
 * @return true if status needs to be shown
 */
bool processSensorCalibration(SharpSensor &sensor, int &cal_samples, float &cal_avg,
                              const int REQUIRED_SAMPLES, ProgramState &program_state,
                              std::string &status_message)
{
    // Take a sample for calibration
    int raw_value = sensor.readRawValue();
    if (raw_value > 0)
    {
        cal_avg += raw_value;
        cal_samples++;

        // Show progress
        std::cout << "\rCalibrating: " << cal_samples << "/" << REQUIRED_SAMPLES
                  << " samples collected..." << std::flush;

        // When we have enough samples, finish calibration
        if (cal_samples >= REQUIRED_SAMPLES)
        {
            cal_avg /= cal_samples;

            // Calculate and apply scale factor
            float expected_value = 20.0f;
            float new_scale = cal_avg * expected_value / 20.0f;
            sensor.setScale(new_scale);

            status_message = "Calibration complete! Scale factor set to: " +
                             std::to_string(new_scale);
            program_state = ProgramState::NORMAL;
            return true;
        }
    }
    else
    {
        std::cout << "\rError reading sensor during calibration. Retrying...       " << std::flush;
    }

    // Add a small delay between calibration readings
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return false;
}
