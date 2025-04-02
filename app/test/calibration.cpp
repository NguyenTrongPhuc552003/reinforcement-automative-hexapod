#include <iostream>
#include <iomanip>
#include <string>
#include <csignal>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "calibration.hpp" // This now has the authoritative Calibration definition
#include "controller.hpp"  // Include controller for non-blocking input handling

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
        terminal_modified = false;
    }
}

// Restore terminal to original state
static void restore_terminal()
{
    if (terminal_modified)
    {
        // Restore original terminal settings
        if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios) < 0)
        {
            std::cerr << "Warning: Failed to restore terminal attributes" << std::endl;
        }
        terminal_modified = false;
    }
}

// Test loading and saving calibration
static bool test_load_save_calibration()
{
    std::cout << "Testing calibration loading and saving..." << std::endl;

    // Generate test calibration data
    std::vector<Calibration> testCalibrations(NUM_LEGS);
    for (int i = 0; i < NUM_LEGS; i++)
    {
        testCalibrations[i].leg_num = i;
        testCalibrations[i].hip_offset = 5 * (i % 3);         // 0, 5, 10, 0, 5, 10
        testCalibrations[i].knee_offset = -3 * ((i + 1) % 3); // -3, -6, 0, -3, -6, 0
        testCalibrations[i].ankle_offset = (i % 2) ? 7 : -7;  // -7, 7, -7, 7, -7, 7
    }

    std::cout << "Saving test calibration data..." << std::endl;
    if (!CalibrationManager::saveCalibration(testCalibrations))
    {
        std::cerr << "Failed to save calibration data" << std::endl;
        return false;
    }

    // Clear and reload
    std::vector<Calibration> loadedCalibrations;
    std::cout << "Loading calibration data..." << std::endl;
    if (!CalibrationManager::loadCalibration(loadedCalibrations))
    {
        std::cerr << "Failed to load calibration data" << std::endl;
        return false;
    }

    // Verify data matches
    if (loadedCalibrations.size() != NUM_LEGS)
    {
        std::cerr << "Invalid number of calibrations loaded: " << loadedCalibrations.size() << std::endl;
        return false;
    }

    bool match = true;
    for (int i = 0; i < NUM_LEGS; i++)
    {
        if (loadedCalibrations[i].leg_num != testCalibrations[i].leg_num ||
            loadedCalibrations[i].hip_offset != testCalibrations[i].hip_offset ||
            loadedCalibrations[i].knee_offset != testCalibrations[i].knee_offset ||
            loadedCalibrations[i].ankle_offset != testCalibrations[i].ankle_offset)
        {
            std::cerr << "Mismatch in calibration data for leg " << i << std::endl;
            match = false;
        }
    }

    if (match)
    {
        std::cout << "Calibration data saved and loaded successfully" << std::endl;
    }

    return match;
}

// Apply calibration to the hexapod
static bool test_apply_calibration(Hexapod &hexapod)
{
    std::cout << "Testing calibration application to hexapod..." << std::endl;

    std::vector<Calibration> calibrations;
    if (!CalibrationManager::loadCalibration(calibrations))
    {
        std::cerr << "Failed to load calibration data" << std::endl;
        return false;
    }

    // Apply calibration to each leg with better error handling
    size_t success_count = 0;
    for (const auto &cal : calibrations)
    {
        std::cout << "Applying calibration to leg " << static_cast<int>(cal.leg_num)
                  << ": hip=" << cal.hip_offset
                  << ", knee=" << cal.knee_offset
                  << ", ankle=" << cal.ankle_offset << std::endl;

        // Added retry logic for calibration application
        bool success = false;
        for (int retry = 0; retry < 3 && !success; retry++)
        {
            if (hexapod.setCalibration(cal.leg_num, cal.hip_offset, cal.knee_offset, cal.ankle_offset))
            {
                success = true;
                success_count++;
            }
            else if (retry < 2)
            {
                std::cerr << "Retrying calibration for leg " << static_cast<int>(cal.leg_num)
                          << " (attempt " << retry + 2 << ")..." << std::endl;
                usleep(100000); // 100ms delay between retries
            }
        }

        if (!success)
        {
            std::cerr << "Failed to apply calibration to leg " << static_cast<int>(cal.leg_num)
                      << ": " << hexapod.getLastErrorMessage() << std::endl;
        }

        // Short delay between legs
        usleep(100000);
    }

    std::cout << success_count << " of " << calibrations.size()
              << " legs calibrated successfully" << std::endl;
    return (success_count == calibrations.size());
}

// Test with visual verification
static bool test_visual_calibration(Hexapod &hexapod)
{
    std::cout << "Testing calibration with visual verification..." << std::endl;
    std::cout << "This will move each leg to show calibration effect" << std::endl;

    // Center all legs first
    std::cout << "Centering all legs..." << std::endl;
    if (!hexapod.centerAll())
    {
        std::cerr << "Failed to center legs" << std::endl;
        return false;
    }
    sleep(1);

    // For each leg, move to show calibration effect
    for (int leg = 0; leg < NUM_LEGS && running; leg++)
    {
        std::cout << "Testing leg " << leg << " calibration effect..." << std::endl;

        // Move to 15 degrees on all joints to show calibration
        LegPosition pos(15, 15, 15);
        if (!hexapod.setLegPosition(leg, pos))
        {
            std::cerr << "Failed to set leg position: "
                      << hexapod.getLastErrorMessage() << std::endl;
            continue;
        }

        sleep(1);

        // Center this leg
        LegPosition center(0, 0, 0);
        hexapod.setLegPosition(leg, center);
        usleep(500000);
    }

    // Center all legs again
    std::cout << "Centering all legs..." << std::endl;
    hexapod.centerAll();

    return true;
}

// Reset to default calibration
static bool reset_calibration(Hexapod &hexapod)
{
    std::cout << "Resetting to default calibration..." << std::endl;

    std::vector<Calibration> defaults = CalibrationManager::getDefaultCalibration();

    // Save defaults to file
    if (!CalibrationManager::saveCalibration(defaults))
    {
        std::cerr << "Failed to save default calibration" << std::endl;
        return false;
    }

    // Apply to hexapod
    for (const auto &cal : defaults)
    {
        if (!hexapod.setCalibration(cal.leg_num, cal.hip_offset, cal.knee_offset, cal.ankle_offset))
        {
            std::cerr << "Failed to apply default calibration to leg " << static_cast<int>(cal.leg_num)
                      << ": " << hexapod.getLastErrorMessage() << std::endl;
            return false;
        }
    }

    std::cout << "Calibration reset to defaults" << std::endl;
    return true;
}

// Main program
int main(int argc, char *argv[])
{
    // Set signal handler
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    // Setup terminal
    setup_terminal();

    std::cout << "Hexapod Calibration Test" << std::endl;
    std::cout << "------------------------" << std::endl;

    // Initialize hexapod with retry logic
    Hexapod hexapod;
    bool initialized = false;
    int retries = 3;

    while (!initialized && retries-- > 0)
    {
        std::cout << "Initializing hexapod hardware (attempt " << 3 - retries << ")..." << std::endl;
        if (hexapod.init())
        {
            initialized = true;
        }
        else
        {
            std::cerr << "Failed to initialize: " << hexapod.getLastErrorMessage() << std::endl;
            if (retries > 0)
            {
                std::cout << "Retrying in 1 second..." << std::endl;
                sleep(1);
            }
        }
    }

    if (!initialized)
    {
        std::cerr << "Maximum retries reached, giving up." << std::endl;
        restore_terminal();
        return 1;
    }

    // Process command line arguments
    if (argc > 1)
    {
        std::string test_type = argv[1];

        if (test_type == "loadsave")
        {
            test_load_save_calibration();
        }
        else if (test_type == "apply")
        {
            test_apply_calibration(hexapod);
        }
        else if (test_type == "visual")
        {
            test_visual_calibration(hexapod);
        }
        else if (test_type == "reset")
        {
            reset_calibration(hexapod);
        }
        else
        {
            std::cout << "Unknown test: " << test_type << std::endl;
            std::cout << "Available tests: loadsave, apply, visual, reset" << std::endl;
        }
    }
    else
    {
        // Run all tests
        std::cout << "Running all calibration tests. Press Ctrl+C to skip to next test." << std::endl;

        std::cout << "\n=== Testing Load/Save Calibration ===" << std::endl;
        test_load_save_calibration();
        if (!running)
            goto cleanup;
        sleep(1);

        std::cout << "\n=== Testing Apply Calibration ===" << std::endl;
        test_apply_calibration(hexapod);
        if (!running)
            goto cleanup;
        sleep(1);

        std::cout << "\n=== Testing Visual Calibration ===" << std::endl;
        test_visual_calibration(hexapod);
        if (!running)
            goto cleanup;
        sleep(1);
    }

cleanup:
    // Center all legs before exit
    std::cout << "\nCentering all legs..." << std::endl;
    hexapod.centerAll();

    // Explicitly cleanup hexapod resources
    std::cout << "Cleaning up hexapod resources..." << std::endl;
    hexapod.cleanup();

    // Restore terminal settings
    restore_terminal();

    std::cout << "Test program completed." << std::endl;
    return 0;
}
