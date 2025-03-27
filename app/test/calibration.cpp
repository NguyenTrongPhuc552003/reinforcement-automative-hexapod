#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "calibration.hpp" // This now has the authoritative Calibration definition
#include "controller.hpp"  // Include controller for non-blocking input handling

// Global flag for program termination
static volatile int running = 1;
static struct termios orig_termios;
static bool terminal_modified = false;

// Signal handler for graceful termination
static void handle_signal(int sig)
{
    printf("\nReceived signal %d, shutting down...\n", sig);
    running = 0;
}

// Setup terminal for non-blocking input
static void setup_terminal()
{
    struct termios new_termios;

    // Get current terminal settings
    if (tcgetattr(STDIN_FILENO, &orig_termios) < 0)
    {
        fprintf(stderr, "Warning: Failed to get terminal attributes\n");
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
        fprintf(stderr, "Warning: Failed to set terminal attributes\n");
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
            fprintf(stderr, "Warning: Failed to restore terminal attributes\n");
        }
        terminal_modified = false;
    }
}

// Test loading and saving calibration
static bool test_load_save_calibration()
{
    printf("Testing calibration loading and saving...\n");

    // Generate test calibration data
    std::vector<Calibration> testCalibrations(NUM_LEGS);
    for (int i = 0; i < NUM_LEGS; i++)
    {
        testCalibrations[i].leg_num = i;
        testCalibrations[i].hip_offset = 5 * (i % 3);         // 0, 5, 10, 0, 5, 10
        testCalibrations[i].knee_offset = -3 * ((i + 1) % 3); // -3, -6, 0, -3, -6, 0
        testCalibrations[i].ankle_offset = (i % 2) ? 7 : -7;  // -7, 7, -7, 7, -7, 7
    }

    printf("Saving test calibration data...\n");
    if (!CalibrationManager::saveCalibration(testCalibrations))
    {
        fprintf(stderr, "Failed to save calibration data\n");
        return false;
    }

    // Clear and reload
    std::vector<Calibration> loadedCalibrations;
    printf("Loading calibration data...\n");
    if (!CalibrationManager::loadCalibration(loadedCalibrations))
    {
        fprintf(stderr, "Failed to load calibration data\n");
        return false;
    }

    // Verify data matches
    if (loadedCalibrations.size() != NUM_LEGS)
    {
        fprintf(stderr, "Invalid number of calibrations loaded: %zu\n", loadedCalibrations.size());
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

            fprintf(stderr, "Mismatch in calibration data for leg %d\n", i);
            match = false;
        }
    }

    if (match)
    {
        printf("Calibration data saved and loaded successfully\n");
    }

    return match;
}

// Apply calibration to the hexapod
static bool test_apply_calibration(Hexapod &hexapod)
{
    printf("Testing calibration application to hexapod...\n");

    std::vector<Calibration> calibrations;
    if (!CalibrationManager::loadCalibration(calibrations))
    {
        fprintf(stderr, "Failed to load calibration data\n");
        return false;
    }

    // Apply calibration to each leg with better error handling
    size_t success_count = 0;
    for (const auto &cal : calibrations)
    {
        printf("Applying calibration to leg %d: hip=%d, knee=%d, ankle=%d\n",
               cal.leg_num, cal.hip_offset, cal.knee_offset, cal.ankle_offset);

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
                fprintf(stderr, "Retrying calibration for leg %d (attempt %d)...\n",
                        cal.leg_num, retry + 2);
                usleep(100000); // 100ms delay between retries
            }
        }

        if (!success)
        {
            fprintf(stderr, "Failed to apply calibration to leg %d: %s\n",
                    cal.leg_num, hexapod.getLastErrorMessage().c_str());
        }

        // Short delay between legs
        usleep(100000);
    }

    printf("%d of %zu legs calibrated successfully\n", success_count, calibrations.size());
    return (success_count == calibrations.size());
}

// Test with visual verification
static bool test_visual_calibration(Hexapod &hexapod)
{
    printf("Testing calibration with visual verification...\n");
    printf("This will move each leg to show calibration effect\n");

    // Center all legs first
    printf("Centering all legs...\n");
    if (!hexapod.centerAll())
    {
        fprintf(stderr, "Failed to center legs\n");
        return false;
    }
    sleep(1);

    // For each leg, move to show calibration effect
    for (int leg = 0; leg < NUM_LEGS && running; leg++)
    {
        printf("Testing leg %d calibration effect...\n", leg);

        // Move to 15 degrees on all joints to show calibration
        LegPosition pos(15, 15, 15);
        if (!hexapod.setLegPosition(leg, pos))
        {
            fprintf(stderr, "Failed to set leg position: %s\n",
                    hexapod.getLastErrorMessage().c_str());
            continue;
        }

        sleep(1);

        // Center this leg
        LegPosition center(0, 0, 0);
        hexapod.setLegPosition(leg, center);
        usleep(500000);
    }

    // Center all legs again
    printf("Centering all legs...\n");
    hexapod.centerAll();

    return true;
}

// Reset to default calibration
static bool reset_calibration(Hexapod &hexapod)
{
    printf("Resetting to default calibration...\n");

    std::vector<Calibration> defaults = CalibrationManager::getDefaultCalibration();

    // Save defaults to file
    if (!CalibrationManager::saveCalibration(defaults))
    {
        fprintf(stderr, "Failed to save default calibration\n");
        return false;
    }

    // Apply to hexapod
    for (const auto &cal : defaults)
    {
        if (!hexapod.setCalibration(cal.leg_num, cal.hip_offset, cal.knee_offset, cal.ankle_offset))
        {
            fprintf(stderr, "Failed to apply default calibration to leg %d: %s\n",
                    cal.leg_num, hexapod.getLastErrorMessage().c_str());
            return false;
        }
    }

    printf("Calibration reset to defaults\n");
    return true;
}

// Main program
int main(int argc, char *argv[])
{
    // Set signal handler
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // Setup terminal
    setup_terminal();

    printf("Hexapod Calibration Test\n");
    printf("------------------------\n");

    // Initialize hexapod with retry logic
    Hexapod hexapod;
    bool initialized = false;
    int retries = 3;

    while (!initialized && retries-- > 0)
    {
        printf("Initializing hexapod hardware (attempt %d)...\n", 3 - retries);
        if (hexapod.init())
        {
            initialized = true;
        }
        else
        {
            fprintf(stderr, "Failed to initialize: %s\n", hexapod.getLastErrorMessage().c_str());
            if (retries > 0)
            {
                printf("Retrying in 1 second...\n");
                sleep(1);
            }
        }
    }

    if (!initialized)
    {
        fprintf(stderr, "Maximum retries reached, giving up.\n");
        restore_terminal();
        return 1;
    }

    // Process command line arguments
    if (argc > 1)
    {
        if (strcmp(argv[1], "loadsave") == 0)
        {
            test_load_save_calibration();
        }
        else if (strcmp(argv[1], "apply") == 0)
        {
            test_apply_calibration(hexapod);
        }
        else if (strcmp(argv[1], "visual") == 0)
        {
            test_visual_calibration(hexapod);
        }
        else if (strcmp(argv[1], "reset") == 0)
        {
            reset_calibration(hexapod);
        }
        else
        {
            printf("Unknown test: %s\n", argv[1]);
            printf("Available tests: loadsave, apply, visual, reset\n");
        }
    }
    else
    {
        // Run all tests
        printf("Running all calibration tests. Press Ctrl+C to skip to next test.\n");

        printf("\n=== Testing Load/Save Calibration ===\n");
        test_load_save_calibration();
        if (!running)
            goto cleanup;
        sleep(1);

        printf("\n=== Testing Apply Calibration ===\n");
        test_apply_calibration(hexapod);
        if (!running)
            goto cleanup;
        sleep(1);

        printf("\n=== Testing Visual Calibration ===\n");
        test_visual_calibration(hexapod);
        if (!running)
            goto cleanup;
        sleep(1);
    }

cleanup:
    // Center all legs before exit
    printf("\nCentering all legs...\n");
    hexapod.centerAll();

    // Explicitly cleanup hexapod resources
    printf("Cleaning up hexapod resources...\n");
    hexapod.cleanup();

    // Restore terminal settings
    restore_terminal();

    printf("Test program completed.\n");
    return 0;
}
