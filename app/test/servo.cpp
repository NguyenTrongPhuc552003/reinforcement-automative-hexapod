#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include "hexapod.hpp"

// Global flags for program control
static volatile int running = 1;
static volatile int emergency_stop = 0;

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
        printf("\nReceived interrupt signal, initiating shutdown...\n");
        running = 0;
    }
    else if (sig == SIGTERM)
    {
        printf("\nReceived termination signal, shutting down immediately...\n");
        emergency_stop = 1;
        running = 0;
    }
}

// Print progress bar
void print_progress(int current, int total, const char *joint_name)
{
    const int bar_width = 40;
    float progress = (float)current / total;
    int pos = bar_width * progress;

    printf("%s [", joint_name);
    for (int i = 0; i < bar_width; ++i)
    {
        if (i < pos)
            printf("=");
        else if (i == pos)
            printf(">");
        else
            printf(" ");
    }
    printf("] %3d%%\r", (int)(progress * 100));
    fflush(stdout);
}

// Get current time for performance measurements
double get_current_time()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + (ts.tv_nsec / 1.0e9);
}

// Test a specific joint with improved error handling and feedback
bool test_joint(Hexapod &hexapod, uint8_t leg_num, int joint_index, const char *joint_name)
{
    printf("\nTesting %s joint of leg %d...\n", joint_name, leg_num);

    double start_time = get_current_time();
    int success_count = 0;
    int error_count = 0;

    // Perform the sweep motion
    for (int i = 0; i < config.sweep_steps && running && !emergency_stop; i++)
    {
        // Calculate angle using sine wave: smooth transition from -range to +range
        double progress = (double)i / config.sweep_steps;
        double angle = sin(progress * M_PI) * config.sweep_range;

        // Create position - only set the target joint's angle
        LegPosition pos(
            joint_index == 0 ? static_cast<int16_t>(angle) : 0,
            joint_index == 1 ? static_cast<int16_t>(angle) : 0,
            joint_index == 2 ? static_cast<int16_t>(angle) : 0);

        // Set position with retry logic
        bool success = false;
        for (int retry = 0; retry < 3 && !success; retry++)
        {
            if (hexapod.setLegPosition(leg_num, pos))
            {
                success = true;
                success_count++;
            }
            else if (retry < 2)
            { // Don't sleep on last retry
                fprintf(stderr, "Retrying position set (%d)...\n", retry + 1);
                usleep(50000); // 50ms delay before retry
            }
        }

        if (!success)
        {
            fprintf(stderr, "Failed to set position: %s\n",
                    hexapod.getLastErrorMessage().c_str());
            error_count++;

            // Skip ahead if too many errors
            if (error_count > 5)
            {
                fprintf(stderr, "Too many errors, skipping remainder of test\n");
                break;
            }
        }

        // Display progress
        print_progress(i, config.sweep_steps, joint_name);

        usleep(config.delay_ms * 1000);
    }

    // Print newline after progress bar
    printf("\n");

    // Return to center position
    LegPosition center(0, 0, 0);
    if (!hexapod.setLegPosition(leg_num, center))
    {
        fprintf(stderr, "Failed to center joint: %s\n",
                hexapod.getLastErrorMessage().c_str());
    }

    double elapsed_time = get_current_time() - start_time;
    printf("%s joint test completed in %.2f seconds with %d successful movements and %d errors\n",
           joint_name, elapsed_time, success_count, error_count);

    usleep(config.transition_ms * 1000);

    return (error_count == 0);
}

int main(int argc, char *argv[])
{
    double total_start_time = get_current_time();
    bool test_success = true;

    // Set up signal handlers for clean termination
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // Process command line arguments with better error checking
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--leg") == 0 && i + 1 < argc)
        {
            int leg = atoi(argv[++i]);
            if (leg >= 0 && leg < NUM_LEGS)
            {
                config.leg_num = leg;
            }
            else
            {
                fprintf(stderr, "Error: Leg number must be 0-%d\n", NUM_LEGS - 1);
                return 1;
            }
        }
        else if (strcmp(argv[i], "--range") == 0 && i + 1 < argc)
        {
            config.sweep_range = atoi(argv[++i]);
            if (config.sweep_range < 0 || config.sweep_range > 90)
            {
                fprintf(stderr, "Warning: Range should be between 0-90 degrees, using %d\n", config.sweep_range);
            }
        }
        else if (strcmp(argv[i], "--steps") == 0 && i + 1 < argc)
        {
            config.sweep_steps = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--delay") == 0 && i + 1 < argc)
        {
            config.delay_ms = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--help") == 0)
        {
            printf("Usage: %s [options]\n", argv[0]);
            printf("Options:\n");
            printf("  --leg N       Test leg number N (0-%d)\n", NUM_LEGS - 1);
            printf("  --range N     Set sweep range to N degrees (default: 45)\n");
            printf("  --steps N     Set number of steps in sweep (default: 100)\n");
            printf("  --delay N     Set delay between steps to N ms (default: 30)\n");
            printf("  --help        Show this help message\n");
            return 0;
        }
        else
        {
            fprintf(stderr, "Unknown argument: %s (use --help for usage)\n", argv[i]);
            return 1;
        }
    }

    // Initialize hexapod with improved error messaging
    printf("Initializing hexapod hardware interface...\n");
    Hexapod hexapod;
    if (!hexapod.init())
    {
        fprintf(stderr, "ERROR: Failed to initialize hexapod hardware\n");
        fprintf(stderr, "Reason: %s\n", hexapod.getLastErrorMessage().c_str());
        fprintf(stderr, "Please check:\n");
        fprintf(stderr, "  1. Is the hexapod driver loaded? (lsmod | grep hexapod)\n");
        fprintf(stderr, "  2. Do you have permission to access the device? (ls -l /dev/hexapod)\n");
        fprintf(stderr, "  3. Is the hardware properly connected? (I2C bus, power supply)\n");
        return 1;
    }

    printf("\n");
    printf("┌───────────────────────────────────────────────┐\n");
    printf("│            SERVO TEST - LEG %-2d              │\n", config.leg_num);
    printf("├───────────────────────────────────────────────┤\n");
    printf("│ Range: ±%-3d° | Steps: %-3d | Delay: %-3d ms  │\n",
           config.sweep_range, config.sweep_steps, config.delay_ms);
    printf("└───────────────────────────────────────────────┘\n");
    printf("\nPress Ctrl+C to exit\n\n");

    // Center all servos to start with timeout handling
    printf("Centering all servos...\n");
    double center_start = get_current_time();
    if (!hexapod.centerAll())
    {
        fprintf(stderr, "Failed to center servos: %s\n",
                hexapod.getLastErrorMessage().c_str());

        printf("Attempting emergency recovery...\n");
        sleep(1);
        if (!hexapod.centerAll())
        {
            fprintf(stderr, "Emergency recovery failed. Exiting.\n");
            hexapod.cleanup();
            return 1;
        }
    }
    printf("Centering completed in %.2f seconds\n", get_current_time() - center_start);

    // Allow servos to reach position
    sleep(1);

    // Main test loop with joint-specific tests
    while (running && !emergency_stop)
    {
        // Test each joint
        if (!test_joint(hexapod, config.leg_num, 0, "Hip"))
        {
            test_success = false;
            if (!running)
                break;
        }

        if (!test_joint(hexapod, config.leg_num, 1, "Knee"))
        {
            test_success = false;
            if (!running)
                break;
        }

        if (!test_joint(hexapod, config.leg_num, 2, "Ankle"))
        {
            test_success = false;
            if (!running)
                break;
        }

        // Test complete, show summary
        printf("\nTest cycle complete!\n");

        // If single run or interrupted, break out
        if (!running || argc > 1)
            break;

        printf("Starting next test cycle in 3 seconds... (Press Ctrl+C to exit)\n");
        sleep(3);
    }

    // Center all servos before exit with visible indicator
    printf("\nShutting down - Centering all servos for safety...\n");
    if (!hexapod.centerAll())
    {
        fprintf(stderr, "Warning: Failed to center servos during shutdown: %s\n",
                hexapod.getLastErrorMessage().c_str());
        // Try one more time
        sleep(1);
        hexapod.centerAll();
    }

    // Report total runtime
    double total_time = get_current_time() - total_start_time;
    printf("Test program completed in %.2f seconds\n", total_time);
    printf("Status: %s\n", test_success ? "SUCCESS" : "ERRORS ENCOUNTERED");

    // Return appropriate exit code
    return test_success ? 0 : 1;
}
