#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>
#include "hexapod.hpp"

// Global flag for termination
static volatile int running = 1;

// Terminal state
static struct termios orig_termios;

// Signal handler
static void test_mpu6050_signal_handler(int sig)
{
    running = 0;
    printf("\nReceived signal %d, exiting...\n", sig);
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
    }

    // Set stdin to non-blocking mode
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags == -1)
    {
        fprintf(stderr, "Warning: Failed to get file flags\n");
    }
    else if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        fprintf(stderr, "Warning: Failed to set non-blocking mode\n");
    }
}

// Restore terminal to original state
static void restore_terminal()
{
    // Restore original terminal settings
    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios) < 0)
    {
        fprintf(stderr, "Warning: Failed to restore terminal attributes\n");
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
        if (intervals[i] < min)
            min = intervals[i];
        if (intervals[i] > max)
            max = intervals[i];
        sum += intervals[i];
    }

    // Calculate average
    avg = sum / count;

    // Calculate standard deviation
    for (int i = 0; i < count; i++)
    {
        std_dev += (intervals[i] - avg) * (intervals[i] - avg);
    }
    std_dev = sqrt(std_dev / count);

    printf("\nIMU Timing Statistics (ms):\n");
    printf("  Min: %.2f | Max: %.2f | Avg: %.2f | Std Dev: %.2f | Samples: %d\n",
           min * 1000.0, max * 1000.0, avg * 1000.0, std_dev * 1000.0, count);
}

int main(void)
{
    // Set up signal handler for clean termination
    signal(SIGINT, test_mpu6050_signal_handler);
    signal(SIGTERM, test_mpu6050_signal_handler);

    // Set up terminal
    setup_terminal();

    // Initialize hexapod with retry logic
    Hexapod hexapod;
    bool initialized = false;
    int retries = 3;

    printf("MPU6050 Test - Press Ctrl+C to exit\n");
    printf("================================\n");

    while (!initialized && retries-- > 0)
    {
        printf("Initializing hexapod hardware (attempt %d)...\n", 3 - retries);
        if (hexapod.init())
        {
            initialized = true;
            printf("Hexapod initialized successfully!\n");
        }
        else
        {
            fprintf(stderr, "Failed to initialize: %s\n", hexapod.getLastErrorMessage().c_str());
            if (retries > 0)
            {
                printf("Retrying in 1 second...\n");
                sleep(1);
            }
            else
            {
                fprintf(stderr, "Maximum retries reached, giving up.\n");
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

    printf("\nReading IMU data...\n");
    printf("Press 'q' to quit, 's' to show statistics\n\n");

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
                printf("\nUser requested exit\n");
                break;
            }
            else if (key == 's' || key == 'S')
            {
                print_timing_stats(read_intervals, (total_reads > 100) ? 100 : total_reads);
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
            hexapod.printImuData(imuData);
            fflush(stdout);
            last_successful_read = now;
            successful_reads++;
        }
        else
        {
            if (now - last_successful_read > 5.0) // 5 second timeout
            {
                fprintf(stderr, "\nNo response from IMU for 5 seconds, exiting...\n");
                break;
            }
            fprintf(stderr, "\rFailed to read IMU: %s", hexapod.getLastErrorMessage().c_str());
        }

        // Show statistics periodically
        if (now - last_stats_time > 10.0)
        {
            printf("\n\nSuccess rate: %.1f%% (%d/%d) in last 10 seconds\n",
                   (successful_reads * 100.0) / total_reads, successful_reads, total_reads);
            last_stats_time = now;
        }

        // Brief sleep to prevent excessive CPU usage - dynamically adjust based on performance
        usleep(elapsed < 0.005 ? 50000 : 10000); // Sleep longer if reads are fast
    }

    printf("\n\nExiting MPU6050 test program...\n");

    // Print final statistics
    print_timing_stats(read_intervals, (total_reads > 100) ? 100 : total_reads);
    printf("Overall success rate: %.1f%% (%d/%d)\n",
           (successful_reads * 100.0) / total_reads, successful_reads, total_reads);

    // Explicitly cleanup
    hexapod.cleanup();

    // Restore terminal settings
    restore_terminal();

    return 0;
}
