#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include "hexapod.h"

static volatile bool running = true;

static void sigint_handler(int sig)
{
    printf("\nStopping MPU6050 test with signal %d...\n", sig);
    running = false;
}

// Convert raw accelerometer data to g's
static double raw_to_g(int16_t raw)
{
    // ±2g range, 16-bit signed value
    return (double)raw / 16384.0;
}

// Convert raw gyroscope data to degrees/second
static double raw_to_dps(int16_t raw)
{
    // ±500 degrees/sec range, 16-bit signed value
    return (double)raw / 65.5;
}

int main(void)
{
    imu_data_t data;
    int count = 0;

    printf("Starting MPU6050 test...\n");

    // Set up Ctrl+C handler
    signal(SIGINT, sigint_handler);

    if (hexapod_init() < 0)
    {
        printf("Failed to initialize hexapod\n");
        return 1;
    }

    printf("\nReading MPU6050 data (Ctrl+C to stop)...\n");
    printf("\nAccelerometer data in g's, Gyroscope data in degrees/second\n");

    while (running)
    {
        if (hexapod_get_imu_data(&data) < 0)
        {
            printf("Failed to read IMU data\n");
            break;
        }

        // Print every 10th reading (about twice per second)
        if (count++ % 10 == 0)
        {
            printf("\033[2K\r"); // Clear line and return cursor
            printf("Acc (g): X=%.2f Y=%.2f Z=%.2f | Gyro (°/s): X=%.1f Y=%.1f Z=%.1f",
                   raw_to_g(data.accel_x), raw_to_g(data.accel_y), raw_to_g(data.accel_z),
                   raw_to_dps(data.gyro_x), raw_to_dps(data.gyro_y), raw_to_dps(data.gyro_z));
            fflush(stdout);
        }
        usleep(50000); // 50ms delay (20Hz reading rate)
    }

    printf("\nTest completed\n");
    hexapod_cleanup();
    return 0;
}
