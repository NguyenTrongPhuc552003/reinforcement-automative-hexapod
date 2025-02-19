#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include "../include/hexapod_ioctl_user.h"

#define DEVICE_PATH "/dev/hexapod"
#define SAMPLE_RATE_MS 100  // 10Hz sampling rate

static volatile int g_running = 1;
static int g_fd = -1;

// Signal handler for clean exit
void signal_handler(int signo) {
    if (signo == SIGINT) {
        printf("\nReceived SIGINT. Stopping...\n");
        g_running = 0;
    }
}

// Convert raw accelerometer data to g force
float convert_accel(int16_t raw) {
    return (float)raw / 16384.0f;  // ±2g range
}

// Convert raw gyroscope data to degrees per second
float convert_gyro(int16_t raw) {
    return (float)raw / 131.0f;    // ±250°/s range
}

// Convert raw temperature data to Celsius
float convert_temp(int16_t raw) {
    return (float)raw / 340.0f + 36.53f;
}

// Calculate pitch and roll from accelerometer data
void calculate_angles(float ax, float ay, float az, float *pitch, float *roll) {
    *pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
    *roll = atan2(ay, az) * 180.0f / M_PI;
}

int main(int argc, char *argv[]) {
    struct mpu6050_data data;
    float ax, ay, az;  // Accelerometer data in g
    float gx, gy, gz;  // Gyroscope data in °/s
    float pitch, roll; // Orientation angles
    float temp;        // Temperature in °C
    int continuous = 0;
    int raw = 0;
    int opt;

    // Parse command line options
    while ((opt = getopt(argc, argv, "cr")) != -1) {
        switch (opt) {
            case 'c':
                continuous = 1;
                break;
            case 'r':
                raw = 1;
                break;
            default:
                fprintf(stderr, "Usage: %s [-c] [-r]\n", argv[0]);
                fprintf(stderr, "  -c  Continuous monitoring\n");
                fprintf(stderr, "  -r  Show raw values\n");
                exit(EXIT_FAILURE);
        }
    }

    // Set up signal handler
    signal(SIGINT, signal_handler);

    // Open device
    g_fd = open(DEVICE_PATH, O_RDWR);
    if (g_fd < 0) {
        perror("Failed to open device");
        return EXIT_FAILURE;
    }

    printf("MPU6050 Test Program\n");
    if (continuous) {
        printf("Press Ctrl+C to stop\n\n");
    }

    do {
        // Read sensor data
        if (ioctl(g_fd, IOCTL_GET_MPU6050, &data) < 0) {
            perror("Failed to read MPU6050");
            break;
        }

        if (raw) {
            // Show raw values
            printf("\rAcc: X=%6d Y=%6d Z=%6d | Gyro: X=%6d Y=%6d Z=%6d | Temp=%6d   ",
                   data.accel_x, data.accel_y, data.accel_z,
                   data.gyro_x, data.gyro_y, data.gyro_z,
                   data.temp);
            fflush(stdout);
        } else {
            // Convert and calculate
            ax = convert_accel(data.accel_x);
            ay = convert_accel(data.accel_y);
            az = convert_accel(data.accel_z);
            gx = convert_gyro(data.gyro_x);
            gy = convert_gyro(data.gyro_y);
            gz = convert_gyro(data.gyro_z);
            temp = convert_temp(data.temp);
            calculate_angles(ax, ay, az, &pitch, &roll);

            // Display processed data
            if (continuous) {
                printf("\rAcc(g): X=%6.2f Y=%6.2f Z=%6.2f | Gyro(°/s): X=%6.1f Y=%6.1f Z=%6.1f | "
                       "Pitch=%6.1f° Roll=%6.1f° | Temp=%5.1f°C   ",
                       ax, ay, az, gx, gy, gz, pitch, roll, temp);
                fflush(stdout);
            } else {
                printf("Accelerometer (g):\n");
                printf("  X: %7.3f\n  Y: %7.3f\n  Z: %7.3f\n\n", ax, ay, az);
                printf("Gyroscope (°/s):\n");
                printf("  X: %7.2f\n  Y: %7.2f\n  Z: %7.2f\n\n", gx, gy, gz);
                printf("Orientation:\n");
                printf("  Pitch: %7.2f°\n  Roll:  %7.2f°\n\n", pitch, roll);
                printf("Temperature: %.1f°C\n", temp);
            }
        }

        if (continuous) {
            usleep(SAMPLE_RATE_MS * 1000);
        }
    } while (continuous && g_running);

    if (continuous) {
        printf("\n\nTest program terminated\n");
    }

    close(g_fd);
    return EXIT_SUCCESS;
}
