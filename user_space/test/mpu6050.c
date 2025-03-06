#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include "hexapod.h"

/* Global flag for termination */
static volatile int running = 1;

/* Signal handler */
void handle_signal(int sig)
{
    printf("\nReceived signal %d, shutting down...\n", sig);
    running = 0;
}

/* Convert raw accelerometer data to g's */
float accel_to_g(int16_t raw)
{
    return raw / 16384.0f;  /* For ±2g range */
}

/* Convert raw gyroscope data to degrees/second */
float gyro_to_dps(int16_t raw)
{
    return raw / 65.5f;  /* For ±500°/s range */
}

int main(void)
{
    hexapod_t hex;
    imu_data_t imu_data;
    int ret;
    
    /* Set up signal handler for clean termination */
    signal(SIGINT, handle_signal);
    
    /* Initialize hexapod */
    ret = hexapod_init(&hex);
    if (ret < 0) {
        fprintf(stderr, "Failed to initialize hexapod: %s\n", 
                hexapod_error_string(ret));
        return 1;
    }
    
    printf("IMU Test - Press Ctrl+C to exit\n");
    printf("================================\n");
    
    /* Main loop */
    while (running) {
        /* Get IMU data */
        ret = hexapod_get_imu_data(&hex, &imu_data);
        if (ret < 0) {
            fprintf(stderr, "Failed to read IMU: %s\n", 
                    hexapod_error_string(ret));
            break;
        }
        
        /* Print formatted IMU data */
        printf("\rAccel: X=%+6.2fg Y=%+6.2fg Z=%+6.2fg | Gyro: X=%+7.2f° Y=%+7.2f° Z=%+7.2f°/s",
               accel_to_g(imu_data.accel_x),
               accel_to_g(imu_data.accel_y),
               accel_to_g(imu_data.accel_z),
               gyro_to_dps(imu_data.gyro_x),
               gyro_to_dps(imu_data.gyro_y),
               gyro_to_dps(imu_data.gyro_z));
        fflush(stdout);
        
        /* Short delay */
        usleep(100000); /* 100ms */
    }
    
    printf("\n\nExiting...\n");
    
    /* Cleanup */
    hexapod_cleanup(&hex);
    return 0;
}
