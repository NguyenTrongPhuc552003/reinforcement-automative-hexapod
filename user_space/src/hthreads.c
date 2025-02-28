#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "hthreads.h"

static pthread_t imu_thread;
static pthread_t servo_thread;
static thread_control_t thread_control = {
    .running = 0,
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .cond = PTHREAD_COND_INITIALIZER};

void *imu_reader_thread(void *arg)
{
    imu_thread_args_t *args = (imu_thread_args_t *)arg;
    int sleep_us = 1000000 / args->update_rate_hz;

    while (args->control->running)
    {
        pthread_mutex_lock(&args->control->lock);
        if (hexapod_get_imu_data(args->imu_data) < 0)
        {
            fprintf(stderr, "IMU read error\n");
        }
        pthread_mutex_unlock(&args->control->lock);
        usleep(sleep_us);
    }
    return NULL;
}

void *servo_controller_thread(void *arg)
{
    gait_thread_args_t *args = (gait_thread_args_t *)arg;
    int sleep_us = 1000000 / args->update_rate_hz;

    while (args->control->running)
    {
        pthread_mutex_lock(&args->control->lock);
        // Update servo positions from state
        for (int i = 0; i < NUM_LEGS; i++)
        {
            hexapod_set_leg_position(i, &args->state->legs[i]);
        }
        pthread_mutex_unlock(&args->control->lock);
        usleep(sleep_us);
    }
    return NULL;
}

int hexapod_start_threads(hexapod_state_t *state)
{
    thread_control.running = 1;

    // Create IMU reader thread
    imu_thread_args_t *imu_args = malloc(sizeof(imu_thread_args_t));
    imu_args->control = &thread_control;
    imu_args->imu_data = &state->imu_data;
    imu_args->update_rate_hz = 100; // 100Hz IMU reading

    if (pthread_create(&imu_thread, NULL, imu_reader_thread, imu_args) != 0)
    {
        return -1;
    }

    // Create servo controller thread
    gait_thread_args_t *servo_args = malloc(sizeof(gait_thread_args_t));
    servo_args->control = &thread_control;
    servo_args->state = state;
    servo_args->update_rate_hz = 50; // 50Hz servo updates

    if (pthread_create(&servo_thread, NULL, servo_controller_thread, servo_args) != 0)
    {
        return -1;
    }

    return 0;
}

void hexapod_stop_threads(void)
{
    thread_control.running = 0;
    pthread_join(imu_thread, NULL);
    pthread_join(servo_thread, NULL);
}
