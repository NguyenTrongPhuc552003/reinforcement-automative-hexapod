#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <math.h>
#include "gait.h"
#include "hexapod.h"

static void test_tripod_gait(void)
{
    printf("Testing tripod gait...\n");

    assert(hexapod_init() == 0); // Use real hardware

    gait_params_t params = {
        .step_height = 30.0,
        .step_length = 50.0,
        .cycle_time = 1.0};

    // Test one complete gait cycle
    for (int step = 0; step < 6; step++)
    {
        tripod_gait_step(step, &params);
        usleep(100000); // 100ms delay between steps
    }

    hexapod_cleanup();
    printf("Tripod gait test completed\n");
}

static void test_wave_gait(void)
{
    printf("Testing wave gait...\n");

    assert(hexapod_init() == 0); // Use real hardware

    gait_params_t params = {
        .step_height = 20.0,
        .step_length = 40.0,
        .cycle_time = 2.0};

    // Test one complete gait cycle
    for (int step = 0; step < 12; step++)
    {
        wave_gait_step(step, &params);
        usleep(200000); // 200ms delay between steps
    }

    hexapod_cleanup();
    printf("Wave gait test completed\n");
}

static void test_gait_transition(void)
{
    printf("Testing gait transition...\n");

    assert(hexapod_init() == 0); // Use real hardware

    gait_params_t params = {
        .step_height = 25.0,
        .step_length = 45.0,
        .cycle_time = 1.5};

    // Start with tripod gait
    for (int step = 0; step < 3; step++)
    {
        tripod_gait_step(step, &params);
        usleep(150000);
    }

    // Transition to wave gait
    params.step_height = 20.0;
    params.step_length = 40.0;
    gait_transition_to_wave(&params);

    // Continue with wave gait
    for (int step = 0; step < 6; step++)
    {
        wave_gait_step(step, &params);
        usleep(200000);
    }

    hexapod_cleanup();
    printf("Gait transition test completed\n");
}

int main(void)
{
    printf("Starting gait tests...\n");

    test_tripod_gait();
    test_wave_gait();
    test_gait_transition();

    printf("All gait tests passed!\n");
    return 0;
}