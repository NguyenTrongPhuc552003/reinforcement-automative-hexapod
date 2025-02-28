#include <stdio.h>
#include <unistd.h>
#include "hexapod.h"

static void test_single_servo(uint8_t leg_num, double hip, double knee, double ankle)
{
    leg_position_t position = {
        .hip = hip,
        .knee = knee,
        .ankle = ankle};

    printf("Setting leg %d to position:    hip=%.1f, knee=%.1f, ankle=%.1f\n",
           leg_num, hip, knee, ankle);

    if (hexapod_set_leg_position(leg_num, &position) < 0)
    {
        printf("Failed to set leg position\n");
        return;
    }

    // Wait to observe movement
    sleep(2);

    // Read back position to verify
    leg_position_t current;
    if (hexapod_get_leg_position(leg_num, &current) == 0)
    {
        printf("Getting current leg position: hip=%.1f, knee=%.1f, ankle=%.1f\n",
               current.hip, current.knee, current.ankle);
    }
}

int main(void)
{
    printf("Starting servo test...\n");

    if (hexapod_init() < 0)
    {
        printf("Failed to initialize hexapod\n");
        return 1;
    }

    // Test sequence
    printf("\nTesting individual servos...\n");
    for (int i = 0; i < NUM_LEGS; i++)
    {
        test_single_servo(i, 45.0, 0.0, 0.0); // Move hip
        test_single_servo(i, 0.0, 45.0, 0.0); // Move knee
        test_single_servo(i, 0.0, 0.0, 45.0); // Move ankle
        printf("\n");
        sleep(1);
    }

    hexapod_cleanup();
    printf("\nServo test completed\n");
    return 0;
}