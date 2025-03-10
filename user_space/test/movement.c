#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include "gait.h"

/* Signal flag for graceful termination */
static volatile int running = 1;

/* Signal handler */
static void handle_signal(int sig)
{
    printf("\nReceived signal %d, shutting down...\n", sig);
    running = 0;
}

/* Get current time in seconds */
static double get_time(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + (ts.tv_nsec / 1.0e9);
}

/* Function to test leg movement */
static int test_leg_movement(hexapod_t *hexapod)
{
    leg_position_t pos;
    int leg, result;

    printf("Testing individual leg movement...\n");

    /* Center all legs first */
    printf("Centering all legs...\n");
    hexapod_center_all(hexapod);
    sleep(2);

    /* Test each leg */
    for (leg = 0; leg < NUM_LEGS; leg++)
    {
        printf("Testing leg %d\n", leg);

        /* Move hip joint */
        printf("  Moving hip joint\n");
        for (int angle = -30; angle <= 30 && running; angle += 10)
        {
            pos.hip = angle;
            pos.knee = 0;
            pos.ankle = 0;
            result = hexapod_set_leg_position(hexapod, leg, &pos);
            if (result != 0)
            {
                fprintf(stderr, "Failed to move hip: %s\n",
                        hexapod_error_string(result));
                return result;
            }
            usleep(200000); /* 200ms delay */
        }

        /* Reset hip */
        pos.hip = 0;
        hexapod_set_leg_position(hexapod, leg, &pos);
        usleep(500000); /* 500ms delay */

        /* Move knee joint */
        printf("  Moving knee joint\n");
        for (int angle = 0; angle <= 45 && running; angle += 10)
        {
            pos.hip = 0;
            pos.knee = angle;
            pos.ankle = 0;
            result = hexapod_set_leg_position(hexapod, leg, &pos);
            if (result != 0)
            {
                fprintf(stderr, "Failed to move knee: %s\n",
                        hexapod_error_string(result));
                return result;
            }
            usleep(200000); /* 200ms delay */
        }

        /* Reset knee */
        pos.knee = 0;
        hexapod_set_leg_position(hexapod, leg, &pos);
        usleep(500000); /* 500ms delay */

        /* Move ankle joint */
        printf("  Moving ankle joint\n");
        for (int angle = -30; angle <= 30 && running; angle += 10)
        {
            pos.hip = 0;
            pos.knee = 0;
            pos.ankle = angle;
            result = hexapod_set_leg_position(hexapod, leg, &pos);
            if (result != 0)
            {
                fprintf(stderr, "Failed to move ankle: %s\n",
                        hexapod_error_string(result));
                return result;
            }
            usleep(200000); /* 200ms delay */
        }

        /* Reset ankle */
        pos.ankle = 0;
        hexapod_set_leg_position(hexapod, leg, &pos);
        usleep(500000); /* 500ms delay */
    }

    /* Center all legs again */
    printf("Centering all legs...\n");
    hexapod_center_all(hexapod);

    return 0;
}

/* Test gait patterns */
static int test_gait(hexapod_t *hexapod, gait_type_t gait_type)
{
    gait_params_t params;
    double start_time, current_time, elapsed_time;
    double direction = 0.0; /* Forward */
    double speed = 0.5;     /* Half-speed */
    int result;

    /* Initialize gait parameters */
    params.type = gait_type;
    params.step_height = 30.0; /* 30mm step height */
    params.step_length = 60.0; /* 60mm stride length */
    params.cycle_time = 2.0;   /* 2 seconds per cycle */

    switch (gait_type)
    {
    case GAIT_TRIPOD:
        printf("Testing tripod gait pattern\n");
        params.duty_factor = 0.5; /* 50% stance phase */
        break;

    case GAIT_WAVE:
        printf("Testing wave gait pattern\n");
        params.duty_factor = 0.75; /* 75% stance phase */
        break;

    case GAIT_RIPPLE:
        printf("Testing ripple gait pattern\n");
        params.duty_factor = 0.65; /* 65% stance phase */
        break;

    default:
        fprintf(stderr, "Invalid gait type\n");
        return -1;
    }

    /* Initialize gait controller */
    result = gait_init(hexapod, &params);
    if (result != 0)
    {
        fprintf(stderr, "Failed to initialize gait: %s\n",
                hexapod_error_string(result));
        return result;
    }

    /* Center legs before starting */
    gait_center(hexapod);
    sleep(1);

    /* Run gait pattern for 15 seconds or until Ctrl+C */
    printf("Running gait pattern for 15 seconds (press Ctrl+C to stop)...\n");
    start_time = get_time();
    elapsed_time = 0.0;

    while (elapsed_time < 15.0 && running)
    {
        current_time = get_time();
        elapsed_time = current_time - start_time;

        /* Update direction to create circular movement */
        direction = fmod(elapsed_time * 24.0, 360.0);

        /* Update gait */
        result = gait_update(hexapod, current_time, direction, speed);
        if (result != 0)
        {
            fprintf(stderr, "Gait update error: %s\n",
                    hexapod_error_string(result));
            break;
        }

        /* Print status every second */
        if ((int)elapsed_time != (int)(elapsed_time - 0.1))
        {
            printf("Time: %.1f s, Direction: %.1f°, Speed: %.1f\n",
                   elapsed_time, direction, speed);
        }

        usleep(20000); /* 20ms (50Hz update rate) */
    }

    /* Clean up */
    gait_cleanup();

    /* Center legs again */
    printf("Centering legs...\n");
    gait_center(hexapod);

    return 0;
}

/* Main test program */
int main(int argc, char *argv[])
{
    hexapod_t hexapod;
    int result;

    /* Set up signal handler */
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    printf("Hexapod Movement Test Program\n");
    printf("----------------------------\n");

    /* Initialize hexapod */
    result = hexapod_init(&hexapod);
    if (result != 0)
    {
        fprintf(stderr, "Failed to initialize hexapod: %s\n",
                hexapod_error_string(result));
        return 1;
    }

    printf("Connected to hexapod device\n\n");

    /* Run tests (or specific test if specified) */
    if (argc > 1)
    {
        if (strcmp(argv[1], "leg") == 0)
        {
            test_leg_movement(&hexapod);
        }
        else if (strcmp(argv[1], "tripod") == 0)
        {
            test_gait(&hexapod, GAIT_TRIPOD);
        }
        else if (strcmp(argv[1], "wave") == 0)
        {
            test_gait(&hexapod, GAIT_WAVE);
        }
        else if (strcmp(argv[1], "ripple") == 0)
        {
            test_gait(&hexapod, GAIT_RIPPLE);
        }
        else
        {
            printf("Unknown test: %s\n", argv[1]);
            printf("Available tests: leg, tripod, wave, ripple\n");
        }
    }
    else
    {
        /* Run all tests */
        printf("Running all tests. Press Ctrl+C to skip to next test.\n\n");
        printf("\n=== Testing Individual Leg Movement ===\n");
        test_leg_movement(&hexapod);
        if (!running)
            goto cleanup;
        sleep(1);

        printf("\n=== Testing Tripod Gait ===\n");
        test_gait(&hexapod, GAIT_TRIPOD);
        if (!running)
            goto cleanup;
        sleep(1);

        printf("\n=== Testing Wave Gait ===\n");
        test_gait(&hexapod, GAIT_WAVE);
        if (!running)
            goto cleanup;
        sleep(1);

        printf("\n=== Testing Ripple Gait ===\n");
        test_gait(&hexapod, GAIT_RIPPLE);
    }

cleanup:
    /* Center all legs before exit */
    printf("\nCentering all legs...\n");
    hexapod_center_all(&hexapod);

    /* Clean up */
    hexapod_cleanup(&hexapod);
    printf("Test program completed.\n");

    return 0;
}