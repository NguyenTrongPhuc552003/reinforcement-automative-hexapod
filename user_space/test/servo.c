#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include "hexapod.h"

/* Global flag for program control */
static volatile int running = 1;

/* Signal handler for graceful termination */
void handle_signal(int sig __attribute__((unused)))
{
    printf("\nExiting...\n");
    running = 0;
}

int main(int argc, char *argv[])
{
    hexapod_t hex;
    leg_position_t pos = {0, 0, 0};
    int ret, i;
    uint8_t leg_num = 0;
    double angle = 0;

    /* Set up signal handler for clean termination */
    signal(SIGINT, handle_signal);

    /* Check command line args */
    if (argc > 1)
    {
        leg_num = (uint8_t)atoi(argv[1]);
        if (leg_num >= NUM_LEGS)
        {
            fprintf(stderr, "Leg number must be 0-%d\n", NUM_LEGS - 1);
            return 1;
        }
    }

    /* Initialize hexapod */
    ret = hexapod_init(&hex);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to initialize hexapod: %s\n",
                hexapod_error_string(ret));
        return 1;
    }

    printf("Servo Test - Leg %d - Press Ctrl+C to exit\n", leg_num);
    printf("==========================================\n");

    /* Center all servos to start */
    ret = hexapod_center_all(&hex);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to center servos: %s\n",
                hexapod_error_string(ret));
        hexapod_cleanup(&hex);
        return 1;
    }

    /* Allow servos to reach position */
    sleep(1);

    /* Main loop - sweep each joint back and forth */
    while (running)
    {
        /* Sweep hip */
        printf("Sweeping hip joint...\n");
        for (i = 0; i < 100 && running; i++)
        {
            angle = sin((double)i * 0.063) * 45.0; /* -45 to +45 degrees */
            pos.hip = (int16_t)angle;
            pos.knee = 0;
            pos.ankle = 0;

            ret = hexapod_set_leg_position(&hex, leg_num, &pos);
            if (ret < 0)
            {
                fprintf(stderr, "Failed to set position: %s\n",
                        hexapod_error_string(ret));
                break;
            }

            usleep(30000); /* 30ms */
        }

        if (!running)
            break;

        /* Center */
        pos.hip = 0;
        hexapod_set_leg_position(&hex, leg_num, &pos);
        usleep(300000); /* 300ms */

        /* Sweep knee */
        printf("Sweeping knee joint...\n");
        for (i = 0; i < 100 && running; i++)
        {
            angle = sin((double)i * 0.063) * 45.0;
            pos.hip = 0;
            pos.knee = (int16_t)angle;
            pos.ankle = 0;

            ret = hexapod_set_leg_position(&hex, leg_num, &pos);
            if (ret < 0)
            {
                fprintf(stderr, "Failed to set position: %s\n",
                        hexapod_error_string(ret));
                break;
            }

            usleep(30000); /* 30ms */
        }

        if (!running)
            break;

        /* Center */
        pos.knee = 0;
        hexapod_set_leg_position(&hex, leg_num, &pos);
        usleep(300000); /* 300ms */

        /* Sweep ankle */
        printf("Sweeping ankle joint...\n");
        for (i = 0; i < 100 && running; i++)
        {
            angle = sin((double)i * 0.063) * 45.0;
            pos.hip = 0;
            pos.knee = 0;
            pos.ankle = (int16_t)angle;

            ret = hexapod_set_leg_position(&hex, leg_num, &pos);
            if (ret < 0)
            {
                fprintf(stderr, "Failed to set position: %s\n",
                        hexapod_error_string(ret));
                break;
            }

            usleep(30000); /* 30ms */
        }

        /* Center */
        pos.ankle = 0;
        hexapod_set_leg_position(&hex, leg_num, &pos);
        usleep(300000); /* 300ms */
    }

    /* Center all servos before exit */
    hexapod_center_all(&hex);

    /* Cleanup */
    hexapod_cleanup(&hex);
    return 0;
}
