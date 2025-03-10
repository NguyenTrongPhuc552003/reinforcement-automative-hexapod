#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include "controller.h"

static volatile int running = 1;

void handle_signal(int sig)
{
    printf("\nReceived signal %d\n", sig);
    running = 0;
}

int main(void)
{
    hexapod_t hex; // this is used to store the hexapod state
    int ret;
    char key;

    signal(SIGINT, handle_signal);

    /* Initialize hexapod */
    ret = hexapod_init(&hex);
    if (ret != 0)
    {
        fprintf(stderr, "Failed to initialize hexapod: %s\n",
                hexapod_error_string(ret));
        return 1;
    }

    /* Initialize controller */
    ret = controller_init(&hex);
    if (ret != 0)
    {
        fprintf(stderr, "Failed to initialize controller: %d\n", ret);
        hexapod_cleanup(&hex);
        return 1;
    }

    printf("Hexapod Controller\n");
    printf("Controls:\n");
    printf("  W/S: Forward/Backward\n");
    printf("  A/D: Rotate Left/Right\n");
    printf("  I/K: Raise/Lower\n");
    printf("  J/L: Tilt Left/Right\n");
    printf("  Space: Stop\n");
    printf("  Ctrl+C: Exit\n\n");

    /* Set up non-blocking input */
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    /* Main control loop */
    while (running)
    {
        /* Check for keyboard input (non-blocking) */
        if (read(STDIN_FILENO, &key, 1) > 0)
        {
            ret = controller_process_key(key);
            if (ret != 0)
            {
                fprintf(stderr, "Control error: %d\n", ret);
                break;
            }
        }

        /* Update hexapod state (now called regularly regardless of keypresses) */
        ret = controller_update();
        if (ret != 0)
        {
            fprintf(stderr, "Update error: %d\n", ret);
            break;
        }

        /* Small delay to prevent CPU overload */
        usleep(10000); /* 10ms */
    }

    /* Cleanup */
    controller_cleanup();
    hexapod_cleanup(&hex);
    printf("\nShutdown complete\n");

    return 0;
}
