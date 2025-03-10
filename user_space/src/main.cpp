#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include "hexapod.hpp"
#include "controller.hpp"

static volatile int running = 1;

void handle_signal(int sig)
{
    printf("\nReceived signal %d\n", sig);
    running = 0;
}

int main(void)
{
    signal(SIGINT, handle_signal);

    // Initialize hexapod
    Hexapod hexapod;
    if (!hexapod.init())
    {
        fprintf(stderr, "Failed to initialize hexapod: %s\n",
                hexapod.getLastErrorMessage().c_str());
        return 1;
    }

    // Initialize controller
    Controller controller(hexapod);
    if (!controller.init())
    {
        fprintf(stderr, "Failed to initialize controller\n");
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

    // Set up non-blocking input
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    // Main control loop
    while (running)
    {
        // Check for keyboard input (non-blocking)
        char key;
        if (read(STDIN_FILENO, &key, 1) > 0)
        {
            if (!controller.processKey(key))
            {
                fprintf(stderr, "Control error\n");
                break;
            }
        }

        // Update hexapod state (now called regularly regardless of keypresses)
        if (!controller.update())
        {
            fprintf(stderr, "Update error\n");
            break;
        }

        // Small delay to prevent CPU overload
        usleep(10000); // 10ms
    }

    // Controller will be cleaned up automatically in destructor
    printf("\nShutdown complete\n");

    return 0;
}
