#include "hexapod.hpp"
#include <cstdio>
#include <csignal>
#include <cstdlib>

static bool running = true;

void signalHandler(int signal)
{
    printf("\nShutting down hexapod with signal %d...\n", signal);
    running = false;
}

int main()
{
    printf("=== Hexapod Autonomous Controller ===\n");
    printf("Hardware: BeagleBone Black + 18x MG996R + 2x PCA9685 + HC-SR04\n");
    printf("Features: Autonomous walking with obstacle avoidance\n\n");

    // Setup signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Create hexapod instance
    Hexapod hexapod;

    // Initialize hardware
    if (!hexapod.init())
    {
        printf("Failed to initialize hexapod hardware!\n");
        return -1;
    }

    printf("Press Ctrl+C to stop\n\n");

    // Run autonomous mode
    try
    {
        hexapod.run();
    }
    catch (...)
    {
        printf("Unexpected error occurred\n");
    }

    printf("Hexapod stopped.\n");
    return 0;
}
