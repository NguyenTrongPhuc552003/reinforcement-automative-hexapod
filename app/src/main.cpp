#include "hexapod.hpp"
#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

static bool running = true;

void signalHandler(int signal)
{
    std::cout << "\nReceived signal " << signal << ". Shutting down..." << std::endl;
    running = false;
}

int main()
{
    std::cout << "Hexapod Autonomous Movement Controller" << std::endl;
    std::cout << "=====================================" << std::endl;

    // Setup signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Initialize Hexapod
    Hexapod hexapod;

    if (!hexapod.init())
    {
        std::cerr << "Failed to initialize Hexapod" << std::endl;
        return -1;
    }

    std::cout << "\nInitializing hexapod..." << std::endl;

    // Move to HOME position and wait 2 seconds
    std::cout << "Moving all servos to HOME position..." << std::endl;
    if (!hexapod.homePosition())
    {
        std::cerr << "Failed to set HOME position" << std::endl;
        return -1;
    }

    std::cout << "Hexapod is at HOME position. Waiting 2 seconds before starting movement..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Start autonomous movement
    std::cout << "\nStarting autonomous movement!" << std::endl;
    if (!hexapod.startAutonomousMovement())
    {
        std::cerr << "Failed to start autonomous movement" << std::endl;
        return -1;
    }

    std::cout << "Hexapod is now moving autonomously." << std::endl;
    std::cout << "Press Ctrl+C to stop and exit..." << std::endl;

    // Main movement loop
    auto last_time = std::chrono::high_resolution_clock::now();

    while (running)
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            current_time - last_time);
        double time_step = duration.count() / 1000000.0; // Convert to seconds

        // Cap time step to prevent large jumps
        if (time_step > 0.1)
            time_step = 0.1;

        // Update hexapod movement
        hexapod.update(time_step);

        last_time = current_time;

        // Small delay to prevent excessive CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50Hz update rate
    }

    std::cout << "\nStopping movement and cleaning up..." << std::endl;
    hexapod.stopMovement();
    hexapod.cleanup();

    std::cout << "Hexapod controller exited successfully." << std::endl;
    return 0;
}
