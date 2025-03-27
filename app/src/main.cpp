/**
 * @file main.cpp
 * @brief Main entry point for the hexapod robot control application
 *
 * This file initializes and runs the main application, handling basic
 * startup, error reporting, and shutdown.
 */
#include <iostream>
#include <csignal>
#include "application.hpp"

/**
 * @brief Main entry point
 *
 * Creates the Application singleton, initializes subsystems, and starts
 * the main application loop.
 */
int main(void)
{
    // Print startup banner
    std::cout << "=============================================" << std::endl;
    std::cout << "    Hexapod Robot Control System v1.0        " << std::endl;
    std::cout << "=============================================" << std::endl;

    // Get singleton instance
    application::Application &app = application::Application::getInstance();

    // Initialize with error checking
    if (!app.init())
    {
        std::cerr << "Initialization failed: " << app.getLastErrorMessage() << std::endl;
        std::cerr << "Exiting with error." << std::endl;
        return 1; // Error code
    }

    std::cout << "Initialization successful." << std::endl;

    // Run main application and check result
    auto result = app.run();

    // Clean shutdown
    std::cout << "Application finished with status: ";
    switch (result)
    {
    case application::ExecutionResult::SUCCESS:
        std::cout << "Success" << std::endl;
        break;

    case application::ExecutionResult::TERMINATED_BY_USER:
        std::cout << "Terminated by user" << std::endl;
        break;

    case application::ExecutionResult::ERROR_INITIALIZATION:
        std::cout << "Initialization error" << std::endl;
        break;

    case application::ExecutionResult::ERROR_RUNTIME:
        std::cout << "Runtime error: " << app.getLastErrorMessage() << std::endl;
        break;

    case application::ExecutionResult::ERROR_SHUTDOWN:
        std::cout << "Shutdown error: " << app.getLastErrorMessage() << std::endl;
        break;

    default:
        std::cout << "Unknown error" << std::endl;
        break;
    }

    // Return appropriate exit code (0 for success, 1 for errors)
    return (result == application::ExecutionResult::SUCCESS ||
            result == application::ExecutionResult::TERMINATED_BY_USER)
               ? 0
               : 1;
}
