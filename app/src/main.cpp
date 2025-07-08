/*
 * Hexapod Project - A Reinforcement Learning-based Autonomous Hexapod
 * Copyright (C) 2025  Nguyen Trong Phuc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/**
 * @file main.cpp
 * @brief Main entry point for the hexapod robot control application
 *
 * This file initializes and runs the main application, handling basic
 * startup, error reporting, and shutdown.
 */
#include <iostream>
#include <csignal>
#include <atomic>
#include "application.hpp"
#include "common.hpp"

// Global running flag for signal handling
static std::atomic<bool> s_running(true);

/**
 * @brief Signal handler using common utilities
 */
void signalHandler(int signal)
{
    s_running.store(false);
    common::ErrorReporter::reportInfo("Main", "Received termination signal " + std::to_string(signal));
}

/**
 * @brief Main entry point
 *
 * Creates the Application singleton, initializes subsystems, and starts
 * the main application loop.
 */
int main(void)
{
    // Setup graceful shutdown handling using common utilities
    common::SignalManager::setupGracefulShutdown(s_running, signalHandler);

    // Initialize performance monitoring
    common::PerformanceMonitor perfMonitor;

    // Print startup banner using common string utilities
    std::cout << "=============================================" << std::endl;
    std::cout << "    Hexapod Robot Control System v1.0        " << std::endl;
    std::cout << "=============================================" << std::endl;

    common::ErrorReporter::reportInfo("Main", "Starting hexapod control system");

    // Get singleton instance
    application::Application &app = application::Application::getInstance();

    perfMonitor.startFrame();

    // Initialize with error checking
    if (!app.init())
    {
        common::ErrorReporter::reportError("Main", "Initialization", app.getLastErrorMessage());
        std::cerr << "Exiting with error." << std::endl;
        return 1; // Error code
    }

    perfMonitor.endFrame();
    common::ErrorReporter::reportInfo("Main", "Initialization completed in " +
                                                  common::StringUtils::formatNumber(perfMonitor.getAverageFrameTime()) + "ms");

    // Run main application and check result
    perfMonitor.reset();
    perfMonitor.startFrame();
    auto result = app.run();
    perfMonitor.endFrame();

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

    // Print performance report
    perfMonitor.printReport("Application runtime ");

    // Final status report
    bool success = (result == application::ExecutionResult::SUCCESS ||
                    result == application::ExecutionResult::TERMINATED_BY_USER);

    if (success)
    {
        common::ErrorReporter::reportInfo("Main", "Application completed successfully");
    }
    else
    {
        common::ErrorReporter::reportError("Main", "Execution", "Application terminated with errors");
    }

    // Return appropriate exit code (0 for success, 1 for errors)
    return success ? 0 : 1;
}
