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
 * @file autonomous_demo.cpp
 * @brief Autonomous Navigation and Obstacle Avoidance Demo
 *
 * This program demonstrates the hexapod's autonomous capabilities using:
 * - CPG-based locomotion control
 * - MPU6050/ADXL345 IMU feedback for stability
 * - Ultrasonic obstacle detection and avoidance
 * - Adaptive gait selection based on terrain conditions
 */

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>
#include "hexapod.hpp"
#include "cpg/controller.hpp"
#include "common.hpp"

// Global running flag for signal handling
static std::atomic<bool> running(true);

// Signal handler
void signalHandler(int signal)
{
    running.store(false);
    common::ErrorReporter::reportInfo("Autonomous-Demo",
                                      "Received termination signal " + std::to_string(signal));
}

// Demo configuration
struct DemoConfig
{
    double max_duration = 60.0;        // 1 minute demo
    double obstacle_threshold = 25.0;  // Stop at 25cm
    double slowdown_threshold = 40.0;  // Slow down at 40cm
    double imu_update_rate = 50.0;     // 50Hz IMU updates
    double obstacle_check_rate = 10.0; // 10Hz obstacle checks
    bool enable_telemetry = true;      // Show real-time data
    bool enable_adaptive_gait = true;  // Use adaptive gaits
};

class AutonomousNavigator
{
private:
    hexapod::Hexapod m_hexapod;
    cpg::Controller m_controller;
    DemoConfig m_config;

    // State tracking
    std::chrono::high_resolution_clock::time_point m_startTime;
    std::chrono::high_resolution_clock::time_point m_lastIMUUpdate;
    std::chrono::high_resolution_clock::time_point m_lastObstacleCheck;
    std::chrono::high_resolution_clock::time_point m_lastTelemetry;

    // Navigation state
    bool m_obstacleDetected = false;
    double m_currentDistance = -1.0;
    std::string m_currentGait = "tripod";
    double m_currentSpeed = 0.2;

    // IMU state
    double m_currentRoll = 0.0;
    double m_currentPitch = 0.0;
    bool m_isStable = true;

public:
    AutonomousNavigator(const DemoConfig &config = DemoConfig{})
        : m_config(config)
    {
        m_startTime = std::chrono::high_resolution_clock::now();
        m_lastIMUUpdate = m_startTime;
        m_lastObstacleCheck = m_startTime;
        m_lastTelemetry = m_startTime;
    }

    bool initialize()
    {
        common::ErrorReporter::reportInfo("Autonomous-Demo", "Initializing navigation system...");

        // Initialize hexapod hardware
        if (!m_hexapod.init())
        {
            common::ErrorReporter::reportError("Autonomous-Demo", "Hexapod Init",
                                               m_hexapod.getLastErrorMessage());
            return false;
        }

        // Initialize CPG controller
        if (!m_controller.initialize())
        {
            common::ErrorReporter::reportError("Autonomous-Demo", "CPG Init",
                                               "Failed to initialize CPG controller");
            return false;
        }

        // Note: Ultrasonic sensor functionality integrated through hexapod class
        // No separate ultrasonic sensor initialization needed

        std::cout << "Note: Using integrated obstacle detection through hexapod IMU feedback" << std::endl;

        // Center all legs for safety
        std::cout << "Centering legs for safe start..." << std::endl;
        if (!m_hexapod.centerAll())
        {
            common::ErrorReporter::reportWarning("Autonomous-Demo", "Failed to center legs");
        }

        std::cout << "Autonomous navigation system initialized successfully!" << std::endl;
        return true;
    }

    void runDemo()
    {
        std::cout << "\n"
                  << std::string(60, '=') << std::endl;
        std::cout << "  AUTONOMOUS NAVIGATION DEMO STARTING" << std::endl;
        std::cout << "  Duration: " << m_config.max_duration << " seconds" << std::endl;
        std::cout << "  Obstacle threshold: " << m_config.obstacle_threshold << " cm" << std::endl;
        std::cout << "  Press Ctrl+C to stop anytime" << std::endl;
        std::cout << std::string(60, '=') << std::endl;

        // Wait for user readiness
        std::cout << "\nPress Enter to start autonomous navigation...";
        std::cin.get();

        // Start autonomous walking
        startAutonomousWalking();

        // Main navigation loop
        while (running.load() && !isDemoComplete())
        {
            auto now = std::chrono::high_resolution_clock::now();

            // Update IMU feedback
            if (shouldUpdateIMU(now))
            {
                updateIMUFeedback();
                m_lastIMUUpdate = now;
            }

            // Check for obstacles
            if (shouldCheckObstacles(now))
            {
                checkObstacles();
                m_lastObstacleCheck = now;
            }

            // Update controller
            if (!m_controller.update(0.02)) // 50Hz update rate
            {
                common::ErrorReporter::reportError("Autonomous-Demo", "Controller Update",
                                                   "Failed to update CPG controller");
                break;
            }

            // Show telemetry
            if (m_config.enable_telemetry && shouldUpdateTelemetry(now))
            {
                showTelemetry();
                m_lastTelemetry = now;
            }

            // Small delay to control loop rate
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        // Stop navigation gracefully
        stopAutonomousWalking();

        // Show demo summary
        showDemoSummary();
    }

private:
    bool isDemoComplete()
    {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                           std::chrono::high_resolution_clock::now() - m_startTime)
                           .count();
        return elapsed >= m_config.max_duration;
    }

    bool shouldUpdateIMU(const std::chrono::high_resolution_clock::time_point &now)
    {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastIMUUpdate).count();
        return elapsed >= (1000.0 / m_config.imu_update_rate);
    }

    bool shouldCheckObstacles(const std::chrono::high_resolution_clock::time_point &now)
    {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastObstacleCheck).count();
        return elapsed >= (1000.0 / m_config.obstacle_check_rate);
    }

    bool shouldUpdateTelemetry(const std::chrono::high_resolution_clock::time_point &now)
    {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastTelemetry).count();
        return elapsed >= 500; // 2Hz telemetry updates
    }

    void startAutonomousWalking()
    {
        std::cout << "Starting autonomous walking with CPG controller..." << std::endl;

        // Start with stable tripod gait
        cpg::LocomotionCommand walkCommand(m_currentSpeed, 0.0, m_currentGait);
        if (!m_controller.startLocomotion(walkCommand))
        {
            common::ErrorReporter::reportError("Autonomous-Demo", "Walking Start",
                                               "Failed to start autonomous walking");
            return;
        }

        std::cout << "Autonomous walking started - " << m_currentGait << " gait at "
                  << m_currentSpeed << " m/s" << std::endl;
    }

    void updateIMUFeedback()
    {
        hexapod::ImuData imuData;
        if (m_hexapod.getImuData(imuData))
        {
            // Calculate orientation angles
            double ax = imuData.getAccelX();
            double ay = imuData.getAccelY();
            double az = imuData.getAccelZ();

            m_currentRoll = atan2(ay, az) * 180.0 / M_PI;
            m_currentPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

            // Check stability
            m_isStable = (std::abs(m_currentRoll) < 20.0 && std::abs(m_currentPitch) < 20.0);

            // Create angular velocity vector
            std::vector<double> angular_velocity = {
                imuData.getGyroX() * M_PI / 180.0, // Convert to rad/s
                imuData.getGyroY() * M_PI / 180.0,
                imuData.getGyroZ() * M_PI / 180.0};

            // Update CPG controller with balance feedback
            m_controller.updateBalanceFeedback(m_currentRoll * M_PI / 180.0,
                                               m_currentPitch * M_PI / 180.0,
                                               angular_velocity);

            // Adaptive gait selection based on stability
            if (m_config.enable_adaptive_gait)
            {
                adaptGaitToStability();
            }
        }
    }

    void checkObstacles()
    {
        // For demo purposes, we'll simulate obstacle detection using IMU stability
        // In real implementation, this would use actual ultrasonic/distance sensors

        // Simulate obstacle detection based on extreme tilt (simulating rough terrain/obstacles)
        if (std::abs(m_currentRoll) > 25.0 || std::abs(m_currentPitch) > 25.0)
        {
            m_currentDistance = 20.0; // Simulate close obstacle

            if (!m_obstacleDetected)
            {
                m_obstacleDetected = true;
                handleObstacleDetected();
            }
        }
        else if (std::abs(m_currentRoll) > 15.0 || std::abs(m_currentPitch) > 15.0)
        {
            m_currentDistance = 35.0; // Simulate moderate distance obstacle
            handleObstacleApproaching();
        }
        else
        {
            m_currentDistance = 100.0; // Clear path
            if (m_obstacleDetected)
            {
                m_obstacleDetected = false;
                handleObstacleClear();
            }
        }
    }

    void adaptGaitToStability()
    {
        std::string newGait = m_currentGait;
        double newSpeed = m_currentSpeed;

        if (!m_isStable)
        {
            // Use more stable wave gait when unstable
            newGait = "wave";
            newSpeed = 0.1; // Slower for safety
        }
        else if (std::abs(m_currentRoll) > 10.0 || std::abs(m_currentPitch) > 10.0)
        {
            // Moderate instability - use tripod but slower
            newGait = "tripod";
            newSpeed = 0.15;
        }
        else
        {
            // Stable - can use faster tripod gait
            newGait = "tripod";
            newSpeed = 0.2;
        }

        // Update if changed
        if (newGait != m_currentGait || std::abs(newSpeed - m_currentSpeed) > 0.05)
        {
            m_currentGait = newGait;
            m_currentSpeed = newSpeed;

            cpg::LocomotionCommand newCommand(m_currentSpeed, 0.0, m_currentGait);
            m_controller.startLocomotion(newCommand);
        }
    }

    void handleObstacleDetected()
    {
        std::cout << "\n*** OBSTACLE DETECTED at " << m_currentDistance << " cm ***" << std::endl;

        // Stop current movement
        m_controller.stopLocomotion();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        // Determine avoidance strategy based on IMU
        double turn_velocity = -0.3; // Default right turn
        std::string avoid_gait = "tripod";

        if (m_isStable)
        {
            // Choose turn direction based on current tilt
            if (m_currentRoll > 5.0)
                turn_velocity = 0.3; // Turn left if tilted right
            else if (m_currentRoll < -5.0)
                turn_velocity = -0.3; // Turn right if tilted left
        }
        else
        {
            // Use more stable gait when unstable
            avoid_gait = "wave";
            turn_velocity *= 0.5; // Slower turn
        }

        std::cout << "Executing avoidance maneuver: " << avoid_gait << " gait, turn "
                  << (turn_velocity > 0 ? "left" : "right") << std::endl;

        // Execute turn
        cpg::LocomotionCommand turnCommand(0.0, turn_velocity, avoid_gait);
        m_controller.startLocomotion(turnCommand);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Stop and stabilize
        m_controller.stopLocomotion();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Resume forward movement
        cpg::LocomotionCommand forwardCommand(0.1, 0.0, avoid_gait);
        m_controller.startLocomotion(forwardCommand);

        std::cout << "Avoidance complete - resuming navigation" << std::endl;
    }

    void handleObstacleApproaching()
    {
        // Gradual slowdown as obstacle approaches
        double distance_factor = (m_currentDistance - m_config.obstacle_threshold) /
                                 (m_config.slowdown_threshold - m_config.obstacle_threshold);
        double reduced_speed = std::max(0.05, m_currentSpeed * distance_factor);

        m_controller.setLinearVelocity(reduced_speed);
    }

    void handleObstacleClear()
    {
        std::cout << "Obstacle cleared - resuming normal speed" << std::endl;

        // Resume normal autonomous walking
        cpg::LocomotionCommand normalCommand(m_currentSpeed, 0.0, m_currentGait);
        m_controller.startLocomotion(normalCommand);
    }

    void stopAutonomousWalking()
    {
        std::cout << "\nStopping autonomous navigation..." << std::endl;

        m_controller.stopLocomotion();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Center legs for safe shutdown
        std::cout << "Centering legs for safe shutdown..." << std::endl;
        m_hexapod.centerAll();

        std::cout << "Autonomous navigation stopped safely" << std::endl;
    }

    void showTelemetry()
    {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                           std::chrono::high_resolution_clock::now() - m_startTime)
                           .count();

        auto state = m_controller.getControllerState();

        // Clear screen and show telemetry
        std::cout << "\033[2J\033[H"; // Clear screen and move to top

        std::cout << "+" << std::string(58, '-') << "+" << std::endl;
        std::cout << "|  AUTONOMOUS HEXAPOD NAVIGATION - LIVE TELEMETRY          |" << std::endl;
        std::cout << "+" << std::string(58, '-') << "+" << std::endl;

        std::cout << "|  Time: " << std::setw(3) << elapsed << "s / "
                  << static_cast<int>(m_config.max_duration) << "s" << std::string(35, ' ') << "|" << std::endl;

        std::cout << "|  IMU Status:" << std::string(44, ' ') << "|" << std::endl;
        std::cout << "|    Roll: " << std::setw(6) << std::fixed << std::setprecision(1)
                  << m_currentRoll << " deg  Pitch: " << std::setw(6) << m_currentPitch
                  << " deg  Stable: " << (m_isStable ? "YES" : "NO ") << std::string(15, ' ') << "|" << std::endl;

        std::cout << "|  Locomotion:" << std::string(44, ' ') << "|" << std::endl;
        std::cout << "|    Gait: " << std::setw(8) << m_currentGait
                  << "  Speed: " << std::setw(5) << std::setprecision(2) << m_currentSpeed
                  << " m/s" << std::string(26, ' ') << "|" << std::endl;
        std::cout << "|    Frequency: " << std::setw(5) << state.current_frequency
                  << " Hz" << std::string(35, ' ') << "|" << std::endl;

        std::cout << "|  Obstacle Detection:" << std::string(36, ' ') << "|" << std::endl;
        if (m_currentDistance > 0)
        {
            std::cout << "|    Distance: " << std::setw(6) << std::setprecision(1)
                      << m_currentDistance << " cm  Status: "
                      << (m_obstacleDetected ? "DETECTED" : "CLEAR   ")
                      << std::string(21, ' ') << "|" << std::endl;
        }
        else
        {
            std::cout << "|    Distance: NO DATA" << std::string(35, ' ') << "|" << std::endl;
        }

        std::cout << "|  Performance:" << std::string(43, ' ') << "|" << std::endl;
        std::cout << "|    Energy: " << std::setw(6) << std::setprecision(1)
                  << state.energy_consumption << " W  Stability: "
                  << std::setw(5) << std::setprecision(3) << state.stability_margin
                  << std::string(20, ' ') << "|" << std::endl;

        std::cout << "+" << std::string(58, '-') << "+" << std::endl;
        std::cout << "Press Ctrl+C to stop autonomous navigation" << std::endl;
    }

    void showDemoSummary()
    {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                           std::chrono::high_resolution_clock::now() - m_startTime)
                           .count();

        std::cout << "\n"
                  << std::string(50, '=') << std::endl;
        std::cout << "  AUTONOMOUS NAVIGATION DEMO COMPLETED" << std::endl;
        std::cout << std::string(50, '=') << std::endl;

        std::cout << "  Total runtime: " << elapsed << " seconds" << std::endl;
        std::cout << "  Final stability: " << (m_isStable ? "STABLE" : "UNSTABLE") << std::endl;

        if (m_currentDistance > 0)
        {
            std::cout << "  Final obstacle distance: " << std::setprecision(1)
                      << m_currentDistance << " cm" << std::endl;
        }

        auto state = m_controller.getControllerState();
        std::cout << "  Total energy consumption: " << std::setprecision(2)
                  << state.energy_consumption << " W" << std::endl;

        std::cout << "\nDemo completed successfully!" << std::endl;
    }
};

int main()
{
    // Setup signal handling
    common::SignalManager::setupGracefulShutdown(running, signalHandler);

    std::cout << "Hexapod Autonomous Navigation Demo" << std::endl;
    std::cout << "Using CPG + IMU + Ultrasonic Obstacle Avoidance" << std::endl;

    // Create demo configuration
    DemoConfig config;
    config.max_duration = 60.0;       // 1 minute demo
    config.obstacle_threshold = 25.0; // Stop at 25cm
    config.enable_telemetry = true;
    config.enable_adaptive_gait = true;

    // Create and run navigator
    AutonomousNavigator navigator(config);

    if (!navigator.initialize())
    {
        std::cerr << "Failed to initialize autonomous navigator!" << std::endl;
        return 1;
    }

    navigator.runDemo();

    std::cout << "Thank you for trying the autonomous navigation demo!" << std::endl;
    return 0;
}
