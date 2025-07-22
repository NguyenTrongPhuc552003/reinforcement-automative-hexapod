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

#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include <sstream>
#include <cmath>
#include "application.hpp"
#include "ultrasonic.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace application
{

    //==============================================================================
    // Static Member Initialization
    //==============================================================================

    // Initialize static members
    std::atomic<bool> Application::m_running(false);
    std::atomic<bool> Application::m_telemetryActive(false);

    //==============================================================================
    // Types for Implementation
    //==============================================================================

    /**
     * @brief Command function type for key command pattern
     */
    using KeyCommand = std::function<bool()>;

    //==============================================================================
    // Implementation Class (PIMPL idiom)
    //==============================================================================

    class ApplicationImpl
    {
    public:
        /**
         * @brief Construct a new Application Implementation object
         */
        ApplicationImpl()
            : m_currentMode(ControlMode::MANUAL),
              m_hexapod(nullptr),
              m_controller(nullptr),
              m_ultrasonicSensor(nullptr),
              m_updateInterval(0.01f), // 10ms default update interval
              m_frameCount(0),
              m_totalFrameTime(0),
              m_maxFrameTime(0),
              m_performanceMonitoringEnabled(true),
              m_autonomousMode(false),
              m_obstacleDetectionEnabled(true),
              m_safeDistance(30.0f), // 30cm safe distance
              m_lastObstacleCheck(std::chrono::high_resolution_clock::now()),
              m_obstacleCheckInterval(100) // Check every 100ms
        {
            // Initialize performance tracking variables
            m_lastUpdateTime = std::chrono::high_resolution_clock::now();
        }

        /**
         * @brief Destroy the Application Implementation object
         */
        ~ApplicationImpl() = default;

        /**
         * @brief Initialize the hexapod hardware interface
         *
         * @return true if initialization successful
         * @return false if initialization failed
         */
        bool initializeHexapod()
        {
            try
            {
                std::cout << "Initializing hexapod hardware interface..." << std::endl;
                m_hexapod = std::make_unique<hexapod::Hexapod>();

                // Try to initialize with retries
                const int maxRetries = 3;
                for (int retry = 0; retry < maxRetries; retry++)
                {
                    if (m_hexapod->init())
                    {
                        return true;
                    }

                    std::cerr << "Initialization attempt " << (retry + 1) << " failed: "
                              << m_hexapod->getLastErrorMessage() << std::endl;

                    if (retry < maxRetries - 1)
                    {
                        std::cout << "Retrying in 1 second..." << std::endl;
                        sleep(1);
                    }
                }

                m_lastError = "Failed to initialize hexapod after " +
                              std::to_string(maxRetries) + " attempts: " +
                              m_hexapod->getLastErrorMessage();
                return false;
            }
            catch (const std::exception &e)
            {
                m_lastError = std::string("Hexapod initialization exception: ") + e.what();
                return false;
            }
        }

        /**
         * @brief Initialize the controller
         *
         * @return true if initialization successful
         * @return false if initialization failed
         */
        bool initializeController()
        {
            try
            {
                if (!m_hexapod)
                {
                    m_lastError = "Cannot initialize controller: Hexapod not initialized";
                    return false;
                }

                m_controller = std::make_unique<cpg::Controller>();
                if (!m_controller->initialize())
                {
                    m_lastError = "Failed to initialize CPG controller";
                    return false;
                }
                return true;
            }
            catch (const std::exception &e)
            {
                m_lastError = std::string("Controller initialization exception: ") + e.what();
                return false;
            }
        }

        /**
         * @brief Initialize the ultrasonic sensor for obstacle detection
         *
         * @return true if initialization successful
         * @return false if initialization failed
         */
        bool initializeUltrasonicSensor()
        {
            try
            {
                std::cout << "Initializing ultrasonic sensor for obstacle detection..." << std::endl;
                m_ultrasonicSensor = std::make_unique<UltrasonicSensor>();

                if (!m_ultrasonicSensor->init())
                {
                    std::cerr << "Warning: Failed to initialize ultrasonic sensor: "
                              << m_ultrasonicSensor->getLastError() << std::endl;
                    std::cerr << "Obstacle detection will be disabled." << std::endl;
                    m_obstacleDetectionEnabled = false;
                    m_ultrasonicSensor.reset();
                    return true; // Not critical failure, continue without sensor
                }

                std::cout << "Ultrasonic sensor initialized successfully" << std::endl;
                return true;
            }
            catch (const std::exception &e)
            {
                std::cerr << "Warning: Ultrasonic sensor initialization exception: " << e.what() << std::endl;
                std::cerr << "Obstacle detection will be disabled." << std::endl;
                m_obstacleDetectionEnabled = false;
                m_ultrasonicSensor.reset();
                return true; // Not critical, continue without sensor
            }
        }

        /**
         * @brief Set up non-blocking input handling
         *
         * @return true if setup successful
         * @return false if setup failed
         */
        bool setupInputHandling()
        {
            try
            {
                // Set up non-blocking input
                int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
                if (flags == -1)
                {
                    m_lastError = "Failed to get file status flags";
                    return false;
                }

                if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1)
                {
                    m_lastError = "Failed to set non-blocking mode";
                    return false;
                }
                return true;
            }
            catch (const std::exception &e)
            {
                m_lastError = std::string("Input setup exception: ") + e.what();
                return false;
            }
        }

        /**
         * @brief Register key commands for user interaction
         */
        void setupKeyCommands()
        {
            // Basic movement controls
            registerKeyCommand('w', [this]()
                               { 
                                   // Forward movement
                                   m_controller->setLinearVelocity(0.5);
                                   m_controller->setAngularVelocity(0.0);
                                   if (!m_controller->isWalking()) {
                                       cpg::LocomotionCommand cmd = cpg::controller_utils::createWalkCommand(0.5, 0.0, m_controller->getCurrentGait());
                                       return m_controller->startLocomotion(cmd);
                                   }
                                   return true; });
            registerKeyCommand('s', [this]()
                               { 
                                   // Backward movement
                                   m_controller->setLinearVelocity(-0.5);
                                   m_controller->setAngularVelocity(0.0);
                                   if (!m_controller->isWalking()) {
                                       cpg::LocomotionCommand cmd = cpg::controller_utils::createWalkCommand(-0.5, 0.0, m_controller->getCurrentGait());
                                       return m_controller->startLocomotion(cmd);
                                   }
                                   return true; });
            registerKeyCommand('a', [this]()
                               { 
                                   // Turn left
                                   m_controller->setLinearVelocity(0.0);
                                   m_controller->setAngularVelocity(0.5);
                                   if (!m_controller->isWalking()) {
                                       cpg::LocomotionCommand cmd = cpg::controller_utils::createWalkCommand(0.0, 0.5, m_controller->getCurrentGait());
                                       return m_controller->startLocomotion(cmd);
                                   }
                                   return true; });
            registerKeyCommand('d', [this]()
                               { 
                                   // Turn right
                                   m_controller->setLinearVelocity(0.0);
                                   m_controller->setAngularVelocity(-0.5);
                                   if (!m_controller->isWalking()) {
                                       cpg::LocomotionCommand cmd = cpg::controller_utils::createWalkCommand(0.0, -0.5, m_controller->getCurrentGait());
                                       return m_controller->startLocomotion(cmd);
                                   }
                                   return true; });

            // Height and tilt controls - these might need adaptation
            registerKeyCommand('i', [this]()
                               { 
                                   std::cout << "Height increase - not yet implemented for CPG" << std::endl;
                                   return true; });
            registerKeyCommand('k', [this]()
                               { 
                                   std::cout << "Height decrease - not yet implemented for CPG" << std::endl;
                                   return true; });
            registerKeyCommand('j', [this]()
                               { 
                                   std::cout << "Tilt left - not yet implemented for CPG" << std::endl;
                                   return true; });
            registerKeyCommand('l', [this]()
                               { 
                                   std::cout << "Tilt right - not yet implemented for CPG" << std::endl;
                                   return true; });

            // Gait controls
            registerKeyCommand('1', [this]()
                               { return m_controller->switchGait("tripod"); });
            registerKeyCommand('2', [this]()
                               { return m_controller->switchGait("wave"); });
            registerKeyCommand('3', [this]()
                               { return m_controller->switchGait("ripple"); });

            // Speed controls - need to adjust current velocity
            registerKeyCommand('+', [this]()
                               { 
                                   double current_linear = m_controller->getLinearVelocity();
                                   double current_angular = m_controller->getAngularVelocity();
                                   m_controller->setLinearVelocity(std::min(1.0, current_linear * 1.1));
                                   m_controller->setAngularVelocity(std::min(1.0, current_angular * 1.1));
                                   return true; });
            registerKeyCommand('-', [this]()
                               { 
                                   double current_linear = m_controller->getLinearVelocity();
                                   double current_angular = m_controller->getAngularVelocity();
                                   m_controller->setLinearVelocity(std::max(-1.0, current_linear * 0.9));
                                   m_controller->setAngularVelocity(std::max(-1.0, current_angular * 0.9));
                                   return true; });

            // System controls
            registerKeyCommand(' ', [this]()
                               { 
                                   // Stop movement
                                   if (m_controller->isWalking()) {
                                       return m_controller->stopLocomotion();
                                   }
                                   return true; });
            registerKeyCommand('q', [this]()
                               { 
            Application::m_running = false; 
            return true; });

            // Additional function keys
            registerKeyCommand('t', [this]()
                               {
            Application::m_telemetryActive = !Application::m_telemetryActive;
            std::cout << "Telemetry " 
                      << (Application::m_telemetryActive ? "enabled" : "disabled") 
                      << std::endl;
            return true; });

            registerKeyCommand('c', [this]()
                               {
            std::cout << "Centering all legs..." << std::endl;
            return m_hexapod->centerAll(); });

            registerKeyCommand('h', [this]()
                               {
            printHelp();
            return true; });

            registerKeyCommand('p', [this]()
                               {
            m_performanceMonitoringEnabled = !m_performanceMonitoringEnabled;
            std::cout << "Performance monitoring " 
                      << (m_performanceMonitoringEnabled ? "enabled" : "disabled") 
                      << std::endl;
            return true; });

            // Add balance mode controls - now implemented for CPG
            registerKeyCommand('b', [this]()
                               { return toggleBalanceMode(); });

            registerKeyCommand('[', [this]()
                               { return decreaseBalanceResponse(); });

            registerKeyCommand(']', [this]()
                               { return increaseBalanceResponse(); });

            // Autonomous mode controls
            registerKeyCommand('z', [this]()
                               { return toggleAutonomousMode(); });

            registerKeyCommand('x', [this]()
                               { return toggleObstacleDetection(); });

            registerKeyCommand('v', [this]()
                               {
                // Emergency stop - works in both manual and autonomous mode
                std::cout << "EMERGENCY STOP - All movement halted" << std::endl;
                m_autonomousMode = false;
                if (m_controller->isWalking()) {
                    return m_controller->stopLocomotion();
                }
                return true; });
        }

        /**
         * @brief Display help information
         */
        void printHelp() const
        {
            std::cout << "\nHexapod Controller - Help\n"
                      << "========================\n"
                      << "Movement Controls:\n"
                      << "  W/S: Forward/Backward\n"
                      << "  A/D: Rotate Left/Right\n"
                      << "  I/K: Raise/Lower\n"
                      << "  J/L: Tilt Left/Right\n\n"
                      << "Gait Controls:\n"
                      << "  1: Tripod Gait\n"
                      << "  2: Wave Gait\n"
                      << "  3: Ripple Gait\n\n"
                      << "Autonomous Controls:\n"
                      << "  Z: Toggle autonomous mode\n"
                      << "  X: Toggle obstacle detection\n"
                      << "  V: Emergency stop\n\n"
                      << "System Controls:\n"
                      << "  Space: Stop and center\n"
                      << "  U: Toggle ultrasonic sensor\n"
                      << "  +/-: Increase/decrease speed\n"
                      << "  T: Toggle telemetry\n"
                      << "  P: Toggle performance monitoring\n"
                      << "  C: Center all legs\n"
                      << "  H: Show this help\n"
                      << "  Q: Quit\n"
                      << "  B: Toggle balance mode\n"
                      << "  [: Decrease balance response\n"
                      << "  ]: Increase balance response\n"
                      << std::endl;
        }

        /**
         * @brief Register a command handler for a specific key
         *
         * @param key Key character to register
         * @param command Function to execute when key is pressed
         */
        void registerKeyCommand(char key, KeyCommand command)
        {
            m_keyCommands[key] = command;
        }

        /**
         * @brief Execute a command for the given key
         *
         * @param key Key pressed
         * @return true if command execution successful
         * @return false if command execution failed
         */
        bool executeKeyCommand(char key)
        {
            auto it = m_keyCommands.find(key);
            if (it != m_keyCommands.end())
            {
                return it->second();
            }
            return true; // Unknown key, not an error
        }

        /**
         * @brief Process user input
         *
         * @return true if input processing successful
         * @return false if input processing failed
         */
        bool processInput()
        {
            char key;
            if (read(STDIN_FILENO, &key, 1) > 0)
            {
                return executeKeyCommand(key);
            }
            return true;
        }

        /**
         * @brief Update the robot state
         *
         * @return true if update successful
         * @return false if update failed
         */
        bool update()
        {
            // Calculate time delta for physics/animation
            auto now = std::chrono::high_resolution_clock::now();
            float deltaTime = std::chrono::duration<float>(now - m_lastUpdateTime).count();
            m_lastUpdateTime = now;

            // Update IMU feedback for balance control
            if (!updateIMUFeedback())
            {
                std::cerr << "Warning: Failed to update IMU feedback" << std::endl;
            }

            // Handle autonomous mode
            if (m_autonomousMode)
            {
                if (!handleAutonomousMode())
                {
                    std::cerr << "Warning: Autonomous mode error" << std::endl;
                }
            }

            // Update controller state with time delta
            return m_controller->update(deltaTime);
        }

        /**
         * @brief Update IMU feedback for balance control
         *
         * @return true if update successful
         * @return false if update failed
         */
        bool updateIMUFeedback()
        {
            if (!m_hexapod || !m_controller)
                return false;

            hexapod::ImuData imuData;
            if (m_hexapod->getImuData(imuData))
            {
                // Convert accelerometer data to orientation angles using getter methods (in g units)
                double ax = imuData.getAccelX();
                double ay = imuData.getAccelY();
                double az = imuData.getAccelZ();

                // Simple roll/pitch calculation from accelerometer
                double roll = atan2(ay, az);
                double pitch = atan2(-ax, sqrt(ay * ay + az * az));

                // Create angular velocity vector from gyroscope data (in deg/s)
                std::vector<double> angular_velocity = {
                    imuData.getGyroX(),
                    imuData.getGyroY(),
                    imuData.getGyroZ()};

                // Update CPG controller with balance feedback
                m_controller->updateBalanceFeedback(roll, pitch, angular_velocity);

                return true;
            }
            return false;
        }

        /**
         * @brief Handle autonomous navigation with obstacle avoidance
         *
         * @return true if handling successful
         * @return false if handling failed
         */
        bool handleAutonomousMode()
        {
            auto now = std::chrono::high_resolution_clock::now();

            // Check for obstacles periodically
            if (m_obstacleDetectionEnabled &&
                std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastObstacleCheck).count() >= m_obstacleCheckInterval)
            {
                m_lastObstacleCheck = now;

                if (m_ultrasonicSensor)
                {
                    auto measurement = m_ultrasonicSensor->measure();
                    if (measurement.valid)
                    {
                        return handleObstacleDetection(measurement.distance);
                    }
                }
            }

            // Enhanced autonomous behavior with IMU feedback
            if (!m_controller->isWalking())
            {
                // Get current IMU state for navigation decisions
                hexapod::ImuData imuData;
                if (m_hexapod->getImuData(imuData))
                {
                    // Check stability before starting movement
                    double ax = imuData.getAccelX();
                    double ay = imuData.getAccelY();
                    double az = imuData.getAccelZ();

                    double roll = atan2(ay, az) * 180.0 / M_PI;
                    double pitch = atan2(-ax, sqrt(ax * ax + ay * ay)) * 180.0 / M_PI;

                    // Only start movement if hexapod is stable
                    if (std::abs(roll) < 20.0 && std::abs(pitch) < 20.0)
                    {
                        // Smart gait selection based on stability
                        std::string gait_type = "tripod"; // Default
                        double velocity = 0.2;            // Default speed

                        if (std::abs(roll) > 10.0 || std::abs(pitch) > 10.0)
                        {
                            // Use more stable gait when tilted
                            gait_type = "wave";
                            velocity = 0.1; // Slower speed for safety
                        }

                        cpg::LocomotionCommand cmd = cpg::controller_utils::createWalkCommand(velocity, 0.0, gait_type);
                        m_controller->startLocomotion(cmd);
                    }
                    else
                    {
                        // Too unstable - wait for stabilization
                        std::cout << "Waiting for stabilization (Roll: " << roll
                                  << "°, Pitch: " << pitch << "°)" << std::endl;
                    }
                }
                else
                {
                    // Fallback if IMU unavailable
                    cpg::LocomotionCommand cmd = cpg::controller_utils::createWalkCommand(0.15, 0.0, "wave");
                    m_controller->startLocomotion(cmd);
                }
            }

            return true;
        }

        /**
         * @brief Handle obstacle detection and avoidance
         *
         * @param distance Distance to obstacle in cm
         * @return true if handling successful
         * @return false if handling failed
         */
        bool handleObstacleDetection(float distance)
        {
            if (distance < m_safeDistance)
            {
                std::cout << "Obstacle detected at " << distance << "cm - initiating smart avoidance maneuver" << std::endl;

                // Stop current movement
                m_controller->stopLocomotion();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));

                // Get IMU data to determine best avoidance strategy
                hexapod::ImuData imuData;
                bool has_imu = m_hexapod->getImuData(imuData);

                // Determine turn direction and speed based on stability
                double turn_velocity = -0.3; // Default turn right
                std::string gait_type = "tripod";

                if (has_imu)
                {
                    double ax = imuData.getAccelX();
                    double ay = imuData.getAccelY();
                    double az = imuData.getAccelZ();

                    double roll = atan2(ay, az) * 180.0 / M_PI;
                    double pitch = atan2(-ax, sqrt(ax * ax + ay * ay)) * 180.0 / M_PI;

                    // Adjust maneuver based on current stability
                    if (std::abs(roll) > 10.0 || std::abs(pitch) > 10.0)
                    {
                        // Use more stable gait and slower speed when unstable
                        gait_type = "wave";
                        turn_velocity = turn_velocity * 0.5; // Slower turn
                    }

                    // Choose turn direction based on current tilt (turn away from tilt)
                    if (roll > 5.0)
                        turn_velocity = 0.3; // Turn left if tilted right
                    else if (roll < -5.0)
                        turn_velocity = -0.3; // Turn right if tilted left

                    std::cout << "IMU-guided avoidance: Roll=" << roll << "°, Pitch=" << pitch
                              << "°, Turn=" << turn_velocity << " rad/s" << std::endl;
                }

                // Execute turn maneuver
                cpg::LocomotionCommand turnCommand = cpg::controller_utils::createWalkCommand(0.0, turn_velocity, gait_type);
                m_controller->startLocomotion(turnCommand);
                std::this_thread::sleep_for(std::chrono::milliseconds(800));

                // Stop and stabilize
                m_controller->stopLocomotion();
                std::this_thread::sleep_for(std::chrono::milliseconds(300));

                // Move forward cautiously with stability monitoring
                double forward_speed = has_imu ? 0.15 : 0.1; // Slower if no IMU
                cpg::LocomotionCommand forwardCommand = cpg::controller_utils::createWalkCommand(forward_speed, 0.0, gait_type);
                m_controller->startLocomotion(forwardCommand);

                return true;
            }
            else if (distance < m_safeDistance * 1.5)
            {
                // Slow down when approaching obstacle
                m_controller->setLinearVelocity(0.1);
                if (!m_controller->isWalking())
                {
                    cpg::LocomotionCommand slowCommand = cpg::controller_utils::createWalkCommand(0.1, 0.0, "tripod");
                    m_controller->startLocomotion(slowCommand);
                }
            }

            return true;
        }

        /**
         * @brief Toggle autonomous mode
         *
         * @return true if toggle successful
         * @return false if toggle failed
         */
        bool toggleAutonomousMode()
        {
            m_autonomousMode = !m_autonomousMode;

            if (m_autonomousMode)
            {
                std::cout << "Autonomous mode ENABLED - Robot will navigate and avoid obstacles automatically" << std::endl;
                if (!m_obstacleDetectionEnabled)
                {
                    std::cout << "Warning: Obstacle detection is disabled" << std::endl;
                }
            }
            else
            {
                std::cout << "Autonomous mode DISABLED - Manual control restored" << std::endl;
                // Stop any current movement
                if (m_controller && m_controller->isWalking())
                {
                    m_controller->stopLocomotion();
                }
            }

            return true;
        }

        /**
         * @brief Toggle obstacle detection
         *
         * @return true if toggle successful
         * @return false if toggle failed
         */
        bool toggleObstacleDetection()
        {
            if (m_ultrasonicSensor)
            {
                m_obstacleDetectionEnabled = !m_obstacleDetectionEnabled;
                std::cout << "Obstacle detection "
                          << (m_obstacleDetectionEnabled ? "ENABLED" : "DISABLED")
                          << std::endl;
            }
            else
            {
                std::cout << "Obstacle detection unavailable - sensor not initialized" << std::endl;
            }
            return true;
        }

        /**
         * @brief Update performance metrics
         *
         * @param frameTime Time taken to process the current frame
         */
        void updatePerformanceMetrics(const std::chrono::microseconds &frameTime)
        {
            m_frameCount++;
            m_totalFrameTime += frameTime.count();
            m_maxFrameTime = std::max(m_maxFrameTime, (unsigned long)frameTime.count());
        }

        /**
         * @brief Display telemetry information
         */
        void displayTelemetry()
        {
            // Get current hexapod state
            if (!m_hexapod || !m_controller)
                return;

            hexapod::ImuData imuData;
            if (m_hexapod->getImuData(imuData))
            {
                std::cout << "\033[2J\033[H"; // Clear screen and move cursor to home
                std::cout << "=== Hexapod Telemetry ===\n";

                // Display orientation
                std::cout << "Orientation:\n";
                std::cout << "  Accel: X=" << std::setw(6) << imuData.accel_x
                          << " Y=" << std::setw(6) << imuData.accel_y
                          << " Z=" << std::setw(6) << imuData.accel_z << "\n";

                std::cout << "  Gyro:  X=" << std::setw(6) << imuData.gyro_x
                          << " Y=" << std::setw(6) << imuData.gyro_y
                          << " Z=" << std::setw(6) << imuData.gyro_z << "\n";

                // Display controller state
                std::cout << "Controller state: ";
                if (m_controller->isWalking())
                {
                    std::cout << "WALKING";
                }
                else if (m_controller->isTransitioning())
                {
                    std::cout << "TRANSITIONING";
                }
                else
                {
                    std::cout << "IDLE";
                }
                std::cout << "\n";

                // Display autonomous mode status
                std::cout << "Mode: " << (m_autonomousMode ? "AUTONOMOUS" : "MANUAL") << "\n";
                std::cout << "Obstacle Detection: " << (m_obstacleDetectionEnabled ? "ENABLED" : "DISABLED") << "\n";

                // Display distance sensor data if available
                if (m_obstacleDetectionEnabled && m_ultrasonicSensor)
                {
                    auto measurement = m_ultrasonicSensor->measure();
                    if (measurement.valid)
                    {
                        std::cout << "Distance: " << std::fixed << std::setprecision(1)
                                  << measurement.distance << "cm";
                        if (measurement.distance < m_safeDistance)
                        {
                            std::cout << " [OBSTACLE DETECTED!]";
                        }
                        std::cout << "\n";
                    }
                    else
                    {
                        std::cout << "Distance: INVALID\n";
                    }
                }

                // Show current system status
                double avgFrameTime = m_frameCount > 0 ? (m_totalFrameTime / m_frameCount) / 1000.0 : 0;
                std::cout << "System: " << std::fixed << std::setprecision(2)
                          << "Avg frame: " << avgFrameTime << "ms, "
                          << "Max frame: " << (m_maxFrameTime / 1000.0) << "ms, "
                          << "FPS: " << (1000.0 / std::max(1.0, avgFrameTime)) << "\n";

                // CPG-specific status information
                if (m_controller)
                {
                    std::cout << "CPG Gait: " << m_controller->getCurrentGait() << "\n";
                    std::cout << "Linear Vel: " << m_controller->getLinearVelocity() << " m/s\n";
                    std::cout << "Angular Vel: " << m_controller->getAngularVelocity() << " rad/s\n";
                }

                std::cout << "\nPress 'h' for help, 'q' to quit, 't' to hide telemetry\n";
                std::cout << "Press 'z' to toggle autonomous mode, 'v' for emergency stop\n";
            }
        }

        /**
         * @brief Report performance metrics
         */
        void reportPerformance()
        {
            if (m_frameCount == 0)
                return;

            double avgFrameTime = (m_totalFrameTime / m_frameCount) / 1000.0;
            double fps = 1000.0 / std::max(1.0, avgFrameTime);

            std::cout << "[Performance] "
                      << "Frames: " << m_frameCount
                      << ", Avg: " << std::fixed << std::setprecision(2) << avgFrameTime << "ms"
                      << ", Max: " << (m_maxFrameTime / 1000.0) << "ms"
                      << ", FPS: " << fps
                      << std::endl;

            // Reset metrics for next period
            m_frameCount = 0;
            m_totalFrameTime = 0;
            m_maxFrameTime = 0;
        }

        /**
         * @brief Load configuration from file
         *
         * @return true if load successful
         * @return false if load failed
         */
        bool loadConfiguration()
        {
            // Implementation would load from a config file
            // For now, just use default values
            m_updateInterval = 0.01f; // 10ms default
            return true;
        }

        /**
         * @brief Save configuration to file
         *
         * @return true if save successful
         * @return false if save failed
         */
        bool saveConfiguration()
        {
            // Implementation would save to a config file
            return true;
        }

        /**
         * @brief Switch control mode
         *
         * @param mode New control mode
         * @return true if switch successful
         * @return false if switch failed
         */
        bool switchMode(ControlMode mode)
        {
            // Implementation for switching between control modes
            if (m_currentMode == mode)
            {
                return true; // Already in this mode
            }

            // Cleanup current mode - ensure clean transition
            if (m_controller)
            {
                m_controller->stopLocomotion(); // Stop movement and center legs between mode switches
            }

            // Initialize new mode
            m_currentMode = mode;
            return true;
        }

        /**
         * @brief Toggle balance mode for CPG controller
         *
         * @return true if toggle successful
         * @return false if toggle failed
         */
        bool toggleBalanceMode()
        {
            if (!m_controller)
            {
                std::cout << "Balance mode unavailable - controller not initialized" << std::endl;
                return false;
            }

            auto config = m_controller->getConfiguration();
            config.balance_control = !config.balance_control;

            if (m_controller->configure(config))
            {
                std::cout << "Balance mode "
                          << (config.balance_control ? "ENABLED" : "DISABLED")
                          << " - CPG controller will "
                          << (config.balance_control ? "use" : "ignore")
                          << " IMU feedback for stability" << std::endl;
                return true;
            }
            else
            {
                std::cout << "Failed to toggle balance mode" << std::endl;
                return false;
            }
        }

        /**
         * @brief Decrease balance response sensitivity
         *
         * @return true if decrease successful
         * @return false if decrease failed
         */
        bool decreaseBalanceResponse()
        {
            if (!m_controller)
            {
                std::cout << "Balance control unavailable - controller not initialized" << std::endl;
                return false;
            }

            auto config = m_controller->getConfiguration();
            if (config.balance_control)
            {
                // Increase stability threshold (less sensitive)
                config.stability_threshold = std::min(config.stability_threshold + 0.05, 0.5);

                if (m_controller->configure(config))
                {
                    std::cout << "Balance response decreased - Stability threshold: "
                              << config.stability_threshold << std::endl;
                    return true;
                }
            }
            else
            {
                std::cout << "Balance mode is disabled - enable it first with 'b'" << std::endl;
            }
            return false;
        }

        /**
         * @brief Increase balance response sensitivity
         *
         * @return true if increase successful
         * @return false if increase failed
         */
        bool increaseBalanceResponse()
        {
            if (!m_controller)
            {
                std::cout << "Balance control unavailable - controller not initialized" << std::endl;
                return false;
            }

            auto config = m_controller->getConfiguration();
            if (config.balance_control)
            {
                // Decrease stability threshold (more sensitive)
                config.stability_threshold = std::max(config.stability_threshold - 0.05, 0.05);

                if (m_controller->configure(config))
                {
                    std::cout << "Balance response increased - Stability threshold: "
                              << config.stability_threshold << std::endl;
                    return true;
                }
            }
            else
            {
                std::cout << "Balance mode is disabled - enable it first with 'b'" << std::endl;
            }
            return false;
        }

        // Member variables
        std::string m_lastError;
        ControlMode m_currentMode;

        // Core components
        std::unique_ptr<hexapod::Hexapod> m_hexapod;
        std::unique_ptr<cpg::Controller> m_controller;
        std::unique_ptr<UltrasonicSensor> m_ultrasonicSensor;

        // Key command handling
        std::unordered_map<char, KeyCommand> m_keyCommands;

        // Performance monitoring
        std::chrono::high_resolution_clock::time_point m_lastUpdateTime;
        float m_updateInterval; // seconds between updates
        unsigned long m_frameCount;
        unsigned long m_totalFrameTime; // microseconds
        unsigned long m_maxFrameTime;   // microseconds
        bool m_performanceMonitoringEnabled;

        // Autonomous mode variables
        bool m_autonomousMode;
        bool m_obstacleDetectionEnabled;
        float m_safeDistance; // Safe distance from obstacles in cm
        std::chrono::high_resolution_clock::time_point m_lastObstacleCheck;
        int m_obstacleCheckInterval; // Obstacle check interval in milliseconds
    };

    //==============================================================================
    // Application Class Implementation
    //==============================================================================

    // Singleton implementation
    Application &Application::getInstance()
    {
        static Application instance;
        return instance;
    }

    Application::Application()
        : pImpl(std::make_unique<ApplicationImpl>())
    {

        // Load configuration
        pImpl->loadConfiguration();
    }

    Application::~Application()
    {
        shutdown();
    }

    void Application::signalHandler(int signal)
    {
        std::cout << "\nReceived signal " << signal << std::endl;
        m_running = false;
    }

    bool Application::init()
    {
        // Register signal handlers
        signal(SIGINT, Application::signalHandler);
        signal(SIGTERM, Application::signalHandler);

        // Initialize components with improved error handling
        if (!pImpl->initializeHexapod())
        {
            std::cerr << "Failed to initialize hexapod: " << pImpl->m_lastError << std::endl;
            return false;
        }

        if (!pImpl->initializeController())
        {
            std::cerr << "Failed to initialize controller: " << pImpl->m_lastError << std::endl;
            return false;
        }

        if (!pImpl->setupInputHandling())
        {
            std::cerr << "Failed to set up input handling: " << pImpl->m_lastError << std::endl;
            return false;
        }

        // Setup key command mappings
        pImpl->setupKeyCommands();

        std::cout << "Application initialized successfully" << std::endl;
        return true;
    }

    ExecutionResult Application::run()
    {
        // Print instructions and controls
        std::cout << "\nHexapod Controller Started" << std::endl;
        pImpl->printHelp();

        // Reset timer for accurate performance tracking
        auto lastUpdateTime = std::chrono::high_resolution_clock::now();
        auto lastTelemetryTime = lastUpdateTime;
        auto lastPerformanceReportTime = lastUpdateTime;

        // Main control loop
        m_running = true;
        pImpl->m_frameCount = 0;
        pImpl->m_totalFrameTime = 0;
        pImpl->m_maxFrameTime = 0;

        std::chrono::high_resolution_clock::time_point frameStart;
        std::chrono::microseconds frameTime;
        const std::chrono::microseconds targetFrameTime(static_cast<int>(pImpl->m_updateInterval * 1000000));

        // Initial leg centering for safety
        if (!pImpl->m_hexapod->centerAll())
        {
            std::cerr << "Warning: Failed to center legs at startup" << std::endl;
        }

        std::cout << "Ready. Use 'h' for help, 'q' to quit." << std::endl;

        while (m_running)
        {
            frameStart = std::chrono::high_resolution_clock::now();

            // Process input (non-blocking)
            if (!pImpl->processInput())
            {
                pImpl->m_lastError = "Input processing error";
                return ExecutionResult::ERROR_RUNTIME;
            }

            // Update hexapod state
            if (!pImpl->update())
            {
                pImpl->m_lastError = "Update error";
                return ExecutionResult::ERROR_RUNTIME;
            }

            // Calculate frame time and update stats
            auto now = std::chrono::high_resolution_clock::now();
            frameTime = std::chrono::duration_cast<std::chrono::microseconds>(now - frameStart);

            // Update performance metrics
            pImpl->updatePerformanceMetrics(frameTime);

            // Display telemetry periodically
            if (m_telemetryActive &&
                std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTelemetryTime).count() >= 500)
            {
                pImpl->displayTelemetry();
                lastTelemetryTime = now;
            }

            // Report performance metrics periodically
            if (pImpl->m_performanceMonitoringEnabled &&
                std::chrono::duration_cast<std::chrono::seconds>(now - lastPerformanceReportTime).count() >= 5)
            {
                pImpl->reportPerformance();
                lastPerformanceReportTime = now;
            }

            // Sleep for the remaining time to maintain target frame rate
            if (frameTime < targetFrameTime)
            {
                std::this_thread::sleep_for(targetFrameTime - frameTime);
            }
            else if (frameTime > std::chrono::milliseconds(100))
            {
                // Log warning for significant frame time overruns
                std::cerr << "Warning: Long frame time: "
                          << frameTime.count() / 1000.0 << "ms" << std::endl;
            }
        }

        return ExecutionResult::TERMINATED_BY_USER;
    }

    void Application::shutdown()
    {
        std::cout << "\nShutting down application..." << std::endl;

        // Center legs for safety before shutdown
        if (pImpl->m_controller)
        {
            std::cout << "Stopping movement and centering legs..." << std::endl;
            pImpl->m_controller->stopLocomotion(); // Stop movement and center legs

            // Give some time for the command to complete
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // Save configuration if needed
        pImpl->saveConfiguration();

        // Clean up resources (smart pointers will handle deallocation)
        std::cout << "Releasing resources..." << std::endl;
        pImpl->m_controller.reset();
        pImpl->m_hexapod.reset();

        std::cout << "Shutdown complete" << std::endl;
    }

    bool Application::switchMode(ControlMode mode)
    {
        return pImpl->switchMode(mode);
    }

    ControlMode Application::getCurrentMode() const
    {
        return pImpl->m_currentMode;
    }

    std::string Application::getLastErrorMessage() const
    {
        return pImpl->m_lastError;
    }

} // namespace application
