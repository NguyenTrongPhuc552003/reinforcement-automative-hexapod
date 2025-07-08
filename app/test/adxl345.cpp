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

#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include "hexapod.hpp"
#include "common.hpp"

// Flag to control the main loop
static std::atomic<bool> running(true);

// Signal handler
void signalHandler(int signal)
{
    std::cout << "\nReceived signal " << signal << ", exiting...\n";
    running = false;
}

// Function to display accelerometer data
void displayAccelerometerData(const hexapod::ImuData &data)
{
    // Calculate pitch and roll from accelerometer data
    double ax = data.getAccelX();
    double ay = data.getAccelY();
    double az = data.getAccelZ();

    // Calculate tilt angles in degrees
    double roll = atan2(ay, az) * 180.0 / M_PI;
    double pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

    // ANSI escape codes for text formatting
    const char *reset = "\033[0m";
    const char *bold = "\033[1m";
    const char *blue = "\033[34m";
    const char *green = "\033[32m";

    // Clear screen and move cursor to home position
    std::cout << "\033[2J\033[H";

    // Display header
    std::cout << bold << "ADXL345 Accelerometer Test" << reset << "\n";
    std::cout << "=============================\n\n";

    // Display sensor type
    std::cout << "Sensor: " << (data.getSensorType() == hexapod::SensorType::ADXL345 ? green : blue) << (data.getSensorType() == hexapod::SensorType::ADXL345 ? "ADXL345 (3-axis)" : "MPU6050 (6-axis)") << reset << "\n\n";

    // Display raw accelerometer values
    std::cout << "Raw Acceleration Values:\n";
    std::cout << "  X: " << std::setw(6) << data.accel_x << "\n";
    std::cout << "  Y: " << std::setw(6) << data.accel_y << "\n";
    std::cout << "  Z: " << std::setw(6) << data.accel_z << "\n\n";

    // Display converted accelerometer values (g units)
    std::cout << "Acceleration (g):\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  X: " << std::setw(7) << data.getAccelX() << " g\n";
    std::cout << "  Y: " << std::setw(7) << data.getAccelY() << " g\n";
    std::cout << "  Z: " << std::setw(7) << data.getAccelZ() << " g\n\n";

    // Display orientation
    std::cout << "Orientation:\n";
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "  Roll:  " << std::setw(6) << roll << " degrees\n";
    std::cout << "  Pitch: " << std::setw(6) << pitch << " degrees\n\n";

    // Visual representation of tilt
    const int barWidth = 40;
    int rollPos = static_cast<int>((roll + 90.0) / 180.0 * barWidth);
    int pitchPos = static_cast<int>((pitch + 90.0) / 180.0 * barWidth);

    std::cout << "Roll  [-90°]: ";
    for (int i = 0; i < barWidth; ++i)
    {
        std::cout << (i == rollPos ? "O" : (i == barWidth / 2 ? "|" : "-"));
    }
    std::cout << " [+90°]\n";

    std::cout << "Pitch [-90°]: ";
    for (int i = 0; i < barWidth; ++i)
    {
        std::cout << (i == pitchPos ? "O" : (i == barWidth / 2 ? "|" : "-"));
    }
    std::cout << " [+90°]\n\n";

    // Show help
    std::cout << bold << "Commands:" << reset << "\n";
    std::cout << "  m: Switch between MPU6050 and ADXL345 sensors\n";
    std::cout << "  q: Quit\n\n";
}

int main()
{
    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "Initializing ADXL345 test program...\n";

    // Initialize hexapod hardware interface
    hexapod::Hexapod hexapod;
    if (!hexapod.init())
    {
        std::cerr << "Failed to initialize hexapod: " << hexapod.getLastErrorMessage() << std::endl;
        return 1;
    }

    // Set up non-blocking terminal input
    common::TerminalManager::setupNonBlocking();

    // Switch to ADXL345 sensor if available
    std::cout << "Switching to ADXL345 sensor...\n";
    hexapod.setSensorType(hexapod::SensorType::ADXL345);

    // Get current sensor type to confirm
    hexapod::SensorType currentSensor;
    if (hexapod.getSensorType(currentSensor))
    {
        std::cout << "Active sensor: " << (currentSensor == hexapod::SensorType::ADXL345 ? "ADXL345" : currentSensor == hexapod::SensorType::MPU6050 ? "MPU6050"
                                                                                                                                                     : "AUTO")
                  << "\n";
    }
    else
    {
        std::cerr << "Failed to get current sensor type\n";
    }

    std::cout << "Press 'q' to quit, 'm' to toggle sensor type\n";

    // Main loop
    while (running)
    {
        // Check for keyboard input
        char ch;
        if (common::TerminalManager::readChar(ch))
        {
            if (ch == 'q')
            {
                running = false;
                break;
            }
            else if (ch == 'm')
            {
                // Toggle between sensors
                hexapod::SensorType newType =
                    (currentSensor == hexapod::SensorType::ADXL345) ? hexapod::SensorType::MPU6050 : hexapod::SensorType::ADXL345;

                if (hexapod.setSensorType(newType))
                {
                    currentSensor = newType;
                    std::cout << "\nSwitched to " << (currentSensor == hexapod::SensorType::ADXL345 ? "ADXL345" : "MPU6050") << " sensor\n";
                }
                else
                {
                    std::cerr << "\nFailed to switch sensor type\n";
                }
            }
        }

        // Read IMU data
        hexapod::ImuData imuData;
        if (hexapod.getImuData(imuData))
        {
            displayAccelerometerData(imuData);
        }
        else
        {
            std::cerr << "\nFailed to read IMU data: " << hexapod.getLastErrorMessage() << std::endl;
        }

        // Sleep to prevent high CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Restore terminal settings
    common::TerminalManager::restore();

    std::cout << "ADXL345 test program terminated.\n";
    return 0;
}
