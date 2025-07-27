#include "ultrasonic.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <csignal>

static bool running = true;

void signalHandler(int signal)
{
    std::cout << "\nReceived signal " << signal << ". Stopping..." << std::endl;
    running = false;
}

int main()
{
    std::cout << "=== HC-SR04 Ultrasonic Test (Updated Logic) ===" << std::endl;
    std::cout << "TRIG Pin: P8_12 (GPIO44)" << std::endl;
    std::cout << "ECHO Pin: P8_11 (GPIO45)" << std::endl;
    std::cout << "Obstacle threshold: 20.0 cm" << std::endl;
    std::cout << "Safe distance: 30.0 cm" << std::endl;
    std::cout << "=========================================" << std::endl;

    // Setup signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    Ultrasonic sensor;

    if (!sensor.init())
    {
        std::cerr << "Failed to initialize ultrasonic sensor" << std::endl;
        return -1;
    }

    std::cout << "Ultrasonic sensor initialized successfully" << std::endl;
    std::cout << "\nStarting distance measurements..." << std::endl;
    std::cout << "Press Ctrl+C to stop\n" << std::endl;

    int measurement_count = 0;
    double total_distance = 0.0;
    double min_distance = 999.0;
    double max_distance = 0.0;
    int valid_readings = 0;
    int error_readings = 0;
    int obstacle_count = 0;

    while (running)
    {
        measurement_count++;
        double distance = sensor.getDistance();

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "[" << std::setw(4) << measurement_count << "] ";

        if (distance >= 0)
        {
            valid_readings++;
            total_distance += distance;

            if (distance < min_distance)
                min_distance = distance;

            if (distance > max_distance)
                max_distance = distance;

            std::cout << "Distance: " << std::setw(7) << distance << " cm";

            // Status indicators based on hexapod obstacle thresholds
            if (distance < 20.0)
            {
                obstacle_count++;
                std::cout << " OBSTACLE DETECTED! (< 20cm)";
            }
            else if (distance < 30.0)
            {
                std::cout << " WARNING ZONE (< 30cm)";
            }
            else if (distance < 50.0)
            {
                std::cout << " SAFE DISTANCE";
            }
            else
            {
                std::cout << " CLEAR PATH";
            }
        }
        else
        {
            error_readings++;
            std::cout << "ERROR: Timeout or invalid reading";
        }

        // Show statistics every 10 readings
        if (measurement_count % 10 == 0)
        {
            std::cout << "\nStatistics after " << measurement_count << " readings:" << std::endl;
            std::cout << "   Valid readings: " << valid_readings << " (" 
                      << (valid_readings * 100.0 / measurement_count) << "%)" << std::endl;
            std::cout << "   Error readings: " << error_readings << " (" 
                      << (error_readings * 100.0 / measurement_count) << "%)" << std::endl;
            
            if (valid_readings > 0)
            {
                std::cout << "   Average distance: " << (total_distance / valid_readings) << " cm" << std::endl;
                std::cout << "   Min distance: " << min_distance << " cm" << std::endl;
                std::cout << "   Max distance: " << max_distance << " cm" << std::endl;
                std::cout << "   Obstacles detected: " << obstacle_count << " times" << std::endl;
            }
            std::cout << "---" << std::endl;
        }

        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 0.5s delay like working code
    }

    sensor.cleanup();
    
    std::cout << "\nFinal Statistics:" << std::endl;
    std::cout << "Total measurements: " << measurement_count << std::endl;
    std::cout << "Success rate: " << (valid_readings * 100.0 / measurement_count) << "%" << std::endl;
    if (valid_readings > 0)
    {
        std::cout << "Average distance: " << (total_distance / valid_readings) << " cm" << std::endl;
        std::cout << "Distance range: " << min_distance << " - " << max_distance << " cm" << std::endl;
    }
    
    std::cout << "\nHC-SR04 test completed successfully!" << std::endl;
    return 0;
}
