#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <vector>
#include <algorithm>
#include <numeric>
#include <signal.h>
#include <unistd.h>
#include <termios.h>
#include <cmath>
#include "ultrasonic.hpp"
#include "common.hpp"

// Global flags for signal handling
static std::atomic<bool> g_running(true);

/**
 * @brief Signal handler for graceful shutdown
 */
void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    g_running = false;
}

/**
 * @brief Test configuration structure
 */
struct TestConfig {
    std::string triggerPin = "P9_12";
    std::string echoPin = "P9_14";
    bool interactive = false;
    bool continuousMode = false;
    int numSamples = 10;
    int delayMs = 100;
    bool verbose = false;
    float alertDistance = 20.0f;
};

/**
 * @brief Display usage information
 */
void printUsage(const char* programName) {
    std::cout << "HC-SR04 Ultrasonic Sensor Test Program\n";
    std::cout << "Usage: " << programName << " [options]\n\n";
    std::cout << "Options:\n";
    std::cout << "  -t <pin>    Trigger pin (default: P9_12)\n";
    std::cout << "  -e <pin>    Echo pin (default: P9_14)\n";
    std::cout << "  -i          Interactive mode\n";
    std::cout << "  -c          Continuous measurement mode\n";
    std::cout << "  -n <num>    Number of samples for batch test (default: 10)\n";
    std::cout << "  -d <ms>     Delay between measurements in ms (default: 100)\n";
    std::cout << "  -v          Verbose output\n";
    std::cout << "  -a <dist>   Alert distance threshold in cm (default: 20.0)\n";
    std::cout << "  -h          Show this help\n\n";
    std::cout << "Examples:\n";
    std::cout << "  " << programName << " -i                    # Interactive mode\n";
    std::cout << "  " << programName << " -c -v                 # Continuous with verbose output\n";
    std::cout << "  " << programName << " -t P9_15 -e P9_16     # Use different pins\n";
    std::cout << "  " << programName << " -n 50 -d 50           # 50 samples with 50ms delay\n";
}

/**
 * @brief Parse command line arguments
 */
TestConfig parseArguments(int argc, char* argv[]) {
    TestConfig config;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            exit(0);
        } else if (arg == "-t" && i + 1 < argc) {
            config.triggerPin = argv[++i];
        } else if (arg == "-e" && i + 1 < argc) {
            config.echoPin = argv[++i];
        } else if (arg == "-i") {
            config.interactive = true;
        } else if (arg == "-c") {
            config.continuousMode = true;
        } else if (arg == "-n" && i + 1 < argc) {
            config.numSamples = std::max(1, std::atoi(argv[++i]));
        } else if (arg == "-d" && i + 1 < argc) {
            config.delayMs = std::max(10, std::atoi(argv[++i]));
        } else if (arg == "-v") {
            config.verbose = true;
        } else if (arg == "-a" && i + 1 < argc) {
            config.alertDistance = std::max(1.0f, std::stof(argv[++i]));
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            printUsage(argv[0]);
            exit(1);
        }
    }
    
    return config;
}

/**
 * @brief Setup terminal for non-blocking input
 */
struct termios setupTerminal() {
    struct termios oldTermios, newTermios;
    
    tcgetattr(STDIN_FILENO, &oldTermios);
    newTermios = oldTermios;
    newTermios.c_lflag &= ~(ICANON | ECHO);
    newTermios.c_cc[VMIN] = 0;
    newTermios.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newTermios);
    
    return oldTermios;
}

/**
 * @brief Restore terminal settings
 */
void restoreTerminal(const struct termios& oldTermios) {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldTermios);
}

/**
 * @brief Check for keyboard input (non-blocking)
 */
bool kbhit() {
    int ch = getchar();
    if (ch != EOF) {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}

/**
 * @brief Basic sensor functionality test
 */
bool basicTest(UltrasonicSensor& sensor, const TestConfig& config) {
    std::cout << "\n=== Basic Functionality Test ===\n";
    
    // Test initialization
    std::cout << "Initializing sensor... ";
    if (!sensor.init()) {
        std::cout << "FAILED\n";
        std::cout << "Error: " << sensor.getLastError() << std::endl;
        return false;
    }
    std::cout << "OK\n";
    
    // Test self-test
    std::cout << "Running self-test... ";
    if (!sensor.selfTest()) {
        std::cout << "FAILED\n";
        std::cout << "Error: " << sensor.getLastError() << std::endl;
        return false;
    }
    std::cout << "OK\n";
    
    // Test single measurement
    std::cout << "Taking single measurement... ";
    auto measurement = sensor.measure();
    if (!measurement.valid) {
        std::cout << "FAILED\n";
        std::cout << "Error: " << sensor.getLastError() << std::endl;
        return false;
    }
    std::cout << "OK (" << std::fixed << std::setprecision(2) 
              << measurement.distance << " cm)\n";
    
    if (config.verbose) {
        std::cout << "  Raw distance: " << measurement.rawDistance << " cm\n";
        std::cout << "  Echo time: " << measurement.echoTimeUs << " μs\n";
        std::cout << "  Timestamp: " << measurement.timestamp.time_since_epoch().count() << "\n";
    }
    
    return true;
}

/**
 * @brief Batch measurement test
 */
bool batchTest(UltrasonicSensor& sensor, const TestConfig& config) {
    std::cout << "\n=== Batch Measurement Test ===\n";
    std::cout << "Taking " << config.numSamples << " measurements with " 
              << config.delayMs << "ms intervals...\n";
    
    std::vector<float> distances;
    std::vector<int> echoTimes;
    int validCount = 0;
    
    auto startTime = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < config.numSamples; i++) {
        auto measurement = sensor.measure();
        
        if (measurement.valid) {
            distances.push_back(measurement.distance);
            echoTimes.push_back(measurement.echoTimeUs);
            validCount++;
            
            if (config.verbose) {
                std::cout << "  Sample " << std::setw(3) << (i + 1) << ": " 
                          << std::fixed << std::setprecision(2) << measurement.distance 
                          << " cm (" << measurement.echoTimeUs << " μs)\n";
            } else {
                // Progress indicator
                if ((i + 1) % (config.numSamples / 10) == 0 || i == config.numSamples - 1) {
                    std::cout << "\rProgress: " << std::setw(3) 
                              << (100 * (i + 1) / config.numSamples) << "%" << std::flush;
                }
            }
        } else {
            if (config.verbose) {
                std::cout << "  Sample " << std::setw(3) << (i + 1) << ": INVALID ("
                          << sensor.getLastError() << ")\n";
            }
        }
        
        if (i < config.numSamples - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(config.delayMs));
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    
    std::cout << "\n\nResults:\n";
    std::cout << "  Valid measurements: " << validCount << "/" << config.numSamples 
              << " (" << std::fixed << std::setprecision(1) 
              << (100.0f * validCount / config.numSamples) << "%)\n";
    std::cout << "  Total time: " << totalTime.count() << " ms\n";
    std::cout << "  Average rate: " << std::fixed << std::setprecision(1) 
              << (1000.0f * validCount / totalTime.count()) << " Hz\n";
    
    if (!distances.empty()) {
        // Calculate statistics
        std::sort(distances.begin(), distances.end());
        float min = distances.front();
        float max = distances.back();
        float median = distances[distances.size() / 2];
        float mean = std::accumulate(distances.begin(), distances.end(), 0.0f) / distances.size();
        
        // Calculate standard deviation
        float variance = 0.0f;
        for (float d : distances) {
            variance += (d - mean) * (d - mean);
        }
        variance /= distances.size();
        float stdDev = std::sqrt(variance);
        
        std::cout << "\nStatistics:\n";
        std::cout << "  Range: " << std::fixed << std::setprecision(2) 
                  << min << " - " << max << " cm\n";
        std::cout << "  Mean: " << mean << " cm\n";
        std::cout << "  Median: " << median << " cm\n";
        std::cout << "  Std Dev: " << stdDev << " cm\n";
        std::cout << "  Coefficient of Variation: " << std::fixed << std::setprecision(1) 
                  << (100.0f * stdDev / mean) << "%\n";
    }
    
    return validCount > 0;
}

/**
 * @brief Averaging test
 */
bool averagingTest(UltrasonicSensor& sensor, const TestConfig& config) {
    std::cout << "\n=== Averaging Test ===\n";
    
    // Test different averaging window sizes
    std::vector<int> sampleCounts = {1, 3, 5, 10};
    
    for (int samples : sampleCounts) {
        std::cout << "Testing " << samples << "-sample average... ";
        
        auto measurement = sensor.measureAverage(samples, 20);
        
        if (measurement.valid) {
            std::cout << std::fixed << std::setprecision(2) << measurement.distance << " cm\n";
            
            if (config.verbose) {
                std::cout << "  Raw: " << measurement.rawDistance << " cm, "
                          << "Echo: " << measurement.echoTimeUs << " μs\n";
            }
        } else {
            std::cout << "FAILED (" << sensor.getLastError() << ")\n";
        }
    }
    
    return true;
}

/**
 * @brief Object detection test
 */
bool detectionTest(UltrasonicSensor& sensor, const TestConfig& config) {
    std::cout << "\n=== Object Detection Test ===\n";
    std::cout << "Alert distance: " << config.alertDistance << " cm\n";
    std::cout << "Move an object closer/farther to test detection...\n";
    std::cout << "Press any key to stop.\n\n";
    
    auto oldTermios = setupTerminal();
    
    int detectionCount = 0;
    int totalChecks = 0;
    
    while (g_running && !kbhit()) {
        bool detected = sensor.isObjectDetected(config.alertDistance);
        auto measurement = sensor.measure();
        
        totalChecks++;
        if (detected) {
            detectionCount++;
        }
        
        // Display status
        std::cout << "\r";
        if (detected) {
            std::cout << "🔴 OBJECT DETECTED! ";
        } else {
            std::cout << "🟢 Clear           ";
        }
        
        if (measurement.valid) {
            std::cout << "Distance: " << std::fixed << std::setprecision(1) 
                      << measurement.distance << " cm   ";
        } else {
            std::cout << "Distance: --- cm   ";
        }
        
        std::cout << "Checks: " << totalChecks << "   " << std::flush;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    restoreTerminal(oldTermios);
    
    std::cout << "\n\nDetection Summary:\n";
    std::cout << "  Total checks: " << totalChecks << "\n";
    std::cout << "  Detections: " << detectionCount << "\n";
    std::cout << "  Detection rate: " << std::fixed << std::setprecision(1) 
              << (100.0f * detectionCount / std::max(1, totalChecks)) << "%\n";
    
    return true;
}

/**
 * @brief Continuous measurement mode
 */
bool continuousMode(UltrasonicSensor& sensor, const TestConfig& config) {
    std::cout << "\n=== Continuous Measurement Mode ===\n";
    std::cout << "Press 'q' to quit, 's' for statistics, 'r' to reset stats\n\n";
    
    auto oldTermios = setupTerminal();
    
    unsigned long measurementCount = 0;
    unsigned long validCount = 0;
    float minDistance = std::numeric_limits<float>::max();
    float maxDistance = 0.0f;
    
    auto startTime = std::chrono::high_resolution_clock::now();
    auto lastDisplayTime = startTime;
    
    while (g_running) {
        auto measurement = sensor.measure();
        measurementCount++;
        
        if (measurement.valid) {
            validCount++;
            minDistance = std::min(minDistance, measurement.distance);
            maxDistance = std::max(maxDistance, measurement.distance);
        }
        
        // Update display every 100ms
        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastDisplayTime).count() >= 100) {
            std::cout << "\r";
            
            if (measurement.valid) {
                std::cout << "Distance: " << std::fixed << std::setprecision(1) 
                          << std::setw(6) << measurement.distance << " cm   ";
                
                // Alert indicator
                if (measurement.distance <= config.alertDistance) {
                    std::cout << "🔴 ALERT   ";
                } else {
                    std::cout << "          ";
                }
            } else {
                std::cout << "Distance: ---.-- cm            ";
            }
            
            std::cout << "Samples: " << validCount << "/" << measurementCount << "   ";
            
            if (config.verbose && measurement.valid) {
                std::cout << "Echo: " << std::setw(5) << measurement.echoTimeUs << "μs   ";
            }
            
            std::cout << std::flush;
            lastDisplayTime = now;
        }
        
        // Check for keyboard input
        int ch = getchar();
        if (ch != EOF) {
            if (ch == 'q' || ch == 'Q') {
                break;
            } else if (ch == 's' || ch == 'S') {
                std::cout << "\n" << sensor.getStatistics() << std::endl;
                std::cout << "Min/Max distance: " << std::fixed << std::setprecision(1) 
                          << minDistance << " / " << maxDistance << " cm\n";
            } else if (ch == 'r' || ch == 'R') {
                measurementCount = 0;
                validCount = 0;
                minDistance = std::numeric_limits<float>::max();
                maxDistance = 0.0f;
                std::cout << "\nStatistics reset.\n";
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(config.delayMs));
    }
    
    restoreTerminal(oldTermios);
    
    auto endTime = std::chrono::high_resolution_clock::now();
    auto totalTime = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime);
    
    std::cout << "\n\nContinuous Mode Summary:\n";
    std::cout << "  Runtime: " << totalTime.count() << " seconds\n";
    std::cout << "  Total measurements: " << measurementCount << "\n";
    std::cout << "  Valid measurements: " << validCount << "\n";
    std::cout << "  Success rate: " << std::fixed << std::setprecision(1) 
              << (100.0f * validCount / std::max(1UL, measurementCount)) << "%\n";
    
    if (validCount > 0) {
        std::cout << "  Distance range: " << std::fixed << std::setprecision(1) 
                  << minDistance << " - " << maxDistance << " cm\n";
        std::cout << "  Average rate: " << std::fixed << std::setprecision(1) 
                  << (validCount / std::max(static_cast<decltype(totalTime.count())>(1), totalTime.count())) << " Hz\n";
    }
    
    return true;
}

/**
 * @brief Interactive test menu
 */
bool interactiveMode(UltrasonicSensor& sensor, const TestConfig& config) {
    std::cout << "\n=== Interactive Mode ===\n";
    
    while (g_running) {
        std::cout << "\nChoose a test:\n";
        std::cout << "  1. Single measurement\n";
        std::cout << "  2. Batch test (" << config.numSamples << " samples)\n";
        std::cout << "  3. Averaging test\n";
        std::cout << "  4. Object detection\n";
        std::cout << "  5. Continuous mode\n";
        std::cout << "  6. Self-test\n";
        std::cout << "  7. Show statistics\n";
        std::cout << "  8. Show configuration\n";
        std::cout << "  q. Quit\n";
        std::cout << "\nChoice: ";
        
        char choice;
        std::cin >> choice;
        
        switch (choice) {
            case '1': {
                auto measurement = sensor.measure();
                if (measurement.valid) {
                    std::cout << "Distance: " << std::fixed << std::setprecision(2) 
                              << measurement.distance << " cm\n";
                    if (config.verbose) {
                        std::cout << "Raw: " << measurement.rawDistance << " cm, "
                                  << "Echo: " << measurement.echoTimeUs << " μs\n";
                    }
                } else {
                    std::cout << "Measurement failed: " << sensor.getLastError() << std::endl;
                }
                break;
            }
            case '2':
                batchTest(sensor, config);
                break;
            case '3':
                averagingTest(sensor, config);
                break;
            case '4':
                detectionTest(sensor, config);
                break;
            case '5':
                continuousMode(sensor, config);
                break;
            case '6':
                if (sensor.selfTest()) {
                    std::cout << "Self-test PASSED\n";
                } else {
                    std::cout << "Self-test FAILED: " << sensor.getLastError() << std::endl;
                }
                break;
            case '7':
                std::cout << sensor.getStatistics() << std::endl;
                break;
            case '8': {
                auto conf = sensor.getConfig();
                std::cout << "Current Configuration:\n";
                std::cout << "  Trigger pin: " << conf.triggerPin << "\n";
                std::cout << "  Echo pin: " << conf.echoPin << "\n";
                std::cout << "  Range: " << conf.minDistance << " - " << conf.maxDistance << " cm\n";
                std::cout << "  Timeout: " << conf.timeoutUs << " μs\n";
                std::cout << "  Filtering: " << (conf.filteringEnabled ? "enabled" : "disabled") << "\n";
                std::cout << "  Filter window: " << conf.filterWindowSize << " samples\n";
                break;
            }
            case 'q':
            case 'Q':
                return true;
            default:
                std::cout << "Invalid choice. Please try again.\n";
                break;
        }
    }
    
    return true;
}

/**
 * @brief Main function
 */
int main(int argc, char* argv[]) {
    // Setup signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // Parse command line arguments
    TestConfig config = parseArguments(argc, argv);
    
    std::cout << "HC-SR04 Ultrasonic Sensor Test Program\n";
    std::cout << "======================================\n";
    std::cout << "Trigger Pin: " << config.triggerPin << "\n";
    std::cout << "Echo Pin: " << config.echoPin << "\n";
    std::cout << "Alert Distance: " << config.alertDistance << " cm\n";
    
    // Configure sensor
    UltrasonicSensor::SensorConfig sensorConfig;
    sensorConfig.triggerPin = config.triggerPin;
    sensorConfig.echoPin = config.echoPin;
    sensorConfig.filteringEnabled = true;
    sensorConfig.filterWindowSize = 5;
    
    // Create sensor instance
    UltrasonicSensor sensor(sensorConfig);
    
    // Run basic test first
    if (!basicTest(sensor, config)) {
        std::cerr << "Basic test failed. Check hardware connections.\n";
        return 1;
    }
    
    bool success = true;
    
    if (config.interactive) {
        success = interactiveMode(sensor, config);
    } else if (config.continuousMode) {
        success = continuousMode(sensor, config);
    } else {
        // Run default test suite
        success = batchTest(sensor, config) && 
                 averagingTest(sensor, config);
    }
    
    std::cout << "\nTest completed. Final statistics:\n";
    std::cout << sensor.getStatistics() << std::endl;
    
    return success ? 0 : 1;
}
