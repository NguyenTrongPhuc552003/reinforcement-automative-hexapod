/**
 * @file pwm.cpp
 * @brief PWM testing and logging program for hexapod servos
 *
 * This program provides tools to test, calibrate, and log PWM values for
 * hexapod servos, helping to diagnose issues with movement and positioning.
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <atomic>
#include <chrono>
#include <thread>
#include <csignal>
#include <algorithm>
#include <sstream>

#include "hexapod.hpp"
#include "kinematics.hpp"
#include "logger.hpp"
#include "common.hpp"

// Global running flag for graceful shutdown
static std::atomic<bool> g_running(true);

// Signal handler
void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", exiting...\n";
    g_running.store(false);
}

// Structure to hold PWM frame data
struct PwmFrame {
    uint64_t timestamp_ms;
    std::vector<uint16_t> pwm_values;
    std::vector<bool> enabled;
};

// PWM testing class
class PwmTester {
public:
    PwmTester() : m_hexapod(nullptr), m_initialized(false), 
                 m_captureInterval(100), m_maxFrames(1000), 
                 m_verbose(false), m_logToFile(false) {
        // Initialize logging
        logger::LoggerConfig logConfig;
        logConfig.file_level = logger::LogLevel::DEBUG;
        logConfig.log_file = "pwm_test.log";
        logger::Logger::getInstance().init(logConfig);
        
        LOG_INFO(logger::LogCategory::PWM, "PWM Tester initialized");
    }
    
    ~PwmTester() {
        cleanup();
    }

    // Initialize hexapod hardware
    bool init() {
        m_hexapod = std::make_unique<Hexapod>();
        
        LOG_INFO(logger::LogCategory::PWM, "Initializing hexapod hardware...");
        
        if (!m_hexapod->init()) {
            LOG_ERROR(logger::LogCategory::PWM, "Failed to initialize hexapod: " + 
                      m_hexapod->getLastErrorMessage());
            return false;
        }
        
        LOG_INFO(logger::LogCategory::PWM, "Hexapod hardware initialized successfully");
        m_initialized = true;
        return true;
    }
    
    // Clean up resources
    void cleanup() {
        if (m_initialized && m_hexapod) {
            LOG_INFO(logger::LogCategory::PWM, "Centering all legs for safety");
            m_hexapod->centerAll();
            m_hexapod->cleanup();
        }
        LOG_INFO(logger::LogCategory::PWM, "Resources cleaned up");
    }
    
    // Start PWM frame capture
    bool startCapture(int intervalMs, bool toFile = false) {
        if (!m_initialized) {
            LOG_ERROR(logger::LogCategory::PWM, "Cannot start capture - system not initialized");
            return false;
        }
        
        m_captureInterval = intervalMs;
        m_logToFile = toFile;
        m_frames.clear();
        
        // Reserve space to avoid reallocations
        m_frames.reserve(m_maxFrames);
        
        LOG_INFO(logger::LogCategory::PWM, "Starting PWM capture with interval of " + 
                 std::to_string(m_captureInterval) + "ms");
        
        return true;
    }
    
    // Stop PWM frame capture
    void stopCapture() {
        LOG_INFO(logger::LogCategory::PWM, "Stopping PWM capture. Captured " + 
                 std::to_string(m_frames.size()) + " frames");
                 
        if (m_logToFile && !m_frames.empty()) {
            saveCaptureToCsv();
        }
    }
    
    // Capture a single PWM frame
    bool capturePwmFrame() {
        if (!m_initialized) {
            return false;
        }
        
        PwmFrame frame;
        frame.timestamp_ms = getCurrentTimeMillis();
        frame.pwm_values.resize(TOTAL_SERVOS, 0);
        frame.enabled.resize(TOTAL_SERVOS, false);
        
        // For a real implementation, we would read actual PWM values here
        // As we don't have direct PWM reading capability, we'll use the leg positions
        // to demonstrate the concept
        
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            LegPosition position;
            if (m_hexapod->getLegPosition(leg, position)) {
                // Calculate theoretical PWM values for joints
                // This is a simplified model - real values would come from driver
                int hip_pwm = convertAngleToPwm(position.getHip());
                int knee_pwm = convertAngleToPwm(position.getKnee());
                int ankle_pwm = convertAngleToPwm(position.getAnkle());
                
                int baseIndex = leg * SERVOS_PER_LEG;
                frame.pwm_values[baseIndex] = hip_pwm;
                frame.pwm_values[baseIndex + 1] = knee_pwm;
                frame.pwm_values[baseIndex + 2] = ankle_pwm;
                
                frame.enabled[baseIndex] = true;
                frame.enabled[baseIndex + 1] = true;
                frame.enabled[baseIndex + 2] = true;
            }
        }
        
        // Add frame to collection if we're within limit
        if (m_frames.size() < m_maxFrames) {
            m_frames.push_back(frame);
        } else {
            // Remove oldest frame to make room
            m_frames.erase(m_frames.begin());
            m_frames.push_back(frame);
        }
        
        if (m_verbose) {
            printFrameInfo(frame);
        }
        
        return true;
    }
    
    // Convert joint angle to PWM value (simplified model)
    uint16_t convertAngleToPwm(int16_t angle) {
        // Simplified conversion model - assumes:
        // -90 degrees = 1000 µs
        // 0 degrees = 1500 µs
        // +90 degrees = 2000 µs
        return static_cast<uint16_t>(1500 + angle * (1000.0 / 180.0));
    }
    
    // Print information about a PWM frame
    void printFrameInfo(const PwmFrame& frame) {
        std::cout << "Frame timestamp: " << frame.timestamp_ms << " ms\n";
        
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            std::cout << "Leg " << leg << ": ";
            
            int baseIndex = leg * SERVOS_PER_LEG;
            
            std::cout << "Hip=" << std::setw(4) << frame.pwm_values[baseIndex]
                      << "µs, Knee=" << std::setw(4) << frame.pwm_values[baseIndex + 1]
                      << "µs, Ankle=" << std::setw(4) << frame.pwm_values[baseIndex + 2]
                      << "µs\n";
        }
        std::cout << std::endl;
    }
    
    // Save captured PWM data to CSV file
    bool saveCaptureToCsv(const std::string& filename = "") {
        if (m_frames.empty()) {
            LOG_WARNING(logger::LogCategory::PWM, "No frames to save");
            return false;
        }
        
        std::string outputFile = filename;
        if (outputFile.empty()) {
            // Generate timestamped filename
            auto now = std::chrono::system_clock::now();
            auto now_time_t = std::chrono::system_clock::to_time_t(now);
            std::tm now_tm = *std::localtime(&now_time_t);
            
            std::stringstream ss;
            ss << "pwm_capture_"
               << std::put_time(&now_tm, "%Y%m%d_%H%M%S")
               << ".csv";
            outputFile = ss.str();
        }
        
        std::ofstream file(outputFile);
        if (!file.is_open()) {
            LOG_ERROR(logger::LogCategory::PWM, "Failed to open file for writing: " + outputFile);
            return false;
        }
        
        LOG_INFO(logger::LogCategory::PWM, "Saving " + std::to_string(m_frames.size()) + 
                 " frames to " + outputFile);
        
        // Write header
        file << "Timestamp";
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            for (int joint = 0; joint < SERVOS_PER_LEG; joint++) {
                std::string jointName;
                switch (joint) {
                    case 0: jointName = "Hip"; break;
                    case 1: jointName = "Knee"; break;
                    case 2: jointName = "Ankle"; break;
                    default: jointName = "Unknown"; break;
                }
                file << ",Leg" << leg << "_" << jointName << "_PWM";
                file << ",Leg" << leg << "_" << jointName << "_Enabled";
            }
        }
        file << std::endl;
        
        // Write data rows
        for (const auto& frame : m_frames) {
            file << frame.timestamp_ms;
            
            for (int i = 0; i < TOTAL_SERVOS; i++) {
                file << "," << frame.pwm_values[i];
                file << "," << (frame.enabled[i] ? "1" : "0");
            }
            file << std::endl;
        }
        
        file.close();
        LOG_INFO(logger::LogCategory::PWM, "PWM data saved successfully to " + outputFile);
        return true;
    }
    
    // Run a continuous capture session
    void runContinuousCapture() {
        if (!startCapture(100, true)) {
            return;
        }
        
        uint64_t lastCaptureTime = 0;
        
        std::cout << "Starting continuous PWM capture. Press Ctrl+C to stop.\n";
        
        while (g_running.load()) {
            uint64_t currentTime = getCurrentTimeMillis();
            
            if (currentTime - lastCaptureTime >= m_captureInterval) {
                capturePwmFrame();
                lastCaptureTime = currentTime;
            }
            
            // Sleep a short time to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        stopCapture();
    }
    
    // Run a movement test to capture PWM during motion
    void runMovementTest() {
        if (!m_initialized) {
            LOG_ERROR(logger::LogCategory::PWM, "Cannot run movement test - system not initialized");
            return;
        }
        
        LOG_INFO(logger::LogCategory::PWM, "Starting movement test with PWM capture");
        
        if (!startCapture(50, true)) {
            return;
        }

        // First center all legs
        std::cout << "Centering all legs...\n";
        m_hexapod->centerAll();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // Test each leg individually
        for (int leg = 0; leg < NUM_LEGS && g_running.load(); leg++) {
            std::cout << "Testing leg " << leg << "...\n";
            
            // Test different positions
            LegPosition pos1(20, -30, 15);   // Forward position
            LegPosition pos2(-20, -30, 15);  // Backward position
            LegPosition pos3(0, -20, 25);    // Raised position
            
            // Move to different positions and capture PWM data
            m_hexapod->setLegPosition(leg, pos1);
            captureForDuration(500);
            
            if (!g_running.load()) break;
            
            m_hexapod->setLegPosition(leg, pos2);
            captureForDuration(500);
            
            if (!g_running.load()) break;
            
            m_hexapod->setLegPosition(leg, pos3);
            captureForDuration(500);
            
            if (!g_running.load()) break;
            
            // Return to center position
            LegPosition center(0, 0, 0);
            m_hexapod->setLegPosition(leg, center);
            captureForDuration(500);
        }
        
        // Return to home position
        m_hexapod->centerAll();
        captureForDuration(1000);
        
        stopCapture();
        LOG_INFO(logger::LogCategory::PWM, "Movement test completed");
    }
    
    // Capture PWM frames for a specified duration
    void captureForDuration(uint64_t durationMs) {
        uint64_t startTime = getCurrentTimeMillis();
        uint64_t lastCaptureTime = startTime;
        
        while (g_running.load() && (getCurrentTimeMillis() - startTime < durationMs)) {
            uint64_t currentTime = getCurrentTimeMillis();
            
            if (currentTime - lastCaptureTime >= m_captureInterval) {
                capturePwmFrame();
                lastCaptureTime = currentTime;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    // Test a specific joint across its range
    void testJointRange(int legIndex, int jointIndex, int minAngle, int maxAngle, int step) {
        if (!m_initialized || legIndex >= NUM_LEGS || jointIndex >= SERVOS_PER_LEG) {
            LOG_ERROR(logger::LogCategory::PWM, "Invalid leg or joint index");
            return;
        }
        
        LOG_INFO(logger::LogCategory::PWM, "Testing range for leg " + std::to_string(legIndex) + 
                 " joint " + std::to_string(jointIndex));
        
        startCapture(50, true);
        
        // Center all legs
        m_hexapod->centerAll();
        captureForDuration(500);
        
        // Create a position with default values
        LegPosition position;
        position.leg_num = legIndex;
        
        // Test the joint through its range
        std::cout << "Joint | Angle | PWM Value\n";
        std::cout << "---------------------\n";
        
        for (int angle = minAngle; angle <= maxAngle && g_running.load(); angle += step) {
            // Set the specified joint to the test angle
            switch (jointIndex) {
                case 0: // Hip
                    position.setHip(angle);
                    break;
                case 1: // Knee
                    position.setKnee(angle);
                    break;
                case 2: // Ankle
                    position.setAnkle(angle);
                    break;
            }
            
            // Apply the position
            m_hexapod->setLegPosition(legIndex, position);
            
            // Wait a bit for the servo to move
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Capture PWM frame
            capturePwmFrame();
            
            // Display joint angle and corresponding PWM
            std::string jointName;
            switch (jointIndex) {
                case 0: jointName = "Hip"; break;
                case 1: jointName = "Knee"; break;
                case 2: jointName = "Ankle"; break;
            }
            
            int servoIndex = legIndex * SERVOS_PER_LEG + jointIndex;
            if (!m_frames.empty()) {
                std::cout << std::left << std::setw(6) << jointName << "| " 
                          << std::setw(5) << angle << " | " 
                          << m_frames.back().pwm_values[servoIndex] << " µs\n";
            }
            
            // Check if we should stop
            if (!g_running.load()) break;
        }
        
        // Return to center position
        m_hexapod->centerAll();
        captureForDuration(500);
        
        stopCapture();
    }
    
    // Set verbosity level
    void setVerbose(bool verbose) {
        m_verbose = verbose;
    }
    
    // Set maximum frames to store
    void setMaxFrames(int maxFrames) {
        if (maxFrames > 0) {
            m_maxFrames = maxFrames;
        }
    }
    
    // Get current time in milliseconds
    uint64_t getCurrentTimeMillis() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
    }

private:
    std::unique_ptr<Hexapod> m_hexapod;
    bool m_initialized;
    uint64_t m_captureInterval;  // ms
    size_t m_maxFrames;
    bool m_verbose;
    bool m_logToFile;
    std::vector<PwmFrame> m_frames;
};

// Print program usage
void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [options]\n"
              << "Options:\n"
              << "  -m, --movement     Run movement test with PWM logging\n"
              << "  -c, --capture      Run continuous PWM capture\n"
              << "  -j, --joint ARGS   Test specific joint range (leg joint min max step)\n"
              << "  -v, --verbose      Enable verbose output\n"
              << "  -h, --help         Display this help message\n\n"
              << "Example:\n"
              << "  " << programName << " --joint 0 0 -45 45 15   # Test leg 0, hip joint from -45 to 45 in steps of 15\n";
}

// Parse joint test arguments
bool parseJointTestArgs(int argc, char** argv, int argIndex, int& leg, int& joint, int& min, int& max, int& step) {
    if (argIndex + 5 > argc) {
        std::cerr << "Error: Not enough arguments for joint test\n";
        return false;
    }
    
    leg = std::stoi(argv[argIndex]);
    joint = std::stoi(argv[argIndex + 1]);
    min = std::stoi(argv[argIndex + 2]);
    max = std::stoi(argv[argIndex + 3]);
    step = std::stoi(argv[argIndex + 4]);
    
    if (leg < 0 || leg >= NUM_LEGS) {
        std::cerr << "Error: Leg index must be between 0 and " << (NUM_LEGS - 1) << "\n";
        return false;
    }
    
    if (joint < 0 || joint >= SERVOS_PER_LEG) {
        std::cerr << "Error: Joint index must be between 0 and " << (SERVOS_PER_LEG - 1) << "\n";
        return false;
    }
    
    if (min > max) {
        std::cerr << "Error: Minimum angle must be less than maximum angle\n";
        return false;
    }
    
    if (step <= 0) {
        std::cerr << "Error: Step must be greater than 0\n";
        return false;
    }
    
    return true;
}

// Main function
int main(int argc, char** argv) {
    // Set up signal handler
    std::signal(SIGINT, signalHandler);
    
    // Default parameters
    bool runMovementTest = false;
    bool runCapture = false;
    bool runJointTest = false;
    bool verbose = false;
    int testLeg = 0, testJoint = 0, minAngle = -45, maxAngle = 45, angleStep = 15;
    
    // Parse command line arguments
    if (argc > 1) {
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            
            if (arg == "-h" || arg == "--help") {
                printUsage(argv[0]);
                return 0;
            } else if (arg == "-m" || arg == "--movement") {
                runMovementTest = true;
            } else if (arg == "-c" || arg == "--capture") {
                runCapture = true;
            } else if (arg == "-j" || arg == "--joint") {
                runJointTest = true;
                if (!parseJointTestArgs(argc, argv, i + 1, testLeg, testJoint, minAngle, maxAngle, angleStep)) {
                    return 1;
                }
                i += 5; // Skip the joint test arguments
            } else if (arg == "-v" || arg == "--verbose") {
                verbose = true;
            } else {
                std::cerr << "Unknown option: " << arg << "\n";
                printUsage(argv[0]);
                return 1;
            }
        }
    }
    
    // If no action specified, show usage
    if (!runMovementTest && !runCapture && !runJointTest) {
        printUsage(argv[0]);
        return 0;
    }
    
    // Initialize the PWM tester
    PwmTester tester;
    tester.setVerbose(verbose);
    
    // Initialize hardware
    if (!tester.init()) {
        std::cerr << "Failed to initialize hardware. Exiting.\n";
        return 1;
    }
    
    // Run the requested test mode
    if (runMovementTest) {
        tester.runMovementTest();
    } else if (runCapture) {
        tester.runContinuousCapture();
    } else if (runJointTest) {
        tester.testJointRange(testLeg, testJoint, minAngle, maxAngle, angleStep);
    }
    
    std::cout << "PWM testing complete.\n";
    return 0;
}
