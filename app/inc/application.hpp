#ifndef APPLICATION_HPP
#define APPLICATION_HPP

#include <memory>
#include <unordered_map>
#include <functional>
#include <string>
#include <chrono>
#include <atomic>
#include "hexapod.hpp"
#include "controller.hpp"

/**
 * Main application class that encapsulates the hexapod control logic
 */
class Application
{
public:
    // Singleton pattern
    static Application &getInstance();

    // Delete copy/move constructors and assignment operators
    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;
    Application(Application &&) = delete;
    Application &operator=(Application &&) = delete;

    // Application control modes
    enum class ControlMode
    {
        MANUAL,     // Direct keyboard control
        AUTONOMOUS, // Self-navigating mode
        SEQUENCE,   // Run pre-programmed sequence
        CALIBRATION // Calibration mode
    };

    // Result of operations
    enum class Result
    {
        SUCCESS,
        ERROR_INITIALIZATION,
        ERROR_RUNTIME,
        ERROR_SHUTDOWN,
        TERMINATED_BY_USER
    };

    // Lifecycle methods
    bool init();
    Result run();
    void shutdown();

    // Control methods
    bool switchMode(ControlMode mode);
    ControlMode getCurrentMode() const;
    std::string getLastErrorMessage() const;

    // Signal handling
    static void signalHandler(int signal);

private:
    Application();
    ~Application();

    // Private implementation methods
    bool initializeHexapod();
    bool initializeController();
    bool setupInputHandling();
    bool processInput();
    bool update();
    void setupKeyCommands();
    void printHelp() const;
    void displayTelemetry();
    void updatePerformanceMetrics(const std::chrono::microseconds &frameTime);
    void reportPerformance();
    bool loadConfiguration();
    bool saveConfiguration();

    // Command pattern for key handling
    using KeyCommand = std::function<bool()>;
    void registerKeyCommand(char key, KeyCommand command);
    bool executeKeyCommand(char key);

    // Member variables
    static std::atomic<bool> m_running;
    static std::atomic<bool> m_telemetryActive;
    std::string m_lastError;
    ControlMode m_currentMode;

    // Smart pointers for resource management
    std::unique_ptr<Hexapod> m_hexapod;
    std::unique_ptr<Controller> m_controller;

    // Key command map
    std::unordered_map<char, KeyCommand> m_keyCommands;

    // Performance monitoring
    std::chrono::high_resolution_clock::time_point m_lastUpdateTime;
    float m_updateInterval; // seconds between updates
    unsigned long m_frameCount;
    unsigned long m_totalFrameTime; // microseconds
    unsigned long m_maxFrameTime;   // microseconds
    bool m_performanceMonitoringEnabled;
};

#endif // APPLICATION_HPP
