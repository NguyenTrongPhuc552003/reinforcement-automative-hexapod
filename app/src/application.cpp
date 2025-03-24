#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include "application.hpp"

// Use atomic for thread safety
std::atomic<bool> Application::m_running(false);
std::atomic<bool> Application::m_telemetryActive(false);

// Singleton implementation
Application &Application::getInstance()
{
    static Application instance;
    return instance;
}

Application::Application()
    : m_currentMode(ControlMode::MANUAL),
      m_hexapod(nullptr),
      m_controller(nullptr),
      m_updateInterval(0.01f), // 10ms default update interval
      m_frameCount(0),
      m_totalFrameTime(0),
      m_maxFrameTime(0),
      m_performanceMonitoringEnabled(true)
{
    // Initialize performance clock
    m_lastUpdateTime = std::chrono::high_resolution_clock::now();

    // Load configuration if available
    loadConfiguration();
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
    if (!initializeHexapod())
    {
        std::cerr << "Failed to initialize hexapod: " << m_lastError << std::endl;
        return false;
    }

    if (!initializeController())
    {
        std::cerr << "Failed to initialize controller: " << m_lastError << std::endl;
        return false;
    }

    if (!setupInputHandling())
    {
        std::cerr << "Failed to set up input handling: " << m_lastError << std::endl;
        return false;
    }

    // Setup key command mappings with extended functionality
    setupKeyCommands();

    std::cout << "Application initialized successfully" << std::endl;
    return true;
}

void Application::setupKeyCommands()
{
    // Basic movement controls
    registerKeyCommand('w', [this]()
                       { return m_controller->processKey('w'); });
    registerKeyCommand('s', [this]()
                       { return m_controller->processKey('s'); });
    registerKeyCommand('a', [this]()
                       { return m_controller->processKey('a'); });
    registerKeyCommand('d', [this]()
                       { return m_controller->processKey('d'); });

    // Height and tilt controls
    registerKeyCommand('i', [this]()
                       { return m_controller->processKey('i'); });
    registerKeyCommand('k', [this]()
                       { return m_controller->processKey('k'); });
    registerKeyCommand('j', [this]()
                       { return m_controller->processKey('j'); });
    registerKeyCommand('l', [this]()
                       { return m_controller->processKey('l'); });

    // Gait controls
    registerKeyCommand('1', [this]()
                       { return m_controller->processKey('1'); });
    registerKeyCommand('2', [this]()
                       { return m_controller->processKey('2'); });
    registerKeyCommand('3', [this]()
                       { return m_controller->processKey('3'); });

    // Speed controls
    registerKeyCommand('+', [this]()
                       { return m_controller->processKey('+'); });
    registerKeyCommand('-', [this]()
                       { return m_controller->processKey('-'); });

    // System controls
    registerKeyCommand(' ', [this]()
                       { return m_controller->processKey(' '); });
    registerKeyCommand('q', [this]()
                       { m_running = false; return true; });

    // Additional function keys
    registerKeyCommand('t', [this]()
                       { 
        m_telemetryActive = !m_telemetryActive; 
        std::cout << "Telemetry " << (m_telemetryActive ? "enabled" : "disabled") << std::endl;
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
                  << (m_performanceMonitoringEnabled ? "enabled" : "disabled") << std::endl;
        return true; });
}

void Application::printHelp() const
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
              << "System Controls:\n"
              << "  Space: Stop and center\n"
              << "  +/-: Increase/decrease speed\n"
              << "  T: Toggle telemetry\n"
              << "  P: Toggle performance monitoring\n"
              << "  C: Center all legs\n"
              << "  H: Show this help\n"
              << "  Q: Quit\n"
              << std::endl;
}

bool Application::initializeHexapod()
{
    try
    {
        std::cout << "Initializing hexapod hardware interface..." << std::endl;
        m_hexapod = std::make_unique<Hexapod>();

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

bool Application::initializeController()
{
    try
    {
        if (!m_hexapod)
        {
            m_lastError = "Cannot initialize controller: Hexapod not initialized";
            return false;
        }

        m_controller = std::make_unique<Controller>(*m_hexapod);
        if (!m_controller->init())
        {
            m_lastError = "Failed to initialize controller";
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

bool Application::setupInputHandling()
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

void Application::registerKeyCommand(char key, KeyCommand command)
{
    m_keyCommands[key] = command;
}

bool Application::executeKeyCommand(char key)
{
    auto it = m_keyCommands.find(key);
    if (it != m_keyCommands.end())
    {
        return it->second();
    }
    return true; // Unknown key, not an error
}

bool Application::processInput()
{
    char key;
    if (read(STDIN_FILENO, &key, 1) > 0)
    {
        return executeKeyCommand(key);
    }
    return true;
}

bool Application::update()
{
    // Calculate time delta using high-resolution clock
    auto now = std::chrono::high_resolution_clock::now();
    [[maybe_unused]] float deltaTime = std::chrono::duration<float>(now - m_lastUpdateTime).count();
    m_lastUpdateTime = now;

    // Update controller
    return m_controller->update();
}

Application::Result Application::run()
{
    // Print instructions and controls
    std::cout << "\nHexapod Controller Started" << std::endl;
    printHelp();

    // Reset timer for accurate performance tracking
    m_lastUpdateTime = std::chrono::high_resolution_clock::now();
    auto lastTelemetryTime = m_lastUpdateTime;
    auto lastPerformanceReportTime = m_lastUpdateTime;

    // Main control loop
    m_running = true;
    m_frameCount = 0;
    m_totalFrameTime = 0;
    m_maxFrameTime = 0;

    std::chrono::high_resolution_clock::time_point frameStart;
    std::chrono::microseconds frameTime;
    const std::chrono::microseconds targetFrameTime(static_cast<int>(m_updateInterval * 1000000));

    // Initial leg centering for safety
    if (!m_hexapod->centerAll())
    {
        std::cerr << "Warning: Failed to center legs at startup" << std::endl;
    }

    std::cout << "Ready. Use 'h' for help, 'q' to quit." << std::endl;

    while (m_running)
    {
        frameStart = std::chrono::high_resolution_clock::now();

        // Process input (non-blocking)
        if (!processInput())
        {
            m_lastError = "Input processing error";
            return Result::ERROR_RUNTIME;
        }

        // Update hexapod state
        if (!update())
        {
            m_lastError = "Update error";
            return Result::ERROR_RUNTIME;
        }

        // Calculate frame time and update stats
        auto now = std::chrono::high_resolution_clock::now();
        frameTime = std::chrono::duration_cast<std::chrono::microseconds>(now - frameStart);

        // Update performance metrics
        updatePerformanceMetrics(frameTime);

        // Display telemetry periodically
        if (m_telemetryActive &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTelemetryTime).count() >= 500)
        {
            displayTelemetry();
            lastTelemetryTime = now;
        }

        // Report performance metrics periodically
        if (m_performanceMonitoringEnabled &&
            std::chrono::duration_cast<std::chrono::seconds>(now - lastPerformanceReportTime).count() >= 5)
        {
            reportPerformance();
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

    return Result::TERMINATED_BY_USER;
}

void Application::updatePerformanceMetrics(const std::chrono::microseconds &frameTime)
{
    m_frameCount++;
    m_totalFrameTime += frameTime.count();
    m_maxFrameTime = std::max(m_maxFrameTime, (unsigned long)frameTime.count());
}

void Application::displayTelemetry()
{
    // Get current hexapod state
    if (!m_hexapod || !m_controller)
        return;

    ImuData imuData;
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
        ControllerState state = m_controller->getState();
        std::cout << "Controller state: ";
        switch (state)
        {
        case ControllerState::IDLE:
            std::cout << "IDLE";
            break;
        case ControllerState::WALKING:
            std::cout << "WALKING";
            break;
        case ControllerState::ROTATING:
            std::cout << "ROTATING";
            break;
        case ControllerState::TILTING:
            std::cout << "TILTING";
            break;
        default:
            std::cout << "UNKNOWN";
            break;
        }
        std::cout << "\n";

        // Show current system status
        double avgFrameTime = m_frameCount > 0 ? (m_totalFrameTime / m_frameCount) / 1000.0 : 0;
        std::cout << "System: " << std::fixed << std::setprecision(2)
                  << "Avg frame: " << avgFrameTime << "ms, "
                  << "Max frame: " << (m_maxFrameTime / 1000.0) << "ms, "
                  << "FPS: " << (1000.0 / std::max(1.0, avgFrameTime)) << "\n";

        std::cout << "\nPress 'h' for help, 'q' to quit, 't' to hide telemetry\n";
    }
}

void Application::reportPerformance()
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

void Application::shutdown()
{
    std::cout << "\nShutting down application..." << std::endl;

    // Center legs for safety before shutdown
    if (m_controller)
    {
        std::cout << "Stopping movement and centering legs..." << std::endl;
        m_controller->processKey(' '); // Use spacebar command to stop and center legs

        // Give some time for the command to complete
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Save configuration if needed
    saveConfiguration();

    // Clean up resources
    std::cout << "Releasing resources..." << std::endl;
    m_controller.reset();
    m_hexapod.reset();

    std::cout << "Shutdown complete" << std::endl;
}

bool Application::loadConfiguration()
{
    // Implementation would load from a config file
    // For now, just use default values
    m_updateInterval = 0.01f; // 10ms default
    return true;
}

bool Application::saveConfiguration()
{
    // Implementation would save to a config file
    return true;
}

bool Application::switchMode(ControlMode mode)
{
    // Implementation for switching between control modes
    if (m_currentMode == mode)
    {
        return true; // Already in this mode
    }

    // Cleanup current mode - ensure clean transition
    if (m_controller)
    {
        m_controller->processKey(' '); // Stop and center legs between mode switches
    }

    // Initialize new mode
    m_currentMode = mode;

    return true;
}

Application::ControlMode Application::getCurrentMode() const
{
    return m_currentMode;
}

std::string Application::getLastErrorMessage() const
{
    return m_lastError;
}
