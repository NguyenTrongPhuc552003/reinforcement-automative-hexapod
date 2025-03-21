#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include "application.hpp"

volatile bool Application::m_running = false;

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
      m_updateInterval(0.01f) // 10ms default update interval
{
    // Initialize performance clock
    m_lastUpdateTime = std::chrono::high_resolution_clock::now();
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
    // Register signal handler
    signal(SIGINT, Application::signalHandler);

    // Initialize components with more efficient error handling
    if (!initializeHexapod() || !initializeController() || !setupInputHandling())
    {
        return false; // m_lastError already set by the specific function
    }

    // Setup key commands - more efficient with initializer list
    const std::pair<char, KeyCommand> keyMappings[] = {
        {'w', [this]()
         { return m_controller->processKey('w'); }},
        {'s', [this]()
         { return m_controller->processKey('s'); }},
        {'a', [this]()
         { return m_controller->processKey('a'); }},
        {'d', [this]()
         { return m_controller->processKey('d'); }},
        {'i', [this]()
         { return m_controller->processKey('i'); }},
        {'k', [this]()
         { return m_controller->processKey('k'); }},
        {'j', [this]()
         { return m_controller->processKey('j'); }},
        {'l', [this]()
         { return m_controller->processKey('l'); }},
        {' ', [this]()
         { return m_controller->processKey(' '); }},
        {'1', [this]()
         { return m_controller->processKey('1'); }}, // Added for gait switching
        {'2', [this]()
         { return m_controller->processKey('2'); }},
        {'3', [this]()
         { return m_controller->processKey('3'); }},
        {'+', [this]()
         { return m_controller->processKey('+'); }}, // Speed control
        {'-', [this]()
         { return m_controller->processKey('-'); }}};

    for (const auto &mapping : keyMappings)
    {
        registerKeyCommand(mapping.first, mapping.second);
    }

    return true;
}

bool Application::initializeHexapod()
{
    try
    {
        m_hexapod = std::make_unique<Hexapod>();
        if (!m_hexapod->init())
        {
            m_lastError = "Failed to initialize hexapod: " + m_hexapod->getLastErrorMessage();
            return false;
        }
        return true;
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
    // Print instructions
    std::cout << "Hexapod Controller" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  W/S: Forward/Backward" << std::endl;
    std::cout << "  A/D: Rotate Left/Right" << std::endl;
    std::cout << "  I/K: Raise/Lower" << std::endl;
    std::cout << "  J/L: Tilt Left/Right" << std::endl;
    std::cout << "  1/2/3: Change gait pattern (Tripod/Wave/Ripple)" << std::endl;
    std::cout << "  +/-: Increase/decrease speed" << std::endl;
    std::cout << "  Space: Stop" << std::endl;
    std::cout << "  Ctrl+C: Exit" << std::endl
              << std::endl;

    // Reset timer for accurate performance tracking
    m_lastUpdateTime = std::chrono::high_resolution_clock::now();

    // Improved main loop with optimized timing
    m_running = true;
    std::chrono::high_resolution_clock::time_point frameStart;
    std::chrono::microseconds frameTime, sleepTime;

    const std::chrono::microseconds targetFrameTime(static_cast<int>(m_updateInterval * 1000000));

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

        // Calculate how long this frame took to process
        frameTime = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - frameStart);

        // Sleep for the remaining time to maintain target frame rate
        if (frameTime < targetFrameTime)
        {
            sleepTime = targetFrameTime - frameTime;
            usleep(sleepTime.count());
        }
    }

    return Result::TERMINATED_BY_USER;
}

void Application::shutdown()
{
    // Center legs for safety before shutdown
    if (m_controller)
    {
        m_controller->processKey(' '); // Use spacebar command to stop and center legs
    }

    // Clean up resources
    m_controller.reset();
    m_hexapod.reset();

    std::cout << "\nShutdown complete" << std::endl;
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
