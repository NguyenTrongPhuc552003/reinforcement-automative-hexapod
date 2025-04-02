#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include <sstream>
#include "application.hpp"

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
              m_updateInterval(0.01f), // 10ms default update interval
              m_frameCount(0),
              m_totalFrameTime(0),
              m_maxFrameTime(0),
              m_performanceMonitoringEnabled(true)
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

                m_controller = std::make_unique<controller::Controller>(*m_hexapod);
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

            registerKeyCommand('m', [this]()
                               {
            std::cout << "Running servo mapping diagnostics..." << std::endl;
            if (m_controller->validateServoMapping()) {
                std::cout << "Servo mapping validation passed!" << std::endl;
            } else {
                std::cout << "Servo mapping validation failed!" << std::endl;
            }
            
            std::cout << m_controller->getServoMappingSummary() << std::endl;
            
            std::cout << "Testing controller connectivity..." << std::endl;
            m_controller->detectControllerIssues();
            
            std::cout << "Testing servo responses..." << std::endl;
            m_controller->testServoConnectivity();
            
            return true; });

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

            // Add balance mode controls
            registerKeyCommand('b', [this]()
                               {
                if (m_controller)
                    m_controller->setBalanceEnabled(!m_controller->isBalanceEnabled());
                return true; });

            registerKeyCommand('[', [this]()
                               {
                if (m_controller) {
                    auto config = m_controller->getBalanceConfig();
                    m_controller->setBalanceResponseFactor(config.response_factor - 0.1);
                }
                return true; });

            registerKeyCommand(']', [this]()
                               {
                if (m_controller) {
                    auto config = m_controller->getBalanceConfig();
                    m_controller->setBalanceResponseFactor(config.response_factor + 0.1);
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
                      << "System Controls:\n"
                      << "  Space: Stop and center\n"
                      << "  +/-: Increase/decrease speed\n"
                      << "  T: Toggle telemetry\n"
                      << "  P: Toggle performance monitoring\n"
                      << "  C: Center all legs\n"
                      << "  M: Run servo mapping diagnostics\n"
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
            [[maybe_unused]] float deltaTime = std::chrono::duration<float>(now - m_lastUpdateTime).count();
            m_lastUpdateTime = now;

            // Update controller state
            return m_controller->update();
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
                controller::ControllerState state = m_controller->getState();
                std::cout << "Controller state: ";
                switch (state)
                {
                case controller::ControllerState::IDLE:
                    std::cout << "IDLE";
                    break;
                case controller::ControllerState::WALKING:
                    std::cout << "WALKING";
                    break;
                case controller::ControllerState::ROTATING:
                    std::cout << "ROTATING";
                    break;
                case controller::ControllerState::TILTING:
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

                // Add balance mode status to telemetry
                if (m_controller)
                {
                    std::cout << "Balance: "
                              << (m_controller->isBalanceEnabled() ? "ENABLED" : "disabled")
                              << " (Response: " << m_controller->getBalanceConfig().response_factor
                              << ", Deadzone: " << m_controller->getBalanceConfig().deadzone
                              << "°)\n";
                }

                std::cout << "\nPress 'h' for help, 'q' to quit, 't' to hide telemetry\n";
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
                m_controller->processKey(' '); // Stop and center legs between mode switches
            }

            // Initialize new mode
            m_currentMode = mode;
            return true;
        }

        // Member variables
        std::string m_lastError;
        ControlMode m_currentMode;

        // Core components
        std::unique_ptr<hexapod::Hexapod> m_hexapod;
        std::unique_ptr<controller::Controller> m_controller;

        // Key command handling
        std::unordered_map<char, KeyCommand> m_keyCommands;

        // Performance monitoring
        std::chrono::high_resolution_clock::time_point m_lastUpdateTime;
        float m_updateInterval; // seconds between updates
        unsigned long m_frameCount;
        unsigned long m_totalFrameTime; // microseconds
        unsigned long m_maxFrameTime;   // microseconds
        bool m_performanceMonitoringEnabled;
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
            pImpl->m_controller->processKey(' '); // Use spacebar command to stop and center legs

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
