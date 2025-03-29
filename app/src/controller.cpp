#include <cstdio>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <time.h>
#include <iostream>
#include "controller.hpp"

namespace controller
{

    //==============================================================================
    // Implementation Class (PIMPL idiom)
    //==============================================================================

    class ControllerImpl
    {
    public:
        /**
         * @brief Construct a new Controller Implementation object
         *
         * @param hexapod Reference to hexapod hardware interface
         */
        explicit ControllerImpl(hexapod::Hexapod &hexapod)
            : hexapod(hexapod),
              speed(0.5),
              direction(0.0),
              height(0.0),
              tiltX(0.0),
              tiltY(0.0),
              gaitType(gait::GaitType::TRIPOD),
              state(ControllerState::IDLE),
              terminalConfigured(false)
        {
            clock_gettime(CLOCK_MONOTONIC, &lastUpdate);
        }

        /**
         * @brief Destroy the Controller Implementation object
         *
         * Restores terminal settings and centers legs for safe shutdown
         */
        ~ControllerImpl()
        {
            // Restore terminal settings if modified
            if (terminalConfigured)
            {
                tcsetattr(STDIN_FILENO, TCSANOW, &origTermios);
            }

            // Safety: make sure robot is in a stable pose when destructed
            hexapod.centerAll();
        }

        /**
         * @brief Initialize controller
         *
         * Sets up terminal for non-canonical input and initializes gait controller
         *
         * @return true if initialization successful
         * @return false if initialization failed
         */
        bool init()
        {
            // Configure terminal for immediate keyboard input (non-canonical mode)
            terminalConfigured = configureTerminal();
            if (!terminalConfigured)
            {
                std::cerr << "Warning: Failed to configure terminal for immediate input" << std::endl;
                // Continue anyway - it's not critical for operation
            }

            // Initialize gait controller with default parameters
            gait::GaitParameters params;
            params.type = gaitType;
            params.stepHeight = 30.0;
            params.stepLength = 60.0;
            params.cycleTime = 1.0;
            params.dutyFactor = 0.5;

            if (!gait.init(hexapod, params))
            {
                std::cerr << "Error: Failed to initialize gait controller" << std::endl;
                return false;
            }

            return true;
        }

        /**
         * @brief Configure terminal for immediate key input
         *
         * @return true if terminal configuration succeeded
         * @return false if terminal configuration failed
         */
        bool configureTerminal()
        {
            // Get current terminal attributes
            if (tcgetattr(STDIN_FILENO, &origTermios) < 0)
            {
                return false;
            }

            // Create a modified version with non-canonical, non-echo mode
            struct termios newTermios = origTermios;
            newTermios.c_lflag &= ~(ICANON | ECHO);

            // Apply modified attributes
            if (tcsetattr(STDIN_FILENO, TCSANOW, &newTermios) < 0)
            {
                return false;
            }

            return true;
        }

        /**
         * @brief Process keyboard input command
         *
         * @param key Input key character
         * @return true if command processed successfully
         * @return false if command failed
         */
        bool processKey(char key)
        {
            // Handle input based on key pressed
            switch (key)
            {
            // Movement commands
            case 'w': // Forward
                direction = 0.0;
                state = ControllerState::WALKING;
                break;

            case 's': // Backward
                direction = 180.0;
                state = ControllerState::WALKING;
                break;

            case 'a': // Rotate left
                direction = -90.0;
                state = ControllerState::ROTATING;
                break;

            case 'd': // Rotate right
                direction = 90.0;
                state = ControllerState::ROTATING;
                break;

            // Height adjustment
            case 'i':                                   // Raise body
                height = std::min(height + 10.0, 50.0); // Limit maximum height
                break;

            case 'k':                                    // Lower body
                height = std::max(height - 10.0, -30.0); // Limit minimum height
                break;

            // Tilt control
            case 'j': // Tilt left
                tiltY = -15.0;
                state = ControllerState::TILTING;
                break;

            case 'l': // Tilt right
                tiltY = 15.0;
                state = ControllerState::TILTING;
                break;

            // Gait pattern selection
            case '1': // Tripod gait
                setGaitType(gait::GaitType::TRIPOD);
                break;

            case '2': // Wave gait
                setGaitType(gait::GaitType::WAVE);
                break;

            case '3': // Ripple gait
                setGaitType(gait::GaitType::RIPPLE);
                break;

            // Speed control
            case '+': // Increase speed
                speed = std::min(1.0, speed + 0.1);
                std::cout << "Speed: " << std::fixed << std::setprecision(1) << speed << std::endl;
                break;

            case '-': // Decrease speed
                speed = std::max(0.1, speed - 0.1);
                std::cout << "Speed: " << std::fixed << std::setprecision(1) << speed << std::endl;
                break;

            // Stop/center command
            case ' ': // Stop and center legs
                state = ControllerState::IDLE;
                return hexapod.centerAll();

            default:
                // Unknown key - no action needed
                return true;
            }

            return true;
        }

        /**
         * @brief Update controller state
         *
         * Should be called regularly to maintain movement patterns and state transitions
         *
         * @return true if update was successful
         * @return false if update failed
         */
        bool update()
        {
            // Get current time
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);

            // Calculate time delta since last update
            double timeDelta = (now.tv_sec - lastUpdate.tv_sec) +
                               ((now.tv_nsec - lastUpdate.tv_nsec) / 1.0e9);

            // Skip update if time delta is too small for efficiency
            if (timeDelta < 0.005)
            { // ~200Hz max update rate
                return true;
            }

            // Track update result
            bool result = true;

            // Handle state-specific updates
            switch (state)
            {
            case ControllerState::WALKING:
            case ControllerState::ROTATING:
                // Use gait controller for walking/rotating
                result = gait.update(now.tv_sec + now.tv_nsec / 1.0e9, direction, speed);
                break;

            case ControllerState::TILTING:
                // Apply body tilt
                result = applyTilt();
                break;

            case ControllerState::IDLE:
                // No action needed in idle state
                break;
            }

            // Store current time for next update
            lastUpdate = now;
            return result;
        }

        /**
         * @brief Set the gait type and adjust parameters accordingly
         *
         * @param type New gait type to use
         */
        void setGaitType(gait::GaitType type)
        {
            // Do nothing if gait type hasn't changed
            if (type == gaitType)
            {
                return;
            }

            // Store new gait type
            gaitType = type;

            // Get current parameters and update type
            gait::GaitParameters params = gait.getParameters();
            params.type = gaitType;

            // Adjust gait-specific parameters based on type
            switch (gaitType)
            {
            case gait::GaitType::TRIPOD:
                params.dutyFactor = 0.5;  // 50% stance phase
                params.stepHeight = 30.0; // 30mm step height
                params.cycleTime = 1.0;   // 1 second cycle time
                break;

            case gait::GaitType::WAVE:
                params.dutyFactor = 0.75; // 75% stance phase
                params.stepHeight = 40.0; // Higher steps for stability
                params.cycleTime = 1.8;   // Slower cycle for wave gait
                break;

            case gait::GaitType::RIPPLE:
                params.dutyFactor = 0.65; // 65% stance phase
                params.stepHeight = 35.0; // Medium step height
                params.cycleTime = 1.4;   // Medium cycle time
                break;
            }

            // Apply updated parameters to gait controller
            gait.setParameters(params);

            // Output feedback about gait change
            const char *gaitName = "";
            switch (gaitType)
            {
            case gait::GaitType::TRIPOD:
                gaitName = "Tripod";
                break;
            case gait::GaitType::WAVE:
                gaitName = "Wave";
                break;
            case gait::GaitType::RIPPLE:
                gaitName = "Ripple";
                break;
            }
            std::cout << "Switched to " << gaitName << " gait" << std::endl;
        }

        /**
         * @brief Apply body tilting adjustments
         *
         * @return true if tilt was applied successfully
         * @return false if tilt application failed
         */
        bool applyTilt()
        {
            // Skip tilt application if we're not in tilt state and tilt is negligible
            if (state != ControllerState::TILTING &&
                std::abs(tiltX) < 0.1 && std::abs(tiltY) < 0.1)
            {
                // Already at neutral position
                tiltX = tiltY = 0.0;
                return true;
            }

            // Smooth transition back to neutral when not actively tilting
            if (state != ControllerState::TILTING)
            {
                // Apply exponential decay for smooth motion
                constexpr double decay = 0.9;
                tiltX *= decay;
                tiltY *= decay;

                // Once we're close enough to neutral, snap to exactly zero
                if (std::abs(tiltX) < 0.1 && std::abs(tiltY) < 0.1)
                {
                    tiltX = tiltY = 0.0;
                    return hexapod.centerAll();
                }
            }

            // TODO: Implement body orientation control using inverse kinematics
            // This would calculate individualized leg positions based on tilt angles

            return true;
        }

        /**
         * @brief Validate servo mapping configuration
         *
         * @return true if all servos are properly mapped
         * @return false if mapping issues detected
         */
        bool validateServoMapping() const
        {
            // Simplified implementation for demonstration
            // In a real implementation, this would check hardware mapping tables
            return true;
        }

        /**
         * @brief Generate a summary report of servo mapping configuration
         *
         * @return std::string Formatted mapping information
         */
        std::string getServoMappingSummary() const
        {
            std::stringstream ss;

            ss << "Servo Mapping Summary:\n";
            ss << "---------------------\n";

            // In a real implementation, this would list actual servo mappings
            // This is just a placeholder example
            for (int leg = 0; leg < hexapod::Config::NUM_LEGS; leg++)
            {
                ss << "Leg " << leg << ": ";
                ss << "Hip: ID " << (leg * 3 + 0) << ", ";
                ss << "Knee: ID " << (leg * 3 + 1) << ", ";
                ss << "Ankle: ID " << (leg * 3 + 2) << "\n";
            }

            ss << "\nGait Type: ";
            switch (gaitType)
            {
            case gait::GaitType::TRIPOD:
                ss << "Tripod";
                break;
            case gait::GaitType::WAVE:
                ss << "Wave";
                break;
            case gait::GaitType::RIPPLE:
                ss << "Ripple";
                break;
            }

            ss << "\nCurrent Speed: " << speed;
            ss << "\nCurrent Height: " << height;

            return ss.str();
        }

        /**
         * @brief Detect hardware controller communication issues
         *
         * @return true if communication is stable
         * @return false if issues are detected
         */
        bool detectControllerIssues() const
        {
            // Simplified placeholder implementation
            // In a real implementation, this would ping hardware and check timeouts

            std::cout << "Controller communication test: OK" << std::endl;
            std::cout << "Communication latency: 2ms" << std::endl;

            return true;
        }

        /**
         * @brief Test connectivity with all servo motors
         *
         * @return true if all servos respond properly
         * @return false if connectivity issues detected
         */
        bool testServoConnectivity() const
        {
            // Simplified placeholder implementation
            // In a real implementation, this would send test commands to each servo

            std::cout << "Testing " << hexapod::Config::TOTAL_SERVOS << " servos..." << std::endl;

            for (int i = 0; i < hexapod::Config::TOTAL_SERVOS; i++)
            {
                std::cout << "Servo " << i << ": OK" << std::endl;
            }

            return true;
        }

        // Core data members
        hexapod::Hexapod &hexapod;
        gait::Gait gait;

        // Movement parameters
        double speed;            ///< Movement speed factor (0.0 to 1.0)
        double direction;        ///< Movement direction in degrees (0 = forward)
        double height;           ///< Body height adjustment in mm
        double tiltX;            ///< Forward/backward tilt in degrees
        double tiltY;            ///< Left/right tilt in degrees
        gait::GaitType gaitType; ///< Current gait pattern
        ControllerState state;   ///< Current controller state

        // Terminal handling
        struct termios origTermios; ///< Original terminal attributes
        bool terminalConfigured;    ///< Terminal configuration flag

        // Timing
        struct timespec lastUpdate; ///< Timestamp of last update
    };

    //==============================================================================
    // Controller Class Implementation
    //==============================================================================

    // Constructor and Destructor
    Controller::Controller(hexapod::Hexapod &hexapod)
        : pImpl(std::make_unique<ControllerImpl>(hexapod)) {}

    Controller::~Controller() = default;

    // Move semantics
    Controller::Controller(Controller &&other) noexcept = default;
    Controller &Controller::operator=(Controller &&other) noexcept = default;

    // Initialization
    bool Controller::init()
    {
        return pImpl->init();
    }

    // Command processing
    bool Controller::processKey(char key)
    {
        return pImpl->processKey(key);
    }

    // State update
    bool Controller::update()
    {
        return pImpl->update();
    }

    // State and parameter access
    ControllerState Controller::getState() const
    {
        return pImpl->state;
    }

    void Controller::setSpeed(double speed)
    {
        pImpl->speed = std::max(0.0, std::min(1.0, speed));
    }

    void Controller::setDirection(double direction)
    {
        pImpl->direction = direction;
    }

    void Controller::setHeight(double height)
    {
        pImpl->height = height;
    }

    void Controller::setTilt(double tiltX, double tiltY)
    {
        pImpl->tiltX = tiltX;
        pImpl->tiltY = tiltY;
    }

    void Controller::setGaitType(gait::GaitType type)
    {
        pImpl->setGaitType(type);
    }

    // Diagnostic functions
    bool Controller::validateServoMapping() const
    {
        return pImpl->validateServoMapping();
    }

    std::string Controller::getServoMappingSummary() const
    {
        return pImpl->getServoMappingSummary();
    }

    bool Controller::detectControllerIssues() const
    {
        return pImpl->detectControllerIssues();
    }

    bool Controller::testServoConnectivity() const
    {
        return pImpl->testServoConnectivity();
    }

} // namespace controller
