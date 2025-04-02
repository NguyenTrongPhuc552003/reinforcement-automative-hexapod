#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <memory>
#include "gait.hpp"

/**
 * @brief Hexapod robot controller system
 *
 * This namespace contains classes and functions for high-level control
 * of hexapod robot movements and behaviors.
 */
namespace controller
{

    //==============================================================================
    // Controller States
    //==============================================================================

    /**
     * @brief Movement states of the controller
     */
    enum class ControllerState
    {
        IDLE,     ///< No movement, standing still
        WALKING,  ///< Forward/backward linear motion
        ROTATING, ///< Turning motion (left/right)
        TILTING   ///< Body orientation tilting
    };

    /**
     * @brief Balance configuration parameters
     */
    struct BalanceConfig
    {
        double max_tilt_adjustment = 30.0; // Maximum angle adjustment in degrees
        double response_factor = 0.8;      // How responsive the balance is (0.0-1.0)
        double deadzone = 2.0;             // Minimum tilt (degrees) before reacting
        bool enabled = false;              // Whether balance mode is enabled
    };

    //==============================================================================
    // Forward declarations
    //==============================================================================

    // Implementation class (PIMPL idiom)
    class ControllerImpl;

    //==============================================================================
    // Main Controller Class
    //==============================================================================

    /**
     * @brief High-level controller for hexapod robot
     *
     * Provides simplified interfaces for robot control, manages movement
     * logic, and interprets user commands into appropriate robot actions.
     */
    class Controller
    {
    public:
        /**
         * @brief Construct a new Controller object
         *
         * @param hexapod Reference to hexapod hardware interface
         */
        explicit Controller(hexapod::Hexapod &hexapod);

        /**
         * @brief Destroy the Controller object
         *
         * Ensures all resources are properly cleaned up
         */
        ~Controller();

        // Non-copyable
        Controller(const Controller &) = delete;
        Controller &operator=(const Controller &) = delete;

        // Move semantics
        Controller(Controller &&other) noexcept;
        Controller &operator=(Controller &&other) noexcept;

        //--------------------------------------------------------------------------
        // Initialization and Configuration
        //--------------------------------------------------------------------------

        /**
         * @brief Initialize the controller
         *
         * Sets up the internal state, terminal handling, and gait controller.
         * Must be called before using other methods.
         *
         * @return true if initialization successful
         * @return false if initialization failed
         */
        bool init();

        //--------------------------------------------------------------------------
        // Control Interface
        //--------------------------------------------------------------------------

        /**
         * @brief Process a keyboard input command
         *
         * Maps keyboard commands to robot actions
         *
         * @param key Character representing the keyboard key pressed
         * @return true if command was processed successfully
         * @return false if command failed
         */
        bool processKey(char key);

        /**
         * @brief Update the robot state based on current inputs
         *
         * Should be called regularly in a control loop to maintain movement
         *
         * @return true if update was successful
         * @return false if update failed
         */
        bool update();

        //--------------------------------------------------------------------------
        // State and Parameter Access
        //--------------------------------------------------------------------------

        /**
         * @brief Get the current controller state
         *
         * @return ControllerState Current movement state
         */
        ControllerState getState() const;

        /**
         * @brief Set the movement speed
         *
         * @param speed Speed factor (0.0 to 1.0)
         */
        void setSpeed(double speed);

        /**
         * @brief Set the movement direction
         *
         * @param direction Direction in degrees (0 = forward)
         */
        void setDirection(double direction);

        /**
         * @brief Set the body height
         *
         * @param height Height adjustment in mm
         */
        void setHeight(double height);

        /**
         * @brief Set the body tilt
         *
         * @param tiltX Forward/backward tilt in degrees
         * @param tiltY Left/right tilt in degrees
         */
        void setTilt(double tiltX, double tiltY);

        /**
         * @brief Set the gait type
         *
         * @param type Gait pattern type
         */
        void setGaitType(gait::GaitType type);

        //--------------------------------------------------------------------------
        // Balance Control
        //--------------------------------------------------------------------------

        /**
         * @brief Enable or disable balance mode
         *
         * @param enabled True to enable balance, false to disable
         */
        void setBalanceEnabled(bool enabled);

        /**
         * @brief Check if balance mode is enabled
         *
         * @return true if balance mode is enabled
         * @return false if balance mode is disabled
         */
        bool isBalanceEnabled() const;

        /**
         * @brief Set balance response factor
         *
         * @param factor Response factor (0.1-1.0)
         */
        void setBalanceResponseFactor(double factor);

        /**
         * @brief Set balance deadzone
         *
         * @param degrees Deadzone in degrees (0.0-10.0)
         */
        void setBalanceDeadzone(double degrees);

        /**
         * @brief Get current balance configuration
         *
         * @return BalanceConfig Current balance settings
         */
        BalanceConfig getBalanceConfig() const;

        //--------------------------------------------------------------------------
        // Diagnostic Functions
        //--------------------------------------------------------------------------

        /**
         * @brief Validate servo mapping configuration
         *
         * @return true if servo mapping is valid
         * @return false if servo mapping has issues
         */
        bool validateServoMapping() const;

        /**
         * @brief Get a summary of servo mapping configuration
         *
         * @return std::string Formatted servo mapping summary
         */
        std::string getServoMappingSummary() const;

        /**
         * @brief Detect controller communication issues
         *
         * @return true if communication is healthy
         * @return false if communication issues detected
         */
        bool detectControllerIssues() const;

        /**
         * @brief Test servo connectivity and response
         *
         * @return true if all servos respond correctly
         * @return false if servo connectivity issues detected
         */
        bool testServoConnectivity() const;

    private:
        /**
         * @brief Implementation pointer (PIMPL idiom)
         */
        std::unique_ptr<ControllerImpl> pImpl;
    };

} // namespace controller

// For backward compatibility, import types into global namespace
using controller::Controller;
using controller::ControllerState;

#endif /* CONTROLLER_HPP */
