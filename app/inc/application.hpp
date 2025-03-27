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
 * @brief Hexapod application system
 *
 * This namespace contains classes and functions for the main hexapod
 * application, providing a high-level interface to the robot control system.
 */
namespace application
{

    //==============================================================================
    // Application Modes
    //==============================================================================

    /**
     * @brief Application control modes
     */
    enum class ControlMode
    {
        MANUAL,     ///< Direct keyboard control
        AUTONOMOUS, ///< Self-navigating mode
        SEQUENCE,   ///< Run pre-programmed sequence
        CALIBRATION ///< Calibration mode
    };

    /**
     * @brief Application execution results
     */
    enum class ExecutionResult
    {
        SUCCESS,              ///< Normal successful completion
        ERROR_INITIALIZATION, ///< Error during initialization
        ERROR_RUNTIME,        ///< Error during execution
        ERROR_SHUTDOWN,       ///< Error during shutdown
        TERMINATED_BY_USER    ///< User requested termination
    };

    //==============================================================================
    // Forward declarations
    //==============================================================================

    // Implementation class (PIMPL idiom)
    class ApplicationImpl;

    //==============================================================================
    // Main Application Class
    //==============================================================================

    /**
     * @brief Main hexapod application class
     *
     * Orchestrates the hexapod robot control system, providing a high-level
     * interface for user interaction and system management.
     */
    class Application
    {
    public:
        /**
         * @brief Running state flag (shared between signal handler and main loop)
         */
        static std::atomic<bool> m_running;

        /**
         * @brief Telemetry display flag
         */
        static std::atomic<bool> m_telemetryActive;

        /**
         * @brief Get the singleton instance
         *
         * @return Application& Reference to the singleton instance
         */
        static Application &getInstance();

        // Delete copy and move operations for singleton
        Application(const Application &) = delete;
        Application &operator=(const Application &) = delete;
        Application(Application &&) = delete;
        Application &operator=(Application &&) = delete;

        //--------------------------------------------------------------------------
        // Lifecycle Methods
        //--------------------------------------------------------------------------

        /**
         * @brief Initialize the application
         *
         * Sets up all subsystems including hexapod hardware, controller,
         * terminal input handling, and key command registration.
         *
         * @return true if initialization successful
         * @return false if initialization failed (check getLastError())
         */
        bool init();

        /**
         * @brief Run the main application loop
         *
         * Processes input, updates the controller state, and handles system monitoring
         * until termination is requested.
         *
         * @return ExecutionResult Result of the execution
         */
        ExecutionResult run();

        /**
         * @brief Shutdown the application
         *
         * Performs a clean shutdown of all subsystems, ensuring the robot
         * is left in a safe state.
         */
        void shutdown();

        //--------------------------------------------------------------------------
        // Control Methods
        //--------------------------------------------------------------------------

        /**
         * @brief Switch to a different control mode
         *
         * @param mode New control mode to switch to
         * @return true if mode switch was successful
         * @return false if mode switch failed
         */
        bool switchMode(ControlMode mode);

        /**
         * @brief Get the current control mode
         *
         * @return ControlMode Current control mode
         */
        ControlMode getCurrentMode() const;

        /**
         * @brief Get the last error message
         *
         * @return std::string Error message
         */
        std::string getLastErrorMessage() const;

        //--------------------------------------------------------------------------
        // Signal Handling
        //--------------------------------------------------------------------------

        /**
         * @brief Signal handler for system signals
         *
         * @param signal Signal number
         */
        static void signalHandler(int signal);

    private:
        /**
         * @brief Private constructor for singleton pattern
         */
        Application();

        /**
         * @brief Destructor
         *
         * Ensures clean shutdown
         */
        ~Application();

        /**
         * @brief Implementation pointer (PIMPL idiom)
         */
        std::unique_ptr<ApplicationImpl> pImpl;
    };

} // namespace application

// For backward compatibility, import into global namespace
using application::Application;
using application::ControlMode;
using application::ExecutionResult;

#endif /* APPLICATION_HPP */
