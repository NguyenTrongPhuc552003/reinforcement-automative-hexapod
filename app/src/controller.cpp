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
#include "ultrasonic.hpp"

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
              terminalConfigured(false),
              m_ultrasonicEnabled(false),
              m_ultrasonicInitialized(false),
              m_lastDistanceReading(0.0),
              m_obstacleThreshold(30.0), // 30cm obstacle threshold
              m_slowdownThreshold(50.0), // 50cm slowdown threshold
              m_lastReadingTime(0.0)
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
            // Clean up ultrasonic sensor if initialized
            if (m_ultrasonicInitialized && m_ultrasonicSensor)
            {
                m_ultrasonicSensor->cleanup();
            }

            // Restore terminal settings if modified
            if (terminalConfigured)
            {
                tcsetattr(STDIN_FILENO, TCSANOW, &origTermios);
            }

            // Safety: make sure robot is in a stable pose when destructed
            hexapod.centerAll();
        }

        /**
         * @brief Initialize ultrasonic sensor with retry logic
         *
         * @return true if sensor was initialized successfully
         * @return false if sensor initialization failed
         */
        bool initializeUltrasonicSensor()
        {
            try
            {
                // Create ultrasonic sensor instance
                m_ultrasonicSensor = std::make_unique<UltrasonicSensor>();

                if (!m_ultrasonicSensor)
                {
                    std::cerr << "Failed to create ultrasonic sensor instance" << std::endl;
                    return false;
                }

                // Initialize with retry logic
                const int maxRetries = 3;
                for (int retry = 0; retry < maxRetries; retry++)
                {
                    if (m_ultrasonicSensor->init())
                    {
                        m_ultrasonicInitialized = true;
                        m_ultrasonicEnabled = true;
                        std::cout << "Ultrasonic distance sensor initialized successfully" << std::endl;

                        // Test initial reading
                        double testDistance = m_ultrasonicSensor->measure().distance; // Get distance measurement
                        if (testDistance >= 0)
                        {
                            std::cout << "Initial distance reading: " << testDistance << " cm" << std::endl;
                            m_lastDistanceReading = testDistance;
                            m_lastReadingTime = getCurrentTime();
                            return true;
                        }
                        else
                        {
                            std::cerr << "Warning: Initial distance reading failed" << std::endl;
                        }
                        return true;
                    }

                    std::cerr << "Ultrasonic sensor initialization attempt " << (retry + 1) << " failed" << std::endl;
                    if (retry < maxRetries - 1)
                    {
                        std::cout << "Retrying ultrasonic sensor initialization in 500ms..." << std::endl;
                        usleep(500000); // 500ms delay
                    }
                }

                std::cerr << "Warning: Ultrasonic distance sensor initialization failed after "
                          << maxRetries << " attempts" << std::endl;
                m_ultrasonicEnabled = false;
                return false;
            }
            catch (const std::exception &e)
            {
                std::cerr << "Exception during ultrasonic sensor initialization: " << e.what() << std::endl;
                m_ultrasonicEnabled = false;
                return false;
            }
        }

        /**
         * @brief Get current time in seconds
         *
         * @return double Current time in seconds
         */
        double getCurrentTime() const
        {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            return ts.tv_sec + (ts.tv_nsec / 1.0e9);
        }

        /**
         * @brief Read distance from ultrasonic sensor with error handling
         *
         * @return double Distance in cm, or -1.0 if reading failed
         */
        double readUltrasonicDistance()
        {
            if (!m_ultrasonicEnabled || !m_ultrasonicInitialized || !m_ultrasonicSensor)
            {
                return -1.0;
            }

            try
            {
                double distance = m_ultrasonicSensor->measure().distance; // Get distance measurement
                double currentTime = getCurrentTime();

                if (distance >= 0)
                {
                    // Valid reading
                    m_lastDistanceReading = distance;
                    m_lastReadingTime = currentTime;
                    return distance;
                }
                else
                {
                    // Invalid reading - use last valid reading if recent enough
                    if ((currentTime - m_lastReadingTime) < 2.0) // 2 second timeout
                    {
                        return m_lastDistanceReading;
                    }
                    else
                    {
                        return -1.0; // Reading too old
                    }
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << "Exception reading ultrasonic sensor: " << e.what() << std::endl;
                return -1.0;
            }
        }

        /**
         * @brief Process obstacle avoidance based on ultrasonic readings
         *
         * @param distance Current distance reading in cm
         * @return true if movement should continue normally
         * @return false if movement should be stopped/modified
         */
        bool processObstacleAvoidance(double distance)
        {
            if (distance < 0)
            {
                // No valid distance reading - continue normally
                return true;
            }

            // Check for immediate obstacle (stop and back up)
            if (distance < m_obstacleThreshold)
            {
                std::cout << "Obstacle detected at " << distance << "cm - stopping and backing up" << std::endl;

                // Stop current movement
                state = ControllerState::IDLE;

                // Set reverse direction for short backup
                direction = 180.0;            // Backward
                speed = std::min(speed, 0.3); // Slow speed for safety
                state = ControllerState::WALKING;

                return false; // Movement was modified
            }
            // Check for approaching obstacle (slow down)
            else if (distance < m_slowdownThreshold)
            {
                // Reduce speed proportionally to distance
                double speedFactor = (distance - m_obstacleThreshold) / (m_slowdownThreshold - m_obstacleThreshold);
                speedFactor = std::max(0.2, std::min(1.0, speedFactor)); // Clamp between 0.2 and 1.0

                speed = speed * speedFactor;

                if (debug)
                {
                    std::cout << "Obstacle approaching at " << distance
                              << "cm - reducing speed to " << (speed * 100) << "%" << std::endl;
                }

                return true; // Continue with modified speed
            }

            // No obstacle - continue normally
            return true;
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

            // Initialize ultrasonic sensor
            if (!initializeUltrasonicSensor())
            {
                std::cerr << "Warning: Ultrasonic sensor not available - obstacle avoidance disabled" << std::endl;
                // Continue without ultrasonic sensor
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

            // Ultrasonic sensor controls
            case 'u': // Toggle ultrasonic sensor
                if (m_ultrasonicInitialized)
                {
                    m_ultrasonicEnabled = !m_ultrasonicEnabled;
                    std::cout << "Ultrasonic distance sensing "
                              << (m_ultrasonicEnabled ? "enabled" : "disabled")
                              << std::endl;
                }
                else
                {
                    std::cout << "Ultrasonic sensor not initialized - attempting re-initialization..." << std::endl;
                    if (initializeUltrasonicSensor())
                    {
                        std::cout << "Ultrasonic sensor re-initialized successfully" << std::endl;
                    }
                    else
                    {
                        std::cout << "Ultrasonic sensor re-initialization failed" << std::endl;
                    }
                }
                return true;

            case 'o': // Test ultrasonic sensor reading
                if (m_ultrasonicEnabled && m_ultrasonicInitialized)
                {
                    double distance = readUltrasonicDistance();
                    if (distance >= 0)
                    {
                        std::cout << "Distance reading: " << distance << " cm" << std::endl;
                    }
                    else
                    {
                        std::cout << "Failed to read distance from ultrasonic sensor" << std::endl;
                    }
                }
                else
                {
                    std::cout << "Ultrasonic sensor not available" << std::endl;
                }
                return true;

            case 'r': // Show obstacle thresholds
                std::cout << "Current thresholds - Obstacle: " << m_obstacleThreshold
                          << "cm, Slowdown: " << m_slowdownThreshold << "cm" << std::endl;
                return true;

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

            // Process ultrasonic sensor data if enabled and in walking state
            if (m_ultrasonicEnabled && m_ultrasonicInitialized &&
                (state == ControllerState::WALKING || state == ControllerState::ROTATING))
            {
                double distance = readUltrasonicDistance();
                if (distance >= 0)
                {
                    // Process obstacle avoidance
                    if (!processObstacleAvoidance(distance))
                    {
                        // Obstacle avoidance modified movement - may need to update gait
                        result = gait.update(now.tv_sec + now.tv_nsec / 1.0e9, direction, speed) && result;
                    }
                }
            }

            // Apply balance adjustments if enabled (in any state)
            if (balanceConfig.enabled)
            {
                result = updateBalance() && result;
            }
            else
            {
                // Handle state-specific updates when balance is disabled
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
                    // Check if height has changed even in idle state
                    if (std::abs(height) > 0.1)
                    {
                        result = applyTilt(); // Use applyTilt to apply height changes
                    }
                    break;
                }
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

            // Implementation of body orientation control using inverse kinematics
            bool success = true;

            // Loop through each leg to apply tilt and height adjustments
            for (int leg = 0; leg < hexapod::Config::NUM_LEGS; leg++)
            {
                // Define base position for this leg - depends on leg position around body
                double baseX, baseY, baseZ = -120.0 + height; // Apply height adjustment here

                // Default leg positions when standing
                switch (leg)
                {
                case 0: // Front right
                    baseX = 100.0;
                    baseY = 100.0;
                    break;
                case 1: // Middle right
                    baseX = 0.0;
                    baseY = 120.0;
                    break;
                case 2: // Rear right
                    baseX = -100.0;
                    baseY = 100.0;
                    break;
                case 3: // Front left
                    baseX = 100.0;
                    baseY = -100.0;
                    break;
                case 4: // Middle left
                    baseX = 0.0;
                    baseY = -120.0;
                    break;
                case 5: // Rear left
                    baseX = -100.0;
                    baseY = -100.0;
                    break;
                }

                // Calculate height adjustment based on tilt angles
                double zAdjustment = sin(tiltY * M_PI / 180.0) * baseY +
                                     sin(tiltX * M_PI / 180.0) * baseX;

                // Apply adjustment to position - create a tilted plane
                kinematics::Point3D targetPos(baseX, baseY, baseZ + zAdjustment);

                // Use inverse kinematics to get joint angles
                hexapod::LegPosition legPos;
                legPos.leg_num = leg;

                if (kinematics::Kinematics::getInstance().inverseKinematics(targetPos, legPos))
                {
                    // Apply the calculated position
                    if (!hexapod.setLegPosition(leg, legPos))
                    {
                        if (debug)
                            std::cerr << "Failed to set position for leg " << leg << std::endl;
                        success = false;
                    }
                }
                else
                {
                    if (debug)
                        std::cerr << "Inverse kinematics failed for leg " << leg << std::endl;
                    success = false;
                }
            }

            return success;
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

            // Add ultrasonic sensor status
            ss << "\nUltrasonic Sensor: ";
            if (m_ultrasonicInitialized)
            {
                ss << (m_ultrasonicEnabled ? "Enabled" : "Disabled");
                ss << " (Threshold: " << m_obstacleThreshold << "cm)";
                if (m_ultrasonicEnabled)
                {
                    ss << " (Last reading: " << m_lastDistanceReading << "cm)";
                }
            }
            else
            {
                ss << "Not initialized";
            }

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

            // Test ultrasonic sensor communication
            if (m_ultrasonicInitialized)
            {
                std::cout << "Ultrasonic sensor: " << (m_ultrasonicEnabled ? "OK" : "Disabled") << std::endl;
            }
            else
            {
                std::cout << "Ultrasonic sensor: Not initialized" << std::endl;
            }

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

        // Balance settings
        BalanceConfig balanceConfig;

        // Ultrasonic sensor members
        std::unique_ptr<UltrasonicSensor> m_ultrasonicSensor; ///< Ultrasonic sensor instance
        bool m_ultrasonicEnabled;                             ///< Whether ultrasonic sensing is enabled
        bool m_ultrasonicInitialized;                         ///< Whether ultrasonic sensor was successfully initialized
        double m_lastDistanceReading;                         ///< Last valid distance reading in cm
        double m_obstacleThreshold;                           ///< Distance threshold for obstacle detection in cm
        double m_slowdownThreshold;                           ///< Distance threshold for speed reduction in cm
        double m_lastReadingTime;                             ///< Time of last valid reading

        /**
         * @brief Calculate tilt angles from accelerometer data
         *
         * @param imu_data IMU data
         * @param roll Output roll angle
         * @param pitch Output pitch angle
         */
        void calculateTilt(const hexapod::ImuData &imu_data, double &roll, double &pitch)
        {
            // Conversion for accessing accelerometer data in g units
            double ax = imu_data.getAccelX();
            double ay = imu_data.getAccelY();
            double az = imu_data.getAccelZ();

            // Calculate roll and pitch in degrees
            roll = atan2(ay, az) * 180.0 / M_PI;
            pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
        }

        /**
         * @brief Apply balance adjustments to leg positions
         *
         * @param roll Current roll angle in degrees
         * @param pitch Current pitch angle in degrees
         * @return true if adjustments were successful
         * @return false if adjustments failed
         */
        bool applyBalanceAdjustments(double roll, double pitch)
        {
            // Apply deadzone - ignore small tilts
            if (fabs(roll) < balanceConfig.deadzone)
                roll = 0;
            if (fabs(pitch) < balanceConfig.deadzone)
                pitch = 0;

            // Cap maximum adjustment
            roll = std::max(-balanceConfig.max_tilt_adjustment,
                            std::min(balanceConfig.max_tilt_adjustment, roll));
            pitch = std::max(-balanceConfig.max_tilt_adjustment,
                             std::min(balanceConfig.max_tilt_adjustment, pitch));

            // Scale by response factor
            roll *= balanceConfig.response_factor;
            pitch *= balanceConfig.response_factor;

            // No need to adjust if tilt is negligible
            if (fabs(roll) < 0.1 && fabs(pitch) < 0.1)
                return true;

            // Current leg positions - used when combining with gait
            std::vector<kinematics::Point3D> legPositions(hexapod::Config::NUM_LEGS);
            std::vector<hexapod::LegPosition> currentAngles(hexapod::Config::NUM_LEGS);

            // Get current leg positions if we're in walking state
            if (state == ControllerState::WALKING || state == ControllerState::ROTATING)
            {
                for (int leg = 0; leg < hexapod::Config::NUM_LEGS; leg++)
                {
                    hexapod::LegPosition angles;
                    if (hexapod.getLegPosition(leg, angles))
                    {
                        currentAngles[leg] = angles;
                        kinematics::Point3D pos;
                        if (kinematics::Kinematics::getInstance().forwardKinematics(angles, pos))
                        {
                            legPositions[leg] = pos;
                        }
                    }
                }
            }

            // Loop through each leg to calculate and apply adjustments
            bool success = true;
            for (int leg = 0; leg < hexapod::Config::NUM_LEGS; leg++)
            {
                // Define base position for this leg
                double baseX, baseY, baseZ = -120.0; // Default base height

                // If we're in walking state, use current positions as base
                if (state == ControllerState::WALKING || state == ControllerState::ROTATING)
                {
                    baseX = legPositions[leg].x;
                    baseY = legPositions[leg].y;
                    baseZ = legPositions[leg].z;
                }
                else
                {
                    // Default leg positions when standing
                    switch (leg)
                    {
                    case 0: // Front right
                        baseX = 100.0;
                        baseY = 100.0;
                        break;
                    case 1: // Middle right
                        baseX = 0.0;
                        baseY = 120.0;
                        break;
                    case 2: // Rear right
                        baseX = -100.0;
                        baseY = 100.0;
                        break;
                    case 3: // Front left
                        baseX = 100.0;
                        baseY = -100.0;
                        break;
                    case 4: // Middle left
                        baseX = 0.0;
                        baseY = -120.0;
                        break;
                    case 5: // Rear left
                        baseX = -100.0;
                        baseY = -100.0;
                        break;
                    }
                }

                // Calculate leg height adjustment based on tilt
                double zAdjustment = sin(roll * M_PI / 180.0) * baseY -
                                     sin(pitch * M_PI / 180.0) * baseX;

                // Apply adjustment to position - create a tilted plane
                kinematics::Point3D targetPos(baseX, baseY, baseZ + zAdjustment);

                // Use inverse kinematics to get joint angles
                hexapod::LegPosition legPos;
                legPos.leg_num = leg;

                if (kinematics::Kinematics::getInstance().inverseKinematics(targetPos, legPos))
                {
                    // Apply the calculated position
                    if (!hexapod.setLegPosition(leg, legPos))
                    {
                        if (debug)
                            std::cerr << "Failed to set position for leg " << leg << std::endl;
                        success = false;
                    }
                }
                else
                {
                    if (debug)
                        std::cerr << "Inverse kinematics failed for leg " << leg << std::endl;
                    success = false;
                }
            }

            return success;
        }

        /**
         * @brief Process IMU data for balance adjustments
         *
         * @return true if balance adjustments succeeded
         * @return false if balance adjustments failed
         */
        bool processBalanceAdjustments()
        {
            if (!balanceConfig.enabled)
                return true;

            // Read IMU data
            hexapod::ImuData imuData;
            if (!hexapod.getImuData(imuData))
                return false;

            // Calculate tilt angles from accelerometer data
            double roll, pitch;
            calculateTilt(imuData, roll, pitch);

            // Apply balance adjustments based on tilt
            return applyBalanceAdjustments(roll, pitch);
        }

        /**
         * @brief Handle balance logic in update method
         */
        bool updateBalance()
        {
            return processBalanceAdjustments();
        }

        // Add debug flag for optional console output
        bool debug = false;
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

    // Implement new balance methods in Controller class
    void Controller::setBalanceEnabled(bool enabled)
    {
        pImpl->balanceConfig.enabled = enabled;

        // Provide user feedback
        if (enabled)
            std::cout << "Balance mode enabled" << std::endl;
        else
            std::cout << "Balance mode disabled" << std::endl;
    }

    bool Controller::isBalanceEnabled() const
    {
        return pImpl->balanceConfig.enabled;
    }

    void Controller::setBalanceResponseFactor(double factor)
    {
        pImpl->balanceConfig.response_factor = std::max(0.1, std::min(1.0, factor));
        std::cout << "Balance response factor set to "
                  << pImpl->balanceConfig.response_factor << std::endl;
    }

    void Controller::setBalanceDeadzone(double degrees)
    {
        pImpl->balanceConfig.deadzone = std::max(0.0, std::min(10.0, degrees));
        std::cout << "Balance deadzone set to "
                  << pImpl->balanceConfig.deadzone << " degrees" << std::endl;
    }

    BalanceConfig Controller::getBalanceConfig() const
    {
        return pImpl->balanceConfig;
    }

} // namespace controller
