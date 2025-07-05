#ifndef ULTRASONIC_HPP
#define ULTRASONIC_HPP

#include <string>
#include <memory>
#include <chrono>

/**
 * @brief HC-SR04 Ultrasonic Distance Sensor Interface
 *
 * This class provides an interface to control HC-SR04 ultrasonic distance sensors
 * using GPIO pins on the BeagleBone. It follows the same PIMPL pattern and error
 * handling conventions as other sensor classes in the project.
 */
class UltrasonicSensor
{
public:
    /**
     * @brief Sensor pinout definitions
     * This structure defines the GPIO pins used for the ultrasonic sensor.
     */
    struct Pinout
    {
        static constexpr const char *TRIGGER_PIN = "P9_12"; ///< GPIO pin for trigger
        static constexpr const char *ECHO_PIN = "P9_15";    ///< GPIO pin for echo
    };

    /**
     * @brief Sensor configuration structure
     */
    struct SensorConfig
    {
        std::string triggerPin; ///< GPIO pin for trigger (e.g., "P9_12")
        std::string echoPin;    ///< GPIO pin for echo (e.g., "P9_14")
        float maxDistance;      ///< Maximum detection distance in cm (default: 400.0)
        float minDistance;      ///< Minimum detection distance in cm (default: 2.0)
        int timeoutUs;          ///< Timeout for echo response in microseconds (default: 30000)
        bool filteringEnabled;  ///< Enable measurement filtering (default: true)
        int filterWindowSize;   ///< Size of moving average filter window (default: 5)

        /**
         * @brief Default constructor with sensible defaults
         */
        SensorConfig()
            : triggerPin(UltrasonicSensor::Pinout::TRIGGER_PIN), echoPin(UltrasonicSensor::Pinout::ECHO_PIN), maxDistance(400.0f),
              minDistance(2.0f), timeoutUs(30000), filteringEnabled(true),
              filterWindowSize(5) {}
    };

    /**
     * @brief Measurement result structure
     */
    struct Measurement
    {
        float distance;                                           ///< Distance in centimeters
        bool valid;                                               ///< Whether the measurement is valid
        std::chrono::high_resolution_clock::time_point timestamp; ///< When measurement was taken
        float rawDistance;                                        ///< Unfiltered distance value
        int echoTimeUs;                                           ///< Raw echo time in microseconds

        /**
         * @brief Default constructor
         */
        Measurement() : distance(0.0f), valid(false), rawDistance(0.0f), echoTimeUs(0) {}
    };

    /**
     * @brief Construct a new Ultrasonic Sensor object
     *
     * @param config Sensor configuration
     */
    explicit UltrasonicSensor(const SensorConfig &config = SensorConfig());

    /**
     * @brief Destroy the Ultrasonic Sensor object
     */
    ~UltrasonicSensor();

    /**
     * @brief Initialize the sensor
     *
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool init();

    /**
     * @brief Cleaning up the sensor
     */
    void cleanup();

    /**
     * @brief Take a single distance measurement
     *
     * @return Measurement Distance measurement result
     */
    Measurement measure();

    /**
     * @brief Take multiple measurements and return the average
     *
     * @param samples Number of samples to take (default: 5)
     * @param delayMs Delay between samples in milliseconds (default: 10)
     * @return Measurement Averaged measurement result
     */
    Measurement measureAverage(int samples = 5, int delayMs = 10);

    /**
     * @brief Get the current sensor configuration
     *
     * @return SensorConfig Current configuration
     */
    SensorConfig getConfig() const;

    /**
     * @brief Update sensor configuration
     *
     * @param config New configuration
     * @return true if configuration updated successfully
     * @return false if configuration update failed
     */
    void setConfig(const SensorConfig &config);

    /**
     * @brief Enable or disable measurement filtering
     *
     * @param enabled True to enable filtering, false to disable
     */
    void setFilteringEnabled(bool enabled);

    /**
     * @brief Get the last error message
     *
     * @return std::string Error message
     */
    std::string getLastError() const;

    /**
     * @brief Check if sensor is properly initialized
     *
     * @return true if sensor is initialized
     * @return false if sensor is not initialized
     */
    bool isInitialized() const;

    /**
     * @brief Check if an object is detected within the specified range
     *
     * @param maxRange Maximum detection range in cm
     * @return true if object detected within range
     * @return false if no object detected or error
     */
    bool isObjectDetected(float maxRange = 50.0f);

    /**
     * @brief Test sensor functionality
     *
     * @return true if sensor test passed
     * @return false if sensor test failed
     */
    bool selfTest();

    /**
     * @brief Get sensor statistics
     *
     * @return std::string Formatted statistics
     */
    std::string getStatistics() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // ULTRASONIC_HPP
