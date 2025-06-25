#ifndef SHARP_SENSOR_HPP
#define SHARP_SENSOR_HPP

#include <string>
#include <memory>

/**
 * @brief Class to interface with Sharp GP2Y0A21YK0F IR distance sensor
 *
 * This sensor provides analog output corresponding to distance in the range of 10-80cm.
 * The BeagleBone's ADC is used to read the analog values from the sensor.
 */
class SharpSensor
{
public:
    /**
     * @brief Construct a SharpSensor object
     *
     * @param ain_path Path to the ADC input (e.g., "AIN0", "AIN1", etc.)
     */
    explicit SharpSensor(const std::string &ain_path);

    /**
     * @brief Destroy the SharpSensor object
     */
    ~SharpSensor();

    /**
     * @brief Initialize the sensor
     *
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool init();

    /**
     * @brief Read raw ADC value from the sensor
     *
     * @return int Raw ADC value (0-4095 for BeagleBone AI)
     */
    int readRawValue();

    /**
     * @brief Read distance in centimeters
     *
     * @return float Distance in centimeters (10-80cm range)
     */
    float readDistanceCm();

    /**
     * @brief Set the ADC scaling value
     *
     * @param scale Scaling factor (default: 1.0)
     */
    void setScale(float scale);

    /**
     * @brief Set the ADC offset value
     *
     * @param offset Offset value (default: 0)
     */
    void setOffset(float offset);

    /**
     * @brief Enable or disable filtering of readings
     *
     * @param enable True to enable filtering, false to disable
     */
    void enableFiltering(bool enable);

    /**
     * @brief Take multiple readings and return the average
     *
     * @param samples Number of samples to average
     * @return float Average distance in centimeters
     */
    float readAverageDistanceCm(int samples = 5);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // SHARP_SENSOR_HPP
