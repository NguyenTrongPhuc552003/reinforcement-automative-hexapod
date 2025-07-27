#ifndef ULTRASONIC_HPP
#define ULTRASONIC_HPP

#include <cstdint>
#include <string>

class Ultrasonic
{
public:
    // HC-SR04 pins on BeagleBone Black - corrected mapping
    static constexpr uint8_t TRIG_PIN = 44; // P8_12 (GPIO44)
    static constexpr uint8_t ECHO_PIN = 45; // P8_11 (GPIO45)

    Ultrasonic();

    bool init();
    double getDistance(); // Return distance in cm, -1.0 on error
    void cleanup();

private:
    uint8_t trigger_pin_;
    uint8_t echo_pin_;
    bool initialized_;

    // GPIO control functions
    bool exportGPIO(uint8_t pin);
    bool unexportGPIO(uint8_t pin);
    bool setGPIODirection(uint8_t pin, const std::string &direction);
    bool setGPIOValue(uint8_t pin, int value);
    int getGPIOValue(uint8_t pin);

    // Precise delay function like C code
    void delayMicroseconds(int us);
};

#endif // ULTRASONIC_HPP
