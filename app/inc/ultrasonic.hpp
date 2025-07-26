#ifndef ULTRASONIC_HPP
#define ULTRASONIC_HPP

class Ultrasonic
{
public:
    Ultrasonic();
    ~Ultrasonic();

    bool init();
    float getDistance(); // Returns distance in cm

private:
    static const int TRIG_PIN = 12; // P8_12
    static const int ECHO_PIN = 11; // P8_11

    bool exportGPIO(int pin);
    bool setDirection(int pin, const char *direction);
    bool writeGPIO(int pin, int value);
    int readGPIO(int pin);
};

#endif
