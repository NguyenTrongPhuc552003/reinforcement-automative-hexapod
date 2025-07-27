#ifndef PCA9685_HPP
#define PCA9685_HPP

#include <cstdint>
#include <string>

class PCA9685
{
public:
    static constexpr uint8_t DEFAULT_ADDR_1 = 0x40;
    static constexpr uint8_t DEFAULT_ADDR_2 = 0x41;
    static constexpr uint16_t SERVO_MIN_PULSE = 1000;  // us
    static constexpr uint16_t SERVO_MAX_PULSE = 2000;  // us
    static constexpr uint16_t SERVO_HOME_PULSE = 1500; // us
    static constexpr uint8_t CHANNELS_PER_DEVICE = 16;
    static constexpr uint8_t TOTAL_SERVOS = 18;

    PCA9685();
    ~PCA9685();

    bool init();
    void cleanup();
    bool setServoMicroseconds(uint8_t channel, uint16_t microseconds);
    bool setAllServosHome();

private:
    bool initialized_;
    int device_fd_;

    bool writeRegister(uint8_t device_addr, uint8_t reg, uint8_t value);
    bool setPWM(uint8_t device_addr, uint8_t channel, uint16_t on, uint16_t off);
    uint16_t microsecondsToTicks(uint16_t microseconds);
};

#endif // PCA9685_HPP
