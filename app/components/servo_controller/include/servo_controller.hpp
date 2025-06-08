#pragma once

#include <cstdint>

class ServoController {
public:
    // Gọi hàm này một lần ở setup
    static void init();

    // Gửi góc đến servo cụ thể (0–31)
    static void setAngle(uint8_t channel, float angle_deg);

private:
    static constexpr uint8_t PCA9685_ADDR_1 = 0x40; // Địa chỉ I2C cho PCA9685 số 1
    static constexpr uint8_t PCA9685_ADDR_2 = 0x41; // Địa chỉ I2C cho PCA9685 số 2
    static constexpr uint32_t SERVO_MIN_PULSE = 500;   // us
    static constexpr uint32_t SERVO_MAX_PULSE = 2500;  // us

    static void writePWM(uint8_t device_addr, uint8_t channel, uint16_t on, uint16_t off);
    static uint16_t angleToPWM(float angle_deg);
};
