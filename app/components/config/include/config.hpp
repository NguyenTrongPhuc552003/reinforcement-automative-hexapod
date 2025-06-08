#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>

struct Config {
    static constexpr const char* WIFI_SSID = "HexapodAP";  // Added const
    static constexpr const char* WIFI_PASS = "hexapod123"; // Added const
    
    // I2C Configuration
    static constexpr int I2C_SDA_PIN = 21;
    static constexpr int I2C_SCL_PIN = 22;
    static constexpr int I2C_FREQ = 400000;
    
    // Servo limits
    static constexpr float SERVO_MIN_ANGLE = 0.0f;
    static constexpr float SERVO_MAX_ANGLE = 180.0f;
    static constexpr float SERVO_MIN_PULSE = 500;  // microseconds
    static constexpr float SERVO_MAX_PULSE = 2500; // microseconds
    
    // Hexapod dimensions
    static constexpr float COXA_LENGTH = 25.0f;   // mm
    static constexpr float FEMUR_LENGTH = 50.0f;  // mm
    static constexpr float TIBIA_LENGTH = 75.0f;  // mm
    
    // Gait parameters
    static constexpr float STEP_HEIGHT = 30.0f;   // mm
    static constexpr float STEP_LENGTH = 50.0f;   // mm
    static constexpr float CYCLE_TIME = 1.0f;     // seconds
};

#endif // CONFIG_HPP
