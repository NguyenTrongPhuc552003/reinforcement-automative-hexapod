#include "servo_controller.hpp"
#include "driver/i2c.h"
#include "esp_log.h"
#include <cmath>
#include <cstdio>

static const char* TAG = "ServoController";  // Added TAG definition

// I2C config
#define I2C_PORT I2C_NUM_0
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ_HZ 400000

#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06

void ServoController::init() {
    // Configure I2C
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_SDA_PIN;
    conf.scl_io_num = (gpio_num_t)I2C_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));

    // Give PCA9685s time to start up
    vTaskDelay(pdMS_TO_TICKS(50));

    auto initPCA9685 = [](uint8_t addr) -> esp_err_t {
        // Software reset
        uint8_t reset_cmd[2] = {PCA9685_MODE1, 0x80};
        ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT, addr, reset_cmd, 2, pdMS_TO_TICKS(100)));
        vTaskDelay(pdMS_TO_TICKS(10));

        // Set PWM frequency to 50Hz
        const float freq = 50.0f;
        const float prescale_val = (25000000 / (4096 * freq)) - 1;
        uint8_t prescale = static_cast<uint8_t>(roundf(prescale_val));
        
        // Set sleep mode to change prescale
        uint8_t sleep_cmd[2] = {PCA9685_MODE1, 0x10};
        ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT, addr, sleep_cmd, 2, pdMS_TO_TICKS(100)));
        
        // Set prescale
        uint8_t prescale_cmd[2] = {PCA9685_PRESCALE, prescale};
        ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT, addr, prescale_cmd, 2, pdMS_TO_TICKS(100)));
        
        // Wake up
        uint8_t wake_cmd[2] = {PCA9685_MODE1, 0x20}; // Auto-increment enabled
        return i2c_master_write_to_device(I2C_PORT, addr, wake_cmd, 2, pdMS_TO_TICKS(100));
    };

    // Initialize both PCA9685 boards
    ESP_ERROR_CHECK(initPCA9685(PCA9685_ADDR_1));
    ESP_ERROR_CHECK(initPCA9685(PCA9685_ADDR_2));

    ESP_LOGI(TAG, "ServoController initialized successfully");
}

void ServoController::setAngle(uint8_t channel, float angle_deg) {
    if (channel > 31) return;

    uint8_t device_addr = channel < 16 ? PCA9685_ADDR_1 : PCA9685_ADDR_2;
    uint8_t pwm_channel = channel % 16;

    uint16_t pulse_width = angleToPWM(angle_deg);
    uint16_t ticks = (pulse_width * 4096) / 20000; // 20ms period (50Hz)

    writePWM(device_addr, pwm_channel, 0, ticks);
}

uint16_t ServoController::angleToPWM(float angle_deg) {
    angle_deg = std::fmax(0.0f, std::fmin(180.0f, angle_deg));
    float pulse = SERVO_MIN_PULSE + (SERVO_MAX_PULSE - SERVO_MIN_PULSE) * (angle_deg / 180.0f);
    return static_cast<uint16_t>(pulse);
}

void ServoController::writePWM(uint8_t device_addr, uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t data[5] = {
        reg,
        (uint8_t)(on & 0xFF),
        (uint8_t)(on >> 8),
        (uint8_t)(off & 0xFF),
        (uint8_t)(off >> 8),
    };
    i2c_master_write_to_device(I2C_PORT, device_addr, data, 5, 100 / portTICK_PERIOD_MS);
}
