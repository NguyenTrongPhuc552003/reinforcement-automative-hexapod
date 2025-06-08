// main/main.cpp

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hexapod.hpp"
#include "gait_controller.hpp"
#include "wifi_comm.hpp"
#include "config.hpp"
#include "servo_controller.hpp"
#include <chrono>

static const char* TAG = "Main";

static Hexapod hexapod;
static GaitController gaitController(GaitController::GaitType::TRIPOD);
static WifiComm wifiComm;

static void command_handler(const std::string& cmd) {
    if (cmd == "walk") {
        gaitController.setMode(GaitController::Mode::WALK);
    } else if (cmd == "stand") {
        gaitController.setMode(GaitController::Mode::STAND);
    } else if (cmd == "left") {
        gaitController.setDirection(GaitController::Direction::LEFT);
    } else if (cmd == "right") {
        gaitController.setDirection(GaitController::Direction::RIGHT);
    } else if (cmd == "forward") {
        gaitController.setDirection(GaitController::Direction::FORWARD);
    } else if (cmd == "backward") {
        gaitController.setDirection(GaitController::Direction::BACKWARD);
    }
}

extern "C" void app_main(void) {
    // Initialize components
    ServoController::init();
    hexapod.init();
    gaitController.attachHexapod(&hexapod);

    // Setup WiFi communication
    wifiComm.begin(Config::WIFI_SSID, Config::WIFI_PASS);
    wifiComm.onCommand(command_handler);

    ESP_LOGI(TAG, "Hexapod initialized and ready");

    auto last_update = std::chrono::high_resolution_clock::now();
    
    while (true) {
        auto now = std::chrono::high_resolution_clock::now();
        float delta = std::chrono::duration<float>(now - last_update).count();
        last_update = now;

        gaitController.update(delta);
        wifiComm.loop();
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
