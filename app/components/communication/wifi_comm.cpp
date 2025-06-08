#include "wifi_comm.hpp"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include <lwip/sockets.h>
#include <cstring>

static const char* TAG = "WifiComm";

WifiComm::WifiComm() : _server_sock(-1), _client_sock(-1) {}

WifiComm::~WifiComm() {
    if (_client_sock != -1) close(_client_sock);
    if (_server_sock != -1) close(_server_sock);
}

void WifiComm::begin(const char* ssid, const char* password, int port) {
    setup_wifi(ssid, password);
    start_server(port);
}

void WifiComm::onCommand(CommandCallback callback) {
    _callback = callback;
}

void WifiComm::loop() {
    handle_client();
}

void WifiComm::setup_wifi(const char* ssid, const char* password) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

void WifiComm::start_server(int port) {
    struct sockaddr_in server_addr{};
    _server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    bind(_server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
    listen(_server_sock, 1);

    ESP_LOGI(TAG, "Listening on port %d", port);
}

void WifiComm::handle_client() {
    if (_client_sock == -1) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        _client_sock = accept(_server_sock, (struct sockaddr*)&client_addr, &addr_len);
        if (_client_sock != -1) {
            ESP_LOGI(TAG, "Client connected");
        }
    }

    if (_client_sock != -1) {
        char buffer[128];
        int len = recv(_client_sock, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);
        if (len > 0) {
            buffer[len] = 0;
            if (_callback) _callback(std::string(buffer));
        }
    }
}
