#ifndef WIFI_COMM_HPP
#define WIFI_COMM_HPP

#include <string>
#include <functional>

class WifiComm {
public:
    using CommandCallback = std::function<void(const std::string&)>;

    WifiComm();
    ~WifiComm();

    void begin(const char* ssid, const char* password, int port = 3333);
    void onCommand(CommandCallback callback);
    void loop();

private:
    int _server_sock;
    int _client_sock;
    CommandCallback _callback;

    void setup_wifi(const char* ssid, const char* password);
    void start_server(int port);
    void handle_client();
};

#endif // WIFI_COMM_HPP
