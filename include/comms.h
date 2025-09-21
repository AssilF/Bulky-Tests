#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

namespace Comms {

// Initializes the Wi-Fi interface in AP+STA mode and sets up ESP-NOW.
// The tcpPort argument is ignored but kept for compatibility with the
// existing firmware configuration.
bool init(const char *ssid, const char *password, int tcpPort);

// Optional helper for registering a custom receive callback.
bool init(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback);

// Maintained for compatibility with the previous TCP implementation. ESP-NOW
// is interrupt driven so this function currently does nothing but allows the
// main loop to call it unconditionally.
void loop();

}
