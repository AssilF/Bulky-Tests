#include "comms.h"

#include <Arduino.h>
#include <WiFi.h>

namespace {
WiFiServer *g_server = nullptr;
WiFiClient g_client;
bool g_running = false;
}

namespace Comms {

bool init(const char *ssid, const char *password, int tcpPort) {
  g_client.stop();
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_18_5dBm);
  if (!WiFi.softAP(ssid, password)) {
    g_server = nullptr;
    g_running = false;
    return false;
  }

  static WiFiServer server(tcpPort);
  g_server = &server;
  g_server->begin();
  g_server->setNoDelay(true);
  g_running = true;
  return true;
}

void loop() {
  if (!g_running || g_server == nullptr) {
    return;
  }

  if (!g_client || !g_client.connected()) {
    WiFiClient next = g_server->available();
    if (next) {
      if (g_client) {
        g_client.stop();
      }
      g_client = next;
      Serial.println("[COMMS] TCP client connected");
    }
  }

  if (!g_client || !g_client.connected()) {
    return;
  }

  while (g_client.available()) {
    uint8_t buffer[128];
    size_t toRead = g_client.available();
    if (toRead > sizeof(buffer)) {
      toRead = sizeof(buffer);
    }
    size_t len = g_client.read(buffer, toRead);
    if (len == 0) {
      break;
    }
    g_client.write(buffer, len);
    g_client.flush();
  }
}

}
