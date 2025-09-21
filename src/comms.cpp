#include "comms.h"

#include <cstring>

namespace {
constexpr uint8_t BroadcastMac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

void logMac(const uint8_t *mac) {
  for (int i = 0; i < 6; ++i) {
    if (i != 0) {
      Serial.print(":");
    }
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
  }
}

void defaultRecvCallback(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.print("[COMMS] ESP-NOW packet from ");
  if (mac != nullptr) {
    logMac(mac);
  } else {
    Serial.print("<unknown>");
  }
  Serial.print(" (len=");
  Serial.print(len);
  Serial.println(")");

  if (incomingData != nullptr && len > 0) {
    Serial.print("[COMMS] Payload: ");
    for (int i = 0; i < len; ++i) {
      if (i != 0) {
        Serial.print(" ");
      }
      uint8_t byteVal = incomingData[i];
      if (byteVal < 16) {
        Serial.print("0");
      }
      Serial.print(byteVal, HEX);
    }
    Serial.println();
  }
}

}  // namespace

namespace Comms {

namespace {
esp_now_recv_cb_t g_recvCallback = nullptr;
}

static bool initInternal(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback) {
  (void)tcpPort;

  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_18_5dBm);
  if (!WiFi.softAP(ssid, password)) {
    return false;
  }

  if (esp_now_init() != ESP_OK) {
    return false;
  }

  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, BroadcastMac, sizeof(BroadcastMac));
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(BroadcastMac)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      return false;
    }
  }

  g_recvCallback = recvCallback != nullptr ? recvCallback : defaultRecvCallback;
  esp_now_register_recv_cb(g_recvCallback);

  return true;
}

bool init(const char *ssid, const char *password, int tcpPort) {
  return initInternal(ssid, password, tcpPort, nullptr);
}

bool init(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback) {
  return initInternal(ssid, password, tcpPort, recvCallback);
}

void loop() {}

}  // namespace Comms

