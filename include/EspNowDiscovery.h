#pragma once

#include <Arduino.h>
#include <array>
#include <cstdint>

#include <esp_now.h>
#include <freertos/portmacro.h>

#include "BulkyPackets.h"
#include "PeerRegistry.h"

class EspNowDiscovery {
 public:
  explicit EspNowDiscovery(Comm::PeerRegistry &registry);

  bool begin();
  void startScan();
  void stopScan();
  bool isScanning() const;

  bool sendControl(const Comm::ControlPacket &packet);
  void requestAckRescan();

  void setTarget(const std::array<uint8_t, 6> &mac);
  void clearTarget();

 private:
  static void onReceiveStatic(const uint8_t *mac, const uint8_t *data, int len);
  static void onSentStatic(const uint8_t *mac, esp_now_send_status_t status);
  static void taskShim(void *param);

  void taskLoop();
  void handleIncoming(const uint8_t *mac, const uint8_t *data, int len);
  void handleDiscoveryPacket(const uint8_t *mac, const Comm::DiscoveryPacket &packet);
  void handleFeedbackPacket(const Comm::FeedbackPacket &packet);
  void sendScanRequest();
  void sendIliteIdentity(const std::array<uint8_t, 6> &mac, const String &droneName);

  bool ensurePeer(const std::array<uint8_t, 6> &mac);
  bool handleIliteMessage(const uint8_t *mac, const uint8_t *data, int len);

  Comm::PeerRegistry &registry_;
  std::array<uint8_t, 6> controllerMac_{};
  std::array<uint8_t, 6> targetMac_{};
  bool hasTarget_ = false;
  bool scanning_ = false;
  uint32_t lastScanBroadcastMs_ = 0;
  TaskHandle_t taskHandle_ = nullptr;
  mutable portMUX_TYPE stateMutex_ = portMUX_INITIALIZER_UNLOCKED;
};
