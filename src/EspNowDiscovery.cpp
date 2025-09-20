#include "EspNowDiscovery.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include <algorithm>
#include <cstring>

namespace {
constexpr uint32_t kScanBroadcastIntervalMs = 750;
constexpr uint32_t kPeerStaleTimeoutMs = 5000;
EspNowDiscovery *g_instance = nullptr;

constexpr uint8_t kIliteScanRequestType = 0x01;
constexpr uint8_t kIliteDroneIdentityType = 0x02;
constexpr uint8_t kIliteControllerIdentityType = 0x03;
constexpr uint8_t kIliteDroneAckType = 0x04;
constexpr size_t kIliteIdentityLength = 16;
constexpr char kIliteControllerIdentity[] = "ILITEA1";

struct IliteIdentityMessage {
  uint8_t type = 0;
  char identity[kIliteIdentityLength]{};
  uint8_t mac[6]{};
} __attribute__((packed));

std::array<uint8_t, 6> broadcastAddress() {
  return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
}

String macToString(const uint8_t *mac) {
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buffer);
}

}  // namespace

EspNowDiscovery::EspNowDiscovery(Comm::PeerRegistry &registry) : registry_(registry) {}

bool EspNowDiscovery::begin() {
  g_instance = this;

  WiFi.mode(WIFI_AP_STA);
  WiFi.setHostname("ILITE-Controller");
  if (!WiFi.softAP("ILITE_CTRL", "ilitepass")) {
    Serial.println("[ESP-NOW] Failed to start SoftAP");
  }
  WiFi.softAPsetHostname("ILITE-Controller");

  WiFi.macAddress(controllerMac_.data());
  bool macValid = false;
  for (auto byte : controllerMac_) {
    if (byte != 0) {
      macValid = true;
      break;
    }
  }
  if (!macValid) {
    Serial.println("[ESP-NOW] Failed to read controller MAC");
    return false;
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Failed to init ESP-NOW");
    return false;
  }

  esp_now_register_recv_cb(&EspNowDiscovery::onReceiveStatic);
  esp_now_register_send_cb(&EspNowDiscovery::onSentStatic);

  registry_.begin();
  startScan();

  if (taskHandle_ == nullptr) {
    BaseType_t created = xTaskCreatePinnedToCore(&EspNowDiscovery::taskShim, "espnow-discovery", 4096, this, 2, &taskHandle_, 1);
    if (created != pdPASS) {
      Serial.println("[ESP-NOW] Failed to create discovery task");
      return false;
    }
  }

  Serial.print("[ESP-NOW] Controller MAC: ");
  Serial.println(macToString(controllerMac_.data()));

  return true;
}

void EspNowDiscovery::startScan() {
  portENTER_CRITICAL(&stateMutex_);
  scanning_ = true;
  lastScanBroadcastMs_ = 0;
  portEXIT_CRITICAL(&stateMutex_);
  registry_.markScanActive(true);
}

void EspNowDiscovery::stopScan() {
  portENTER_CRITICAL(&stateMutex_);
  scanning_ = false;
  portEXIT_CRITICAL(&stateMutex_);
  registry_.markScanActive(false);
}

bool EspNowDiscovery::isScanning() const {
  portENTER_CRITICAL(&stateMutex_);
  bool scanning = scanning_;
  portEXIT_CRITICAL(&stateMutex_);
  return scanning;
}

bool EspNowDiscovery::sendControl(const Comm::ControlPacket &packet) {
  if (!hasTarget_) {
    return false;
  }
  esp_err_t status = esp_now_send(targetMac_.data(), reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  return status == ESP_OK;
}

void EspNowDiscovery::requestAckRescan() {
  startScan();
}

void EspNowDiscovery::setTarget(const std::array<uint8_t, 6> &mac) {
  memcpy(targetMac_.data(), mac.data(), targetMac_.size());
  hasTarget_ = true;
  registry_.selectPeer(mac);
  stopScan();
}

void EspNowDiscovery::clearTarget() {
  hasTarget_ = false;
  registry_.clearTarget();
  startScan();
}

void EspNowDiscovery::onReceiveStatic(const uint8_t *mac, const uint8_t *data, int len) {
  if (g_instance != nullptr) {
    g_instance->handleIncoming(mac, data, len);
  }
}

void EspNowDiscovery::onSentStatic(const uint8_t *mac, esp_now_send_status_t status) {
  (void)mac;
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.print("[ESP-NOW] Send failed: ");
    Serial.println(status);
  }
}

void EspNowDiscovery::taskShim(void *param) {
  auto *self = static_cast<EspNowDiscovery *>(param);
  self->taskLoop();
}

void EspNowDiscovery::taskLoop() {
  for (;;) {
    uint32_t now = millis();
    bool shouldScan = false;
    portENTER_CRITICAL(&stateMutex_);
    shouldScan = scanning_;
    portEXIT_CRITICAL(&stateMutex_);

    if (shouldScan && now - lastScanBroadcastMs_ > kScanBroadcastIntervalMs) {
      sendScanRequest();
      lastScanBroadcastMs_ = now;
    }

    if (hasTarget_ && registry_.isTelemetryTimedOut(now, 3000)) {
      registry_.notifyTelemetryTimeout();
      clearTarget();
    }

    auto snapshot = registry_.peers();
    for (const auto &peer : snapshot) {
      if (now - peer.lastSeenMs > kPeerStaleTimeoutMs) {
        registry_.markPeerLost(peer.mac);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void EspNowDiscovery::handleIncoming(const uint8_t *mac, const uint8_t *data, int len) {
  if (handleIliteMessage(mac, data, len)) {
    return;
  }

  if (len < static_cast<int>(sizeof(Comm::PacketHeader))) {
    return;
  }

  auto *header = reinterpret_cast<const Comm::PacketHeader *>(data);
  if (header->magic != Comm::kPacketMagic || header->version != Comm::kProtocolVersion) {
    return;
  }

  switch (header->type) {
    case Comm::MessageType::DroneIdentity:
    case Comm::MessageType::IliteIdentity:
    case Comm::MessageType::DroneAck: {
      if (len < static_cast<int>(sizeof(Comm::DiscoveryPacket))) {
        return;
      }
      Comm::DiscoveryPacket packet;
      memcpy(&packet, data, sizeof(packet));
      handleDiscoveryPacket(mac, packet);
      break;
    }
    case Comm::MessageType::Feedback: {
      if (len < static_cast<int>(sizeof(Comm::FeedbackPacket))) {
        return;
      }
      Comm::FeedbackPacket packet;
      memcpy(&packet, data, sizeof(packet));
      handleFeedbackPacket(packet);
      break;
    }
    default:
      break;
  }
}

void EspNowDiscovery::handleDiscoveryPacket(const uint8_t *mac, const Comm::DiscoveryPacket &packet) {
  Comm::PeerInfo peer;
  peer.mac = packet.mac;
  peer.name = String(packet.name);
  peer.platform = String(packet.platform);
  peer.lastSeenMs = millis();
  peer.acknowledged = packet.header.type == Comm::MessageType::DroneAck;
  registry_.upsertPeer(peer, peer.acknowledged);

  if (packet.header.type == Comm::MessageType::DroneIdentity) {
    if (ensurePeer(peer.mac)) {
      sendIliteIdentity(peer.mac, peer.name);
    }
  } else if (packet.header.type == Comm::MessageType::DroneAck) {
    if (!hasTarget_) {
      setTarget(peer.mac);
    }
  }

  (void)mac;
}

void EspNowDiscovery::handleFeedbackPacket(const Comm::FeedbackPacket &packet) {
  registry_.pushFeedback(packet);
  registry_.touchTelemetry();
}

void EspNowDiscovery::sendScanRequest() {
  Comm::DiscoveryPacket packet;
  packet.header.magic = Comm::kPacketMagic;
  packet.header.version = Comm::kProtocolVersion;
  packet.header.type = Comm::MessageType::ScanRequest;
  packet.mac = controllerMac_;
  strlcpy(packet.name, "ILITE_CTRL", sizeof(packet.name));
  strlcpy(packet.platform, "Controller", sizeof(packet.platform));

  auto broadcast = broadcastAddress();
  esp_err_t status = esp_now_send(broadcast.data(), reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  if (status != ESP_OK) {
    Serial.print("[ESP-NOW] Scan request failed: ");
    Serial.println(status);
  }

  IliteIdentityMessage iliteRequest;
  iliteRequest.type = kIliteScanRequestType;
  memcpy(iliteRequest.mac, controllerMac_.data(), controllerMac_.size());
  status = esp_now_send(broadcast.data(), reinterpret_cast<uint8_t *>(&iliteRequest), sizeof(iliteRequest));
  if (status != ESP_OK) {
    Serial.print("[ESP-NOW] ILITE scan request failed: ");
    Serial.println(status);
  }
}

void EspNowDiscovery::sendIliteIdentity(const std::array<uint8_t, 6> &mac, const String &droneName) {
  IliteIdentityMessage iliteIdentity;
  iliteIdentity.type = kIliteControllerIdentityType;
  strlcpy(iliteIdentity.identity, kIliteControllerIdentity, sizeof(iliteIdentity.identity));
  memcpy(iliteIdentity.mac, controllerMac_.data(), controllerMac_.size());

  esp_err_t status = esp_now_send(mac.data(), reinterpret_cast<uint8_t *>(&iliteIdentity), sizeof(iliteIdentity));
  if (status != ESP_OK) {
    Serial.print("[ESP-NOW] Failed to send ILITE identity to ");
    Serial.println(droneName);
  }

  Comm::DiscoveryPacket packet;
  packet.header.magic = Comm::kPacketMagic;
  packet.header.version = Comm::kProtocolVersion;
  packet.header.type = Comm::MessageType::IliteIdentity;
  packet.mac = controllerMac_;
  strlcpy(packet.name, "ILITEA1", sizeof(packet.name));
  strlcpy(packet.platform, "Controller", sizeof(packet.platform));

  status = esp_now_send(mac.data(), reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  if (status != ESP_OK) {
    Serial.print("[ESP-NOW] Failed to send ILITE identity to ");
    Serial.println(droneName);
  }
}

bool EspNowDiscovery::ensurePeer(const std::array<uint8_t, 6> &mac) {
  if (esp_now_is_peer_exist(mac.data())) {
    return true;
  }

  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, mac.data(), mac.size());
  peerInfo.channel = 0;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result != ESP_OK) {
    Serial.print("[ESP-NOW] Failed to add peer: ");
    Serial.println(result);
    return false;
  }
  return true;
}

bool EspNowDiscovery::handleIliteMessage(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != static_cast<int>(sizeof(IliteIdentityMessage))) {
    return false;
  }

  auto broadcast = broadcastAddress();
  if (memcmp(mac, broadcast.data(), broadcast.size()) == 0) {
    return false;
  }
  if (memcmp(mac, controllerMac_.data(), controllerMac_.size()) == 0) {
    return false;
  }

  const auto *message = reinterpret_cast<const IliteIdentityMessage *>(data);
  if (message->type != kIliteDroneIdentityType && message->type != kIliteDroneAckType) {
    return false;
  }

  std::array<uint8_t, 6> droneMac{};
  memcpy(droneMac.data(), message->mac, droneMac.size());
  bool macPopulated = false;
  for (auto byte : droneMac) {
    if (byte != 0) {
      macPopulated = true;
      break;
    }
  }
  if (!macPopulated) {
    memcpy(droneMac.data(), mac, droneMac.size());
  }

  if (message->type == kIliteDroneIdentityType) {
    Comm::PeerInfo peer;
    peer.mac = droneMac;
    peer.name = String(message->identity);
    peer.platform = String("ILITE");
    peer.lastSeenMs = millis();
    peer.acknowledged = false;
    registry_.upsertPeer(peer, false);

    if (ensurePeer(droneMac)) {
      sendIliteIdentity(droneMac, peer.name);
    }
    return true;
  }

  if (message->type == kIliteDroneAckType) {
    Comm::PeerInfo peer;
    peer.mac = droneMac;
    peer.lastSeenMs = millis();
    peer.acknowledged = true;
    peer.platform = String("ILITE");

    auto existing = registry_.getPeer(droneMac);
    if (existing.has_value()) {
      peer.name = existing->name;
      peer.platform = existing->platform;
    }
    if (message->identity[0] != '\0') {
      peer.name = String(message->identity);
    }
    if (peer.platform.length() == 0) {
      peer.platform = String("ILITE");
    }

    registry_.upsertPeer(peer, true);
    ensurePeer(droneMac);
    if (!hasTarget_) {
      setTarget(droneMac);
    }
    return true;
  }

  return false;
}
