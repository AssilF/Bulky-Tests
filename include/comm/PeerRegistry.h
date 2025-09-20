#pragma once

#include <Arduino.h>
#include <array>
#include <functional>
#include <optional>
#include <queue>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#include "comm/BulkyPackets.h"

namespace Comm {

enum class LinkState : uint8_t {
  Idle,
  Scanning,
  Paired,
  Lost,
};

struct PeerInfo {
  std::array<uint8_t, 6> mac{};
  String name;
  String platform;
  uint32_t lastSeenMs = 0;
  bool acknowledged = false;
};

struct PeerEvent {
  enum class Type : uint8_t {
    ScanStarted,
    ScanStopped,
    PeerFound,
    PeerUpdated,
    PeerLost,
    PeerAcked,
    TargetSelected,
    TargetCleared,
    TelemetryReceived,
    TelemetryTimeout,
  } type;
  PeerInfo peer;
};

class PeerRegistry {
 public:
  PeerRegistry();

  void begin();

  void markScanActive(bool active);
  void upsertPeer(const PeerInfo &peer, bool triggerAck);
  void markPeerLost(const std::array<uint8_t, 6> &mac);
  void selectPeer(const std::array<uint8_t, 6> &mac);
  void clearTarget();

  bool hasTarget() const;
  std::optional<std::array<uint8_t, 6>> getTarget() const;
  std::optional<PeerInfo> getPeer(const std::array<uint8_t, 6> &mac) const;

  void pushFeedback(const FeedbackPacket &feedback);
  std::optional<FeedbackPacket> getLatestFeedback();

  void touchTelemetry();
  bool isTelemetryTimedOut(uint32_t nowMs, uint32_t timeoutMs) const;
  void notifyTelemetryTimeout();
  uint32_t lastTelemetryMs() const;

  void setLinkState(LinkState state);
  LinkState getLinkState() const;

  bool popEvent(PeerEvent &event);

  std::vector<PeerInfo> peers() const;

 private:
  int findPeerIndex(const std::array<uint8_t, 6> &mac) const;
  void enqueueEvent(PeerEvent::Type type, const PeerInfo &peer);

  std::vector<PeerInfo> peers_;
  std::optional<std::array<uint8_t, 6>> target_;
  mutable portMUX_TYPE mutex_ = portMUX_INITIALIZER_UNLOCKED;
  std::queue<PeerEvent> events_;
  std::optional<FeedbackPacket> latestFeedback_;
  uint32_t lastTelemetryMs_ = 0;
  LinkState linkState_ = LinkState::Idle;
};

}  // namespace Comm
