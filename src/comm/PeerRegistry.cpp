#include "comm/PeerRegistry.h"

#include <algorithm>

namespace Comm {

namespace {
PeerInfo makePeerCopy(const PeerInfo &peer) {
  PeerInfo copy = peer;
  return copy;
}
}

PeerRegistry::PeerRegistry() = default;

void PeerRegistry::begin() {
  portENTER_CRITICAL(&mutex_);
  peers_.clear();
  target_.reset();
  while (!events_.empty()) {
    events_.pop();
  }
  latestFeedback_.reset();
  lastTelemetryMs_ = millis();
  linkState_ = LinkState::Idle;
  portEXIT_CRITICAL(&mutex_);
}

void PeerRegistry::markScanActive(bool active) {
  portENTER_CRITICAL(&mutex_);
  if (active) {
    linkState_ = LinkState::Scanning;
  } else if (!target_.has_value()) {
    linkState_ = LinkState::Idle;
  }
  portEXIT_CRITICAL(&mutex_);

  PeerInfo emptyPeer;
  enqueueEvent(active ? PeerEvent::Type::ScanStarted : PeerEvent::Type::ScanStopped, emptyPeer);
}

void PeerRegistry::upsertPeer(const PeerInfo &peer, bool triggerAck) {
  portENTER_CRITICAL(&mutex_);
  int index = findPeerIndex(peer.mac);
  PeerInfo peerCopy = makePeerCopy(peer);
  if (index >= 0) {
    peers_[index] = peerCopy;
  } else {
    if (peers_.size() >= kMaxPeers) {
      peers_.erase(peers_.begin());
    }
    peers_.push_back(peerCopy);
  }
  portEXIT_CRITICAL(&mutex_);

  enqueueEvent(index >= 0 ? PeerEvent::Type::PeerUpdated : PeerEvent::Type::PeerFound, peerCopy);
  if (triggerAck) {
    enqueueEvent(PeerEvent::Type::PeerAcked, peerCopy);
  }
}

void PeerRegistry::markPeerLost(const std::array<uint8_t, 6> &mac) {
  portENTER_CRITICAL(&mutex_);
  int index = findPeerIndex(mac);
  PeerInfo peerCopy;
  if (index >= 0) {
    peerCopy = peers_[index];
    peers_.erase(peers_.begin() + index);
  }
  bool targetCleared = false;
  if (target_.has_value() && target_.value() == mac) {
    target_.reset();
    linkState_ = LinkState::Lost;
    targetCleared = true;
  }
  portEXIT_CRITICAL(&mutex_);

  if (index >= 0) {
    enqueueEvent(PeerEvent::Type::PeerLost, peerCopy);
  }
  if (targetCleared) {
    enqueueEvent(PeerEvent::Type::TargetCleared, peerCopy);
  }
}

void PeerRegistry::selectPeer(const std::array<uint8_t, 6> &mac) {
  portENTER_CRITICAL(&mutex_);
  target_ = mac;
  linkState_ = LinkState::Paired;
  portEXIT_CRITICAL(&mutex_);

  PeerInfo peerInfo;
  auto maybePeer = getPeer(mac);
  if (maybePeer.has_value()) {
    peerInfo = maybePeer.value();
  } else {
    peerInfo.mac = mac;
  }
  enqueueEvent(PeerEvent::Type::TargetSelected, peerInfo);
}

void PeerRegistry::clearTarget() {
  std::optional<std::array<uint8_t, 6>> previous;
  portENTER_CRITICAL(&mutex_);
  previous = target_;
  target_.reset();
  linkState_ = LinkState::Lost;
  portEXIT_CRITICAL(&mutex_);

  if (previous.has_value()) {
    PeerInfo peerInfo;
    auto maybePeer = getPeer(previous.value());
    if (maybePeer.has_value()) {
      peerInfo = maybePeer.value();
    } else {
      peerInfo.mac = previous.value();
    }
    enqueueEvent(PeerEvent::Type::TargetCleared, peerInfo);
  }
}

bool PeerRegistry::hasTarget() const {
  portENTER_CRITICAL(&mutex_);
  bool hasTarget = target_.has_value();
  portEXIT_CRITICAL(&mutex_);
  return hasTarget;
}

std::optional<std::array<uint8_t, 6>> PeerRegistry::getTarget() const {
  portENTER_CRITICAL(&mutex_);
  auto target = target_;
  portEXIT_CRITICAL(&mutex_);
  return target;
}

std::optional<PeerInfo> PeerRegistry::getPeer(const std::array<uint8_t, 6> &mac) const {
  portENTER_CRITICAL(&mutex_);
  int index = findPeerIndex(mac);
  std::optional<PeerInfo> peer;
  if (index >= 0) {
    peer = peers_[index];
  }
  portEXIT_CRITICAL(&mutex_);
  return peer;
}

void PeerRegistry::pushFeedback(const FeedbackPacket &feedback) {
  portENTER_CRITICAL(&mutex_);
  latestFeedback_ = feedback;
  lastTelemetryMs_ = millis();
  portEXIT_CRITICAL(&mutex_);

  PeerInfo peer;
  if (target_.has_value()) {
    auto maybePeer = getPeer(target_.value());
    if (maybePeer.has_value()) {
      peer = maybePeer.value();
    }
  }
  enqueueEvent(PeerEvent::Type::TelemetryReceived, peer);
}

std::optional<FeedbackPacket> PeerRegistry::getLatestFeedback() {
  portENTER_CRITICAL(&mutex_);
  auto feedback = latestFeedback_;
  portEXIT_CRITICAL(&mutex_);
  return feedback;
}

void PeerRegistry::touchTelemetry() {
  portENTER_CRITICAL(&mutex_);
  lastTelemetryMs_ = millis();
  portEXIT_CRITICAL(&mutex_);
}

bool PeerRegistry::isTelemetryTimedOut(uint32_t nowMs, uint32_t timeoutMs) const {
  portENTER_CRITICAL(&mutex_);
  uint32_t last = lastTelemetryMs_;
  portEXIT_CRITICAL(&mutex_);
  return nowMs - last > timeoutMs;
}

void PeerRegistry::notifyTelemetryTimeout() {
  PeerInfo peer;
  if (target_.has_value()) {
    auto maybePeer = getPeer(target_.value());
    if (maybePeer.has_value()) {
      peer = maybePeer.value();
    } else {
      peer.mac = target_.value();
    }
  }
  enqueueEvent(PeerEvent::Type::TelemetryTimeout, peer);
}

uint32_t PeerRegistry::lastTelemetryMs() const {
  portENTER_CRITICAL(&mutex_);
  uint32_t last = lastTelemetryMs_;
  portEXIT_CRITICAL(&mutex_);
  return last;
}

void PeerRegistry::setLinkState(LinkState state) {
  portENTER_CRITICAL(&mutex_);
  linkState_ = state;
  portEXIT_CRITICAL(&mutex_);
}

LinkState PeerRegistry::getLinkState() const {
  portENTER_CRITICAL(&mutex_);
  LinkState state = linkState_;
  portEXIT_CRITICAL(&mutex_);
  return state;
}

bool PeerRegistry::popEvent(PeerEvent &event) {
  portENTER_CRITICAL(&mutex_);
  bool hasEvent = !events_.empty();
  if (hasEvent) {
    event = events_.front();
    events_.pop();
  }
  portEXIT_CRITICAL(&mutex_);
  return hasEvent;
}

std::vector<PeerInfo> PeerRegistry::peers() const {
  portENTER_CRITICAL(&mutex_);
  auto peersCopy = peers_;
  portEXIT_CRITICAL(&mutex_);
  return peersCopy;
}

int PeerRegistry::findPeerIndex(const std::array<uint8_t, 6> &mac) const {
  for (size_t i = 0; i < peers_.size(); ++i) {
    if (peers_[i].mac == mac) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void PeerRegistry::enqueueEvent(PeerEvent::Type type, const PeerInfo &peer) {
  portENTER_CRITICAL(&mutex_);
  events_.push(PeerEvent{type, peer});
  portEXIT_CRITICAL(&mutex_);
}

}  // namespace Comm
