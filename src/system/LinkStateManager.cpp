#include "system/LinkStateManager.h"

#include <cstring>

LinkStateManager::LinkStateManager() { mux_ = portMUX_INITIALIZER_UNLOCKED; }

void LinkStateManager::updateStatus(const Comms::LinkStatus &status) {
  portENTER_CRITICAL(&mux_);
  linkState_.paired = status.paired;
  linkState_.peerIdentity = status.peerIdentity;
  memcpy(linkState_.peerMac, status.peerMac, sizeof(linkState_.peerMac));
  linkState_.lastActivityMs = status.lastActivityMs;
  linkState_.lastCommandTimeMs = status.lastCommandMs;
  if (!status.paired) {
    commandState_ = {};
  }
  portEXIT_CRITICAL(&mux_);
}

void LinkStateManager::updateCommand(const Comms::ControlPacket &packet,
                                     uint32_t timestampMs) {
  portENTER_CRITICAL(&mux_);
  commandState_.hasCommand = true;
  commandState_.command = packet;
  commandState_.timestampMs = timestampMs;
  linkState_.lastCommandTimeMs = timestampMs;
  portEXIT_CRITICAL(&mux_);
}

LinkStateSnapshot LinkStateManager::linkSnapshot() const {
  portENTER_CRITICAL(&mux_);
  LinkStateSnapshot copy = linkState_;
  portEXIT_CRITICAL(&mux_);
  return copy;
}

CommandSnapshot LinkStateManager::commandSnapshot() const {
  portENTER_CRITICAL(&mux_);
  CommandSnapshot copy = commandState_;
  portEXIT_CRITICAL(&mux_);
  return copy;
}

void LinkStateManager::clearCommand() {
  portENTER_CRITICAL(&mux_);
  commandState_ = {};
  portEXIT_CRITICAL(&mux_);
}

