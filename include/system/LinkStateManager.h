#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "comms.h"

struct LinkStateSnapshot {
  bool paired = false;
  Comms::Identity peerIdentity{};
  uint8_t peerMac[6] = {0};
  uint32_t lastActivityMs = 0;
  uint32_t lastCommandTimeMs = 0;
};

struct CommandSnapshot {
  bool hasCommand = false;
  Comms::ControlPacket command{};
  uint32_t timestampMs = 0;
};

class LinkStateManager {
public:
  LinkStateManager();

  void updateStatus(const Comms::LinkStatus &status);
  void updateCommand(const Comms::ControlPacket &packet, uint32_t timestampMs);

  LinkStateSnapshot linkSnapshot() const;
  CommandSnapshot commandSnapshot() const;

  void clearCommand();

private:
  mutable portMUX_TYPE mux_;
  LinkStateSnapshot linkState_{};
  CommandSnapshot commandState_{};
};

