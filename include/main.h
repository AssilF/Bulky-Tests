#pragma once

#include <Arduino.h>

#include "comms.h"

struct ControlState {
  byte motion;
  byte speed;
  bool pump;
  bool flash;
  bool buzzer;
  bool cameraMode;
  uint8_t cameraYaw;
  uint8_t cameraPitch;
  uint8_t craneYaw;
  uint8_t cranePitch;
};

extern ControlState controlState;

void resetControlState();
void applyCommand(const Comms::ControlPacket &cmd);
void updateControlFromComms();
void initOTA();
void updatePairingFeedback();
void updateIliteBroadcastFeedback();
void updateBuzzerOutput();
void action();
