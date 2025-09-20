#pragma once

#include <Arduino.h>
#include <array>

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
  bool linkReady;
  std::array<uint8_t, 6> targetMac;
  uint32_t lastTelemetryMs;
};

extern ControlState controlState;

void resetControlState();
void updateControlFromComms();
void updateBuzzerOutput();
void action();
