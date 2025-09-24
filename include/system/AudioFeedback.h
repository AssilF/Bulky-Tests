#pragma once

#include <Arduino.h>
#include <cstddef>
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

class AudioFeedback {
 public:
  struct Segment {
    uint16_t frequency;
    uint16_t durationMs;
    uint16_t pauseMs;
  };

  enum class Pattern : uint8_t {
    ScanStart,
    ScanStop,
    PeerFound,
    PeerAck,
    TargetSelected,
    TargetCleared,
    TelemetryTimeout,
    PairingPulse,
    MovementStart,
    MovementStop,
    PeerConnected,
    PeerDisconnected,
    PacketReceived,
  };

  explicit AudioFeedback(std::function<void(uint16_t)> toneWriter);
  AudioFeedback(const AudioFeedback &) = delete;
  AudioFeedback &operator=(const AudioFeedback &) = delete;

  void playPattern(Pattern pattern);
  void loop(uint32_t nowMs);
  void stop();
  bool isActive() const;

 private:
  static constexpr size_t kQueueCapacity = 16;

  void enqueuePattern(Pattern pattern);
  void startNextSegment(uint32_t nowMs);

  std::function<void(uint16_t)> toneWriter_;
  Segment queueBuffer_[kQueueCapacity] = {};
  size_t queueHead_ = 0;
  size_t queueTail_ = 0;
  size_t queueSize_ = 0;
  mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  bool hasActiveSegment_ = false;
  bool inPause_ = false;
  Segment activeSegment_{};
  uint32_t segmentStartMs_ = 0;
  uint32_t pauseStartMs_ = 0;
};
