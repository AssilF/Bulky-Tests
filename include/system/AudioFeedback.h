#pragma once

#include <Arduino.h>
#include <functional>
#include <queue>

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
  };

  explicit AudioFeedback(std::function<void(uint16_t)> toneWriter);

  void playPattern(Pattern pattern);
  void loop(uint32_t nowMs);
  void stop();
  bool isActive() const { return hasActiveSegment_ || !queue_.empty(); }

 private:
  void enqueuePattern(Pattern pattern);
  void startNextSegment(uint32_t nowMs);

  std::function<void(uint16_t)> toneWriter_;
  std::queue<Segment> queue_;
  bool hasActiveSegment_ = false;
  bool inPause_ = false;
  Segment activeSegment_{};
  uint32_t segmentStartMs_ = 0;
  uint32_t pauseStartMs_ = 0;
};
