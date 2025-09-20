#include "system/AudioFeedback.h"

namespace {
constexpr AudioFeedback::Segment kScanStartPattern[] = {
    {900, 120, 30},
    {1200, 120, 0},
};
constexpr AudioFeedback::Segment kScanStopPattern[] = {
    {800, 80, 20},
    {0, 0, 0},
};
constexpr AudioFeedback::Segment kPeerFoundPattern[] = {
    {1500, 150, 0},
};
constexpr AudioFeedback::Segment kPeerAckPattern[] = {
    {1400, 100, 20},
    {1700, 160, 0},
};
constexpr AudioFeedback::Segment kTargetSelectedPattern[] = {
    {1300, 120, 20},
    {1800, 180, 0},
};
constexpr AudioFeedback::Segment kTargetClearedPattern[] = {
    {900, 160, 30},
    {600, 200, 0},
};
constexpr AudioFeedback::Segment kTelemetryTimeoutPattern[] = {
    {500, 500, 0},
};

const AudioFeedback::Segment *patternData(AudioFeedback::Pattern pattern, size_t &length) {
  switch (pattern) {
    case AudioFeedback::Pattern::ScanStart:
      length = sizeof(kScanStartPattern) / sizeof(kScanStartPattern[0]);
      return kScanStartPattern;
    case AudioFeedback::Pattern::ScanStop:
      length = sizeof(kScanStopPattern) / sizeof(kScanStopPattern[0]);
      return kScanStopPattern;
    case AudioFeedback::Pattern::PeerFound:
      length = sizeof(kPeerFoundPattern) / sizeof(kPeerFoundPattern[0]);
      return kPeerFoundPattern;
    case AudioFeedback::Pattern::PeerAck:
      length = sizeof(kPeerAckPattern) / sizeof(kPeerAckPattern[0]);
      return kPeerAckPattern;
    case AudioFeedback::Pattern::TargetSelected:
      length = sizeof(kTargetSelectedPattern) / sizeof(kTargetSelectedPattern[0]);
      return kTargetSelectedPattern;
    case AudioFeedback::Pattern::TargetCleared:
      length = sizeof(kTargetClearedPattern) / sizeof(kTargetClearedPattern[0]);
      return kTargetClearedPattern;
    case AudioFeedback::Pattern::TelemetryTimeout:
      length = sizeof(kTelemetryTimeoutPattern) / sizeof(kTelemetryTimeoutPattern[0]);
      return kTelemetryTimeoutPattern;
  }
  length = 0;
  return nullptr;
}

}  // namespace

AudioFeedback::AudioFeedback(std::function<void(uint16_t)> toneWriter) : toneWriter_(toneWriter) {}

void AudioFeedback::playPattern(Pattern pattern) {
  enqueuePattern(pattern);
}

void AudioFeedback::loop(uint32_t nowMs) {
  if (!hasActiveSegment_ && queue_.empty()) {
    return;
  }

  if (!hasActiveSegment_) {
    startNextSegment(nowMs);
  }

  if (!hasActiveSegment_) {
    return;
  }

  if (!inPause_) {
    if (nowMs - segmentStartMs_ >= activeSegment_.durationMs) {
      toneWriter_(0);
      inPause_ = activeSegment_.pauseMs > 0;
      if (inPause_) {
        pauseStartMs_ = nowMs;
      } else {
        hasActiveSegment_ = false;
      }
    }
  } else {
    if (nowMs - pauseStartMs_ >= activeSegment_.pauseMs) {
      hasActiveSegment_ = false;
      inPause_ = false;
    }
  }

  if (!hasActiveSegment_ && !queue_.empty()) {
    startNextSegment(nowMs);
  }
}

void AudioFeedback::stop() {
  while (!queue_.empty()) {
    queue_.pop();
  }
  hasActiveSegment_ = false;
  inPause_ = false;
  toneWriter_(0);
}

void AudioFeedback::enqueuePattern(Pattern pattern) {
  size_t length = 0;
  const Segment *segments = patternData(pattern, length);
  if (segments == nullptr) {
    return;
  }
  for (size_t i = 0; i < length; ++i) {
    queue_.push(segments[i]);
  }
}

void AudioFeedback::startNextSegment(uint32_t nowMs) {
  if (queue_.empty()) {
    hasActiveSegment_ = false;
    return;
  }
  activeSegment_ = queue_.front();
  queue_.pop();
  hasActiveSegment_ = true;
  inPause_ = false;
  segmentStartMs_ = nowMs;
  toneWriter_(activeSegment_.frequency);
  if (activeSegment_.durationMs == 0) {
    hasActiveSegment_ = false;
  }
}
