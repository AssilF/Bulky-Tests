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
constexpr AudioFeedback::Segment kPairingPulsePattern[] = {
    {880, 140, 0},
};
constexpr AudioFeedback::Segment kMovementStartPattern[] = {
    {720, 80, 30},
    {1180, 130, 0},
};
constexpr AudioFeedback::Segment kMovementStopPattern[] = {
    {520, 120, 40},
    {320, 160, 0},
};
constexpr AudioFeedback::Segment kPeerConnectedPattern[] = {
    {940, 100, 20},
    {1420, 150, 0},
};
constexpr AudioFeedback::Segment kPeerDisconnectedPattern[] = {
    {620, 140, 20},
    {360, 200, 0},
};
constexpr AudioFeedback::Segment kPacketReceivedPattern[] = {
    {1800, 40, 0},
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
    case AudioFeedback::Pattern::PairingPulse:
      length = sizeof(kPairingPulsePattern) / sizeof(kPairingPulsePattern[0]);
      return kPairingPulsePattern;
    case AudioFeedback::Pattern::MovementStart:
      length = sizeof(kMovementStartPattern) / sizeof(kMovementStartPattern[0]);
      return kMovementStartPattern;
    case AudioFeedback::Pattern::MovementStop:
      length = sizeof(kMovementStopPattern) / sizeof(kMovementStopPattern[0]);
      return kMovementStopPattern;
    case AudioFeedback::Pattern::PeerConnected:
      length = sizeof(kPeerConnectedPattern) / sizeof(kPeerConnectedPattern[0]);
      return kPeerConnectedPattern;
    case AudioFeedback::Pattern::PeerDisconnected:
      length = sizeof(kPeerDisconnectedPattern) / sizeof(kPeerDisconnectedPattern[0]);
      return kPeerDisconnectedPattern;
    case AudioFeedback::Pattern::PacketReceived:
      length = sizeof(kPacketReceivedPattern) / sizeof(kPacketReceivedPattern[0]);
      return kPacketReceivedPattern;
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
  bool queueEmpty = false;
  bool active = false;
  {
    portENTER_CRITICAL(&lock_);
    queueEmpty = (queueSize_ == 0);
    active = hasActiveSegment_;
    portEXIT_CRITICAL(&lock_);
  }

  if (!active && queueEmpty) {
    return;
  }

  if (!active) {
    startNextSegment(nowMs);
  }

  Segment currentSegment{};
  bool inPause = false;
  uint32_t segmentStart = 0;
  uint32_t pauseStart = 0;
  {
    portENTER_CRITICAL(&lock_);
    if (!hasActiveSegment_) {
      portEXIT_CRITICAL(&lock_);
      return;
    }
    currentSegment = activeSegment_;
    inPause = inPause_;
    segmentStart = segmentStartMs_;
    pauseStart = pauseStartMs_;
    portEXIT_CRITICAL(&lock_);
  }

  if (!inPause) {
    if (nowMs - segmentStart >= currentSegment.durationMs) {
      toneWriter_(0);
      bool shouldPause = currentSegment.pauseMs > 0;
      portENTER_CRITICAL(&lock_);
      if (shouldPause) {
        inPause_ = true;
        pauseStartMs_ = nowMs;
      } else {
        hasActiveSegment_ = false;
      }
      portEXIT_CRITICAL(&lock_);
    }
  } else {
    if (nowMs - pauseStart >= currentSegment.pauseMs) {
      portENTER_CRITICAL(&lock_);
      hasActiveSegment_ = false;
      inPause_ = false;
      portEXIT_CRITICAL(&lock_);
    }
  }

  bool shouldStartNext = false;
  {
    portENTER_CRITICAL(&lock_);
    shouldStartNext = !hasActiveSegment_ && queueSize_ > 0;
    portEXIT_CRITICAL(&lock_);
  }
  if (shouldStartNext) {
    startNextSegment(nowMs);
  }
}

void AudioFeedback::stop() {
  portENTER_CRITICAL(&lock_);
  queueHead_ = 0;
  queueTail_ = 0;
  queueSize_ = 0;
  hasActiveSegment_ = false;
  inPause_ = false;
  portEXIT_CRITICAL(&lock_);
  toneWriter_(0);
}

void AudioFeedback::enqueuePattern(Pattern pattern) {
  size_t length = 0;
  const Segment *segments = patternData(pattern, length);
  if (segments == nullptr) {
    return;
  }
  portENTER_CRITICAL(&lock_);
  if (length > kQueueCapacity - queueSize_) {
    portEXIT_CRITICAL(&lock_);
    return;
  }
  for (size_t i = 0; i < length; ++i) {
    queueBuffer_[queueTail_] = segments[i];
    queueTail_ = (queueTail_ + 1) % kQueueCapacity;
  }
  queueSize_ += length;
  portEXIT_CRITICAL(&lock_);
}

void AudioFeedback::startNextSegment(uint32_t nowMs) {
  Segment nextSegment{};
  bool segmentLoaded = false;
  {
    portENTER_CRITICAL(&lock_);
    if (queueSize_ > 0) {
      nextSegment = queueBuffer_[queueHead_];
      queueHead_ = (queueHead_ + 1) % kQueueCapacity;
      --queueSize_;
      activeSegment_ = nextSegment;
      hasActiveSegment_ = true;
      inPause_ = false;
      segmentStartMs_ = nowMs;
      segmentLoaded = true;
    } else {
      hasActiveSegment_ = false;
    }
    portEXIT_CRITICAL(&lock_);
  }

  if (!segmentLoaded) {
    return;
  }

  toneWriter_(nextSegment.frequency);
  if (nextSegment.durationMs == 0) {
    portENTER_CRITICAL(&lock_);
    hasActiveSegment_ = false;
    portEXIT_CRITICAL(&lock_);
  }
}

bool AudioFeedback::isActive() const {
  portENTER_CRITICAL(&lock_);
  bool active = hasActiveSegment_ || queueSize_ > 0;
  portEXIT_CRITICAL(&lock_);
  return active;
}
