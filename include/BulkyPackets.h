#pragma once

#include <array>
#include <cstdint>

namespace Comm {

constexpr uint32_t kPacketMagic = 0x42554C4B; // "BULK"
constexpr uint8_t kProtocolVersion = 1;
constexpr size_t kMaxNameLength = 32;
constexpr size_t kMaxPlatformLength = 16;
constexpr size_t kMaxPeers = 5;

enum class MessageType : uint8_t {
  ScanRequest = 1,
  DroneIdentity = 2,
  IliteIdentity = 3,
  DroneAck = 4,
  Control = 10,
  Feedback = 11,
};

struct PacketHeader {
  uint32_t magic = kPacketMagic;
  uint8_t version = kProtocolVersion;
  MessageType type = MessageType::ScanRequest;
  uint8_t reserved = 0;
} __attribute__((packed));

struct DiscoveryPacket {
  PacketHeader header;
  std::array<uint8_t, 6> mac{};
  char name[kMaxNameLength];
  char platform[kMaxPlatformLength];
} __attribute__((packed));

struct ControlPayload {
  uint8_t motion = 0;
  uint8_t speed = 0;
  uint8_t pump = 0;
  uint8_t flash = 0;
  uint8_t buzzer = 0;
  uint8_t cameraMode = 1;
  uint8_t cameraYaw = 90;
  uint8_t cameraPitch = 90;
  uint8_t craneYaw = 90;
  uint8_t cranePitch = 0;
} __attribute__((packed));

struct ControlPacket {
  PacketHeader header{ kPacketMagic, kProtocolVersion, MessageType::Control, 0 };
  ControlPayload payload;
} __attribute__((packed));

struct FeedbackPayload {
  uint8_t telemetryFlags = 0;
  uint16_t batteryMv = 0;
  uint16_t motorRpmLeft = 0;
  uint16_t motorRpmRight = 0;
  uint8_t reserved[8]{};
} __attribute__((packed));

struct FeedbackPacket {
  PacketHeader header{ kPacketMagic, kProtocolVersion, MessageType::Feedback, 0 };
  FeedbackPayload payload;
} __attribute__((packed));

inline bool validateHeader(const PacketHeader &header, MessageType expected) {
  return header.magic == kPacketMagic && header.version == kProtocolVersion && header.type == expected;
}

inline ControlPayload encodeControlPayload(uint8_t motion,
                                          uint8_t speed,
                                          bool pump,
                                          bool flash,
                                          bool buzzer,
                                          bool cameraMode,
                                          uint8_t cameraYaw,
                                          uint8_t cameraPitch,
                                          uint8_t craneYaw,
                                          uint8_t cranePitch) {
  ControlPayload payload;
  payload.motion = motion;
  payload.speed = speed;
  payload.pump = pump ? 1 : 0;
  payload.flash = flash ? 1 : 0;
  payload.buzzer = buzzer ? 1 : 0;
  payload.cameraMode = cameraMode ? 1 : 0;
  payload.cameraYaw = cameraYaw;
  payload.cameraPitch = cameraPitch;
  payload.craneYaw = craneYaw;
  payload.cranePitch = cranePitch;
  return payload;
}

}  // namespace Comm
