#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

namespace Comms {

struct ControlPacket {
    uint8_t Speed;
    uint8_t MotionState;
    uint8_t pitch;
    uint8_t yaw;
    bool bool1[4];
} __attribute__((packed));

enum PacketIndex : uint8_t {
    PACK_TELEMETRY = 0,
    PACK_LINE = 1,
    PACK_PID = 2,
    PACK_FIRE = 3,
};

struct TelemetryPacket {
    uint8_t INDEX;
    uint8_t statusByte;
    int dataByte[8];
    uint8_t okIndex;
} __attribute__((packed));

enum PairingType : uint8_t {
    SCAN_REQUEST = 0x01,
    DRONE_IDENTITY = 0x02,
    ILITE_IDENTITY = 0x03,
    DRONE_ACK = 0x04,
};

struct IdentityMessage {
    uint8_t type;
    char identity[16];
    uint8_t mac[6];
} __attribute__((packed));

bool init(const char *ssid, const char *password, int tcpPort);
bool init(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback);

bool receiveCommand(ControlPacket &cmd);
bool paired();

void packTelemetry(PacketIndex index, TelemetryPacket &packet);
bool sendTelemetry(const TelemetryPacket &packet);
bool sendTelemetry(PacketIndex index);

uint32_t lastCommandTimeMs();
const uint8_t *controllerMac();

extern const uint8_t BroadcastMac[6];
extern TelemetryPacket emission;
extern uint8_t resendIndex;

} // namespace Comms

