#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

namespace Comms {

constexpr uint32_t PACKET_MAGIC = 0xA1B2C3D4;

struct ThrustCommand {
    uint32_t magic;
    uint16_t throttle;
    int8_t pitchAngle;
    int8_t rollAngle;
    int8_t yawAngle;
    bool arm_motors;
} __attribute__((packed));

struct TelemetryPacket {
    uint32_t magic;
    float pitch;
    float roll;
    float yaw;
    float pitchCorrection;
    float rollCorrection;
    float yawCorrection;
    uint16_t throttle;
    int8_t pitchAngle;
    int8_t rollAngle;
    int8_t yawAngle;
    float verticalAcc;
    uint32_t commandAge;
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

bool receiveCommand(ThrustCommand &cmd);
bool paired();
bool sendTelemetry(const TelemetryPacket &packet);
uint32_t lastCommandTimeMs();
const uint8_t *controllerMac();
extern const uint8_t BroadcastMac[6];

} // namespace Comms

