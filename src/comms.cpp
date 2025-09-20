#include "comms.h"

#include "line.h"
#include "motion.h"
#include "sensors.h"

#include <cstring>

namespace Comms {
namespace {
constexpr float CM_PER_SECOND_CONVERSION = 10.362f;

bool g_paired = false;
ControlPacket g_lastCommand{};
uint8_t g_controllerMac[6] = {0};
uint32_t g_lastCommandTime = 0;
uint32_t g_lastAckTime = 0;
uint32_t g_lastIliteIdentityTime = 0;
esp_now_recv_cb_t g_externalRecvCallback = nullptr;

bool isBroadcastMac(const uint8_t *mac) {
    if (mac == nullptr) {
        return false;
    }
    for (size_t i = 0; i < 6; ++i) {
        if (mac[i] != 0xff) {
            return false;
        }
    }
    return true;
}

bool macValid(const uint8_t *mac) {
    if (mac == nullptr || isBroadcastMac(mac)) {
        return false;
    }
    for (size_t i = 0; i < 6; ++i) {
        if (mac[i] != 0) {
            return true;
        }
    }
    return false;
}

void copyMac(uint8_t (&dest)[6], const uint8_t *src) {
    if (src == nullptr) {
        return;
    }
    std::memcpy(dest, src, sizeof(dest));
}

void ensurePeer(const uint8_t *mac) {
    if (!macValid(mac)) {
        return;
    }
    if (esp_now_is_peer_exist(mac)) {
        return;
    }

    esp_now_peer_info_t peerInfo{};
    std::memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
}

void setControllerMac(const uint8_t *mac) {
    if (!macValid(mac)) {
        return;
    }

    ensurePeer(mac);
    copyMac(g_controllerMac, mac);
    g_paired = true;
}

void sendIdentityMessage(uint8_t type, const uint8_t *target) {
    IdentityMessage msg{};
    msg.type = type;
    std::strncpy(msg.identity, "Bulky", sizeof(msg.identity) - 1);
    WiFi.macAddress(msg.mac);

    const uint8_t *destination = macValid(target) ? target : BroadcastMac;
    if (macValid(destination)) {
        ensurePeer(destination);
    }
    esp_now_send(destination, reinterpret_cast<const uint8_t *>(&msg), sizeof(msg));
}

void sendAck() {
    if (!macValid(g_controllerMac)) {
        return;
    }
    sendIdentityMessage(DRONE_ACK, g_controllerMac);
    g_lastAckTime = millis();
}

void handleIdentityMessage(const uint8_t *mac, const IdentityMessage *msg) {
    if (msg == nullptr) {
        return;
    }

    const uint8_t *candidateMac = macValid(msg->mac) ? msg->mac : mac;

    switch (msg->type) {
        case ILITE_IDENTITY:
            g_lastIliteIdentityTime = millis();
            setControllerMac(candidateMac);
            sendIdentityMessage(DRONE_IDENTITY, candidateMac);
            sendAck();
            break;
        case SCAN_REQUEST:
            sendIdentityMessage(DRONE_IDENTITY, candidateMac);
            break;
        default:
            break;
    }
}

void handleControlPacket(const uint8_t *mac, const ControlPacket *cmd) {
    if (cmd == nullptr) {
        return;
    }

    setControllerMac(mac);
    g_lastCommand = *cmd;
    g_lastCommandTime = millis();
    sendAck();
}

static void IRAM_ATTR onEspNowDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (mac == nullptr || incomingData == nullptr) {
        return;
    }

    if (g_externalRecvCallback) {
        g_externalRecvCallback(mac, incomingData, len);
    }

    if (len == static_cast<int>(sizeof(IdentityMessage))) {
        const auto *msg = reinterpret_cast<const IdentityMessage *>(incomingData);
        handleIdentityMessage(mac, msg);
        return;
    }

    if (len == static_cast<int>(sizeof(ControlPacket))) {
        const auto *cmd = reinterpret_cast<const ControlPacket *>(incomingData);
        handleControlPacket(mac, cmd);
        return;
    }
}

bool initInternal(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback) {
    (void)tcpPort;

    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    WiFi.softAP(ssid, password);

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    esp_now_peer_info_t peerInfo{};
    std::memcpy(peerInfo.peer_addr, BroadcastMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (!esp_now_is_peer_exist(BroadcastMac)) {
        esp_now_add_peer(&peerInfo);
    }

    g_externalRecvCallback = recvCallback;
    esp_now_register_recv_cb(onEspNowDataRecv);

    g_paired = false;
    std::memset(&g_lastCommand, 0, sizeof(g_lastCommand));
    std::memset(g_controllerMac, 0, sizeof(g_controllerMac));
    g_lastCommandTime = 0;
    g_lastAckTime = 0;
    g_lastIliteIdentityTime = 0;
    resendIndex = 0;
    std::memset(&emission, 0, sizeof(emission));

    return true;
}
} // namespace

TelemetryPacket emission{};
uint8_t resendIndex = 0;

const uint8_t BroadcastMac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

bool init(const char *ssid, const char *password, int tcpPort) {
    return initInternal(ssid, password, tcpPort, nullptr);
}

bool init(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback) {
    return initInternal(ssid, password, tcpPort, recvCallback);
}

bool receiveCommand(ControlPacket &cmd) {
    cmd = g_lastCommand;
    return g_paired;
}

bool paired() {
    return g_paired;
}

void packTelemetry(PacketIndex index, TelemetryPacket &packet) {
    packet = {};
    packet.INDEX = static_cast<uint8_t>(index);
    packet.okIndex = packet.INDEX;

    switch (index) {
        case PACK_TELEMETRY:
            packet.dataByte[0] = linePosition;
            packet.dataByte[1] = front_distance;
            packet.dataByte[2] = bot_distance;
            packet.dataByte[3] = IRBias;
            packet.dataByte[4] = static_cast<int>(average_count * CM_PER_SECOND_CONVERSION);
            packet.dataByte[5] = batteryLevel;
            break;
        case PACK_LINE:
            packet.dataByte[0] = sensor_readings[line_reading1];
            packet.dataByte[1] = sensor_readings[line_reading2];
            packet.dataByte[2] = sensor_readings[line_reading3];
            packet.dataByte[3] = sensor_readings[line_reading4];
            packet.dataByte[4] = lineThresholdsLowers[0];
            packet.dataByte[5] = lineThresholdsLowers[1];
            packet.dataByte[6] = lineThresholdsLowers[2];
            packet.dataByte[7] = lineThresholdsLowers[3];
            break;
        case PACK_PID:
            packet.dataByte[0] = static_cast<int>(kp * 100);
            packet.dataByte[1] = static_cast<int>(kd * 100);
            packet.dataByte[3] = baseSpeed;
            break;
        case PACK_FIRE:
            packet.dataByte[0] = sensor_readings[fire_detection_left];
            packet.dataByte[1] = sensor_readings[fire_detection_right];
            packet.dataByte[3] = fireRange;
            break;
        default:
            break;
    }
}

bool sendTelemetry(const TelemetryPacket &packet) {
    if (!g_paired || !macValid(g_controllerMac)) {
        return false;
    }

    esp_err_t result = esp_now_send(g_controllerMac, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    if (result != ESP_OK) {
        resendIndex = packet.INDEX;
        return false;
    }

    resendIndex = 0;
    return true;
}

bool sendTelemetry(PacketIndex index) {
    packTelemetry(index, emission);
    return sendTelemetry(emission);
}

uint32_t lastCommandTimeMs() {
    return g_lastCommandTime;
}

uint32_t lastPairingAckTimeMs() {
    return g_lastAckTime;
}

const uint8_t *controllerMac() {
    return g_controllerMac;
}

uint32_t lastIliteBroadcastTimeMs() {
    return g_lastIliteIdentityTime;
}
} // namespace Comms