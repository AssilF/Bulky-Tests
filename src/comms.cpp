#include "comms.h"

#include "line.h"
#include "motion.h"
#include "sensors.h"

#include <cstring>

extern int operationMode;

namespace Comms {
namespace {
    static_assert(sizeof(ControlPacket) == 8, "ControlPacket must remain 8 bytes");
    static_assert(sizeof(IdentityMessage) == 23, "IdentityMessage must remain 23 bytes");

    struct IliteCommand {
        uint8_t replyIndex;
        int8_t speed;
        uint8_t motionState;
        uint8_t buttonStates[3];
    } __attribute__((packed));

    static_assert(sizeof(IliteCommand) == 6, "IliteCommand must remain 6 bytes");

    constexpr float CM_PER_SECOND_CONVERSION = 10.362f;

    bool g_paired = false;
    ControlPacket g_lastCommand{};
    uint8_t g_controllerMac[6] = {0};
    uint32_t g_lastCommandTime = 0;

    void respondWithIdentity(const uint8_t *mac) {
        IdentityMessage resp{};
        resp.type = DRONE_IDENTITY;
        strncpy(resp.identity, "Bulky", sizeof(resp.identity) - 1);
        WiFi.macAddress(resp.mac);
        esp_now_send(mac, reinterpret_cast<const uint8_t *>(&resp), sizeof(resp));
    }

    void acknowledgeController(const uint8_t *mac) {
        IdentityMessage ack{};
        ack.type = DRONE_ACK;
        esp_now_send(mac, reinterpret_cast<const uint8_t *>(&ack), sizeof(ack));
    }

    void ensurePeerRegistered(const uint8_t *mac) {
        if (esp_now_is_peer_exist(mac)) {
            return;
        }

        esp_now_peer_info_t peerInfo{};
        memcpy(peerInfo.peer_addr, mac, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
    }

    bool controllerMacValid() {
        for (uint8_t byte : g_controllerMac) {
            if (byte != 0) {
                return true;
            }
        }
        return false;
    }

    void handleIliteCommand(const uint8_t *mac, const IliteCommand *cmd) {
        ensurePeerRegistered(mac);

        if (!controllerMacValid()) {
            memcpy(g_controllerMac, mac, 6);
        }

        ControlPacket converted = g_lastCommand;

        int speed = static_cast<int>(cmd->speed);
        if (speed < 0) {
            speed = -speed;
        }
        if (speed > 100) {
            speed = 100;
        }

        converted.Speed = static_cast<uint8_t>(speed);
        converted.MotionState = cmd->motionState;
        converted.bool1[0] = cmd->buttonStates[0] != 0;
        converted.bool1[1] = cmd->buttonStates[1] != 0;
        converted.bool1[2] = cmd->buttonStates[2] != 0;

        g_lastCommand = converted;
        g_lastCommandTime = millis();
        g_paired = true;
    }

    void onDataRecvInternal(const uint8_t *mac, const uint8_t *incomingData, int len) {
        if (len == static_cast<int>(sizeof(IdentityMessage))) {
            const IdentityMessage *msg = reinterpret_cast<const IdentityMessage *>(incomingData);
            if (msg->type == SCAN_REQUEST) {
                respondWithIdentity(mac);
                return;
            }
            if (msg->type == ILITE_IDENTITY) {
                ensurePeerRegistered(mac);
                memcpy(g_controllerMac, mac, 6);
                g_paired = true;
                acknowledgeController(mac);
                return;
            }
        }

        if (len == static_cast<int>(sizeof(ControlPacket))) {
            const ControlPacket *cmd = reinterpret_cast<const ControlPacket *>(incomingData);
            g_lastCommand = *cmd;
            g_lastCommandTime = millis();
            g_paired = true;
            if (!controllerMacValid()) {
                memcpy(g_controllerMac, mac, 6);
            }
            ensurePeerRegistered(mac);
            return;
        }

        if (len == static_cast<int>(sizeof(IliteCommand))) {
            const IliteCommand *cmd = reinterpret_cast<const IliteCommand *>(incomingData);
            handleIliteCommand(mac, cmd);
            return;
        }
    }

    bool initInternal(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback) {
        (void)tcpPort;

        WiFi.mode(WIFI_AP_STA);
        WiFi.setTxPower(WIFI_POWER_8_5dBm);
        WiFi.setSleep(false);
        if (password && std::strlen(password) >= 8) {
            WiFi.softAP(ssid, password);
        } else {
            WiFi.softAP(ssid);
        }

        if (esp_now_init() != ESP_OK) {
            return false;
        }

        esp_now_peer_info_t peerInfo{};
        memcpy(peerInfo.peer_addr, BroadcastMac, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        if (!esp_now_is_peer_exist(BroadcastMac)) {
            esp_now_add_peer(&peerInfo);
        }

        esp_now_register_recv_cb(recvCallback ? recvCallback : onDataRecvInternal);

        g_paired = false;
        memset(g_controllerMac, 0, sizeof(g_controllerMac));
        g_lastCommand = ControlPacket{};
        g_lastCommandTime = 0;
        resendIndex = 0;
        memset(&emission, 0, sizeof(emission));
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
            packet.dataByte[6] = ::operationMode;
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
    if (!g_paired || !controllerMacValid()) {
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

const uint8_t *controllerMac() {
    return g_controllerMac;
}

} // namespace Comms

