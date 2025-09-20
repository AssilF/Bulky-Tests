#include "comms.h"

#include "line.h"
#include "motion.h"
#include "sensors.h"

#include <cctype>
#include <cstring>

#if defined(ESP_PLATFORM)
#include <esp_idf_version.h>
#endif

#include <esp_err.h>

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
    uint32_t g_lastPairingAckTime = 0;
    uint32_t g_lastIliteBroadcastTime = 0;
    bool g_espNowInitialised = false;
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

    bool identityContains(const IdentityMessage &msg, const char *needle) {
        if (needle == nullptr || needle[0] == '\0') {
            return false;
        }
        const size_t maxLen = sizeof(msg.identity);
        const size_t needleLen = std::strlen(needle);
        if (needleLen == 0 || needleLen > maxLen) {
            return false;
        }

        for (size_t start = 0; start + needleLen <= maxLen; ++start) {
            if (msg.identity[start] == '\0') {
                break;
            }

            bool match = true;
            for (size_t i = 0; i < needleLen; ++i) {
                size_t idx = start + i;
                if (idx >= maxLen) {
                    match = false;
                    break;
                }
                char actual = msg.identity[idx];
                if (actual == '\0') {
                    match = false;
                    break;
                }
                char expected = needle[i];
                actual = static_cast<char>(std::toupper(static_cast<unsigned char>(actual)));
                expected = static_cast<char>(std::toupper(static_cast<unsigned char>(expected)));
                if (actual != expected) {
                    match = false;
                    break;
                }
            }

            if (match) {
                return true;
            }
        }
        return false;
    }

    bool isEliteControllerIdentity(const IdentityMessage &msg) {
        if (msg.type == ILITE_IDENTITY || msg.type == ELITE_IDENTITY) {
            return true;
        }
        if (identityContains(msg, "ILITE")) {
            return true;
        }
        if (identityContains(msg, "ELITE")) {
            return true;
        }
        return false;
    }

    bool isIliteIdentity(const IdentityMessage &msg) {
        if (msg.type == ILITE_IDENTITY) {
            return true;
        }
        return identityContains(msg, "ILITE");
    }

    const uint8_t *selectValidMac(const IdentityMessage *msg, const uint8_t *fallback) {
        if (msg && macValid(msg->mac)) {
            return msg->mac;
        }
        if (macValid(fallback)) {
            return fallback;
        }
        return BroadcastMac;
    }

    void ensurePeerRegistered(const uint8_t *mac) {
        if (!macValid(mac)) {
            return;
        }
        if (esp_now_is_peer_exist(mac)) {
            return;
        }

        esp_now_peer_info_t peerInfo{};
        memcpy(peerInfo.peer_addr, mac, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
    }

    void respondWithIdentity(const IdentityMessage *request, const uint8_t *mac) {
        const uint8_t *primary = request ? request->mac : nullptr;
        if (macValid(primary)) {
            ensurePeerRegistered(primary);
        }
        if (macValid(mac)) {
            ensurePeerRegistered(mac);
        }

        const uint8_t *target = selectValidMac(request, mac);

        IdentityMessage resp{};
        resp.type = DRONE_IDENTITY;
        strncpy(resp.identity, "Bulky", sizeof(resp.identity) - 1);
        WiFi.macAddress(resp.mac);
        esp_now_send(target, reinterpret_cast<const uint8_t *>(&resp), sizeof(resp));
    }

    void acknowledgeController(const uint8_t *mac) {
        if (mac == nullptr) {
            return;
        }
        IdentityMessage ack{};
        ack.type = DRONE_ACK;
        strncpy(ack.identity, "Bulky", sizeof(ack.identity) - 1);
        WiFi.macAddress(ack.mac);
        esp_now_send(mac, reinterpret_cast<const uint8_t *>(&ack), sizeof(ack));
    }

    bool controllerMacValid() {
        return macValid(g_controllerMac);
    }

    void handleIliteCommand(const uint8_t *mac, const IliteCommand *cmd) {
        if (mac == nullptr || cmd == nullptr) {
            return;
        }
        ensurePeerRegistered(mac);

        if (!controllerMacValid() || memcmp(g_controllerMac, mac, 6) != 0) {
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

    void IRAM_ATTR onDataRecvInternal(const uint8_t *mac, const uint8_t *incomingData, int len) {
        ledcWriteTone(2,380);
        delay(10);
        Serial.println("Recieved something");
        delay(10);
        if (mac == nullptr || incomingData == nullptr) {
            return;
        }

        if (g_externalRecvCallback) {
            g_externalRecvCallback(mac, incomingData, len);
        }
        if (len == static_cast<int>(sizeof(IdentityMessage))) {
            const IdentityMessage *msg = reinterpret_cast<const IdentityMessage *>(incomingData);
            if (isIliteIdentity(*msg) && !macValid(msg->mac)) {
                g_lastIliteBroadcastTime = millis();
                respondWithIdentity(msg, mac);
                return;
            }
            if (msg->type == SCAN_REQUEST) {
                respondWithIdentity(msg, mac);
                return;
            }
            if (isEliteControllerIdentity(*msg)) {
                const uint8_t *primaryMac = macValid(msg->mac) ? msg->mac : nullptr;
                const uint8_t *fallbackMac = macValid(mac) ? mac : nullptr;
                const uint8_t *targetMac = primaryMac ? primaryMac : fallbackMac;

                if (primaryMac) {
                    ensurePeerRegistered(primaryMac);
                }
                if (fallbackMac) {
                    ensurePeerRegistered(fallbackMac);
                }
                if (targetMac) {
                    if (!controllerMacValid() || memcmp(g_controllerMac, targetMac, 6) != 0) {
                        memcpy(g_controllerMac, targetMac, 6);
                    }
                    g_paired = true;
                    uint32_t now = millis();
                    g_lastCommandTime = now;
                    g_lastPairingAckTime = now;
                    acknowledgeController(targetMac);
                }
                return;
            }
        }

        if (len == static_cast<int>(sizeof(ControlPacket))) {
            const ControlPacket *cmd = reinterpret_cast<const ControlPacket *>(incomingData);
            g_lastCommand = *cmd;
            g_lastCommandTime = millis();
            g_paired = true;
            if (macValid(mac) && (!controllerMacValid() || memcmp(g_controllerMac, mac, 6) != 0)) {
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

#if defined(ESP_IDF_VERSION) && defined(ESP_IDF_VERSION_VAL)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
    void IRAM_ATTR onEspNowDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
        const uint8_t *mac = info ? info->src_addr : nullptr;
        onDataRecvInternal(mac, incomingData, len);
    }
#else
    void IRAM_ATTR onEspNowDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
        onDataRecvInternal(mac, incomingData, len);
    }
#endif
#else
    void IRAM_ATTR onEspNowDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
        onDataRecvInternal(mac, incomingData, len);
    }
#endif

    bool initInternal(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback) {
        (void)tcpPort;
        Serial.println("Initialising Comms");
            delay(10);
        WiFi.mode(WIFI_AP_STA);
            delay(10);
        WiFi.setSleep(false);
        WiFi.softAP(ssid, password);
            delay(10);
        WiFi.setTxPower(WIFI_POWER_19_5dBm);
            delay(100);

        if (g_espNowInitialised) {
            esp_now_register_recv_cb(nullptr);
            esp_now_deinit();
            g_espNowInitialised = false;
            delay(10);
        }

        esp_err_t initResult = esp_now_init();
        if (initResult != ESP_OK) {
            Serial.printf("ESP-NOW init failed: %s (%d)\n", esp_err_to_name(initResult), static_cast<int>(initResult));
            return false;
        }
        delay(10);
        g_espNowInitialised = true;

        esp_now_peer_info_t peerInfo{};
        memcpy(peerInfo.peer_addr, BroadcastMac, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        if (!esp_now_is_peer_exist(BroadcastMac)) {
            esp_now_add_peer(&peerInfo);
        }

        g_externalRecvCallback = recvCallback;

        esp_err_t registerResult = esp_now_register_recv_cb(&onEspNowDataRecv);
        if (registerResult != ESP_OK) {
            Serial.printf("Failed to register ESP-NOW recv callback: %s (%d)\n", esp_err_to_name(registerResult), static_cast<int>(registerResult));
            esp_now_deinit();
            g_espNowInitialised = false;
            return false;
        }

        delay(10);
        g_paired = false;
        memset(g_controllerMac, 0, sizeof(g_controllerMac));
        g_lastCommand = ControlPacket{};
        g_lastCommandTime = 0;
        g_lastPairingAckTime = 0;
        g_lastIliteBroadcastTime = 0;
        resendIndex = 0;
        memset(&emission, 0, sizeof(emission));
        return true;
    }
} // namespace

TelemetryPacket emission{};
uint8_t resendIndex = 0;

const uint8_t BroadcastMac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

bool init(const char *ssid, const char *password, int tcpPort) {
    Serial.println("Initialising Comms with custom recv callback");
    return initInternal(ssid, password, tcpPort, nullptr);
}

bool init(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback) {
    Serial.println("Initialising Comms with custom recv callback");
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

uint32_t lastPairingAckTimeMs() {
    return g_lastPairingAckTime;
}

const uint8_t *controllerMac() {
    return g_controllerMac;
}

uint32_t lastIliteBroadcastTimeMs() {
    return g_lastIliteBroadcastTime;
}

} // namespace Comms

