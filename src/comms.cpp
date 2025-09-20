#include "comms.h"

#include <cstring>

namespace Comms {
namespace {
    static_assert(sizeof(ControlPacket) == 8, "ControlPacket must remain 8 bytes");
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

    void onDataRecvInternal(const uint8_t *mac, const uint8_t *incomingData, int len) {
        if (len == static_cast<int>(sizeof(IdentityMessage))) {
            const IdentityMessage *msg = reinterpret_cast<const IdentityMessage *>(incomingData);
            if (msg->type == SCAN_REQUEST) {
                respondWithIdentity(mac);
                return;
            }
            if (msg->type == ILITE_IDENTITY) {
                if (!esp_now_is_peer_exist(mac)) {
                    esp_now_peer_info_t peerInfo{};
                    memcpy(peerInfo.peer_addr, mac, 6);
                    peerInfo.channel = 0;
                    peerInfo.encrypt = false;
                    esp_now_add_peer(&peerInfo);
                }
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
        return true;
    }
} // namespace

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

bool sendTelemetry(const TelemetryPacket &packet) {
    if (!g_paired) {
        return false;
    }
    if (memcmp(g_controllerMac, "\0\0\0\0\0\0", 6) == 0) {
        return false;
    }
    return esp_now_send(g_controllerMac, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet)) == ESP_OK;
}

uint32_t lastCommandTimeMs() {
    return g_lastCommandTime;
}

const uint8_t *controllerMac() {
    return g_controllerMac;
}

} // namespace Comms

