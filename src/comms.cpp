#include "comms.h"

#include <ctype.h>
#include <cstring>
#include <esp_wifi.h>

namespace Comms {
namespace {
    bool g_paired = false;
    ThegillCommand g_lastCommand{};
    uint8_t g_controllerMac[6] = {0};
    uint32_t g_lastCommandTime = 0;
    uint32_t g_lastPairingAckTime = 0;
}

const uint8_t BroadcastMac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

namespace {

bool identityContains(const IdentityMessage &msg, const char *needle) {
    const size_t maxLen = sizeof(msg.identity);
    const size_t needleLen = strlen(needle);
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
            actual = toupper(static_cast<unsigned char>(actual));
            expected = toupper(static_cast<unsigned char>(expected));
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
    if (identityContains(msg, "ELITE")) {
        return true;
    }
    if (identityContains(msg, "ILITE")) {
        return true;
    }
    return false;
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    
    if (len == static_cast<int>(sizeof(IdentityMessage)) && !g_paired) {
        const IdentityMessage *msg = reinterpret_cast<const IdentityMessage *>(incomingData);
        if (msg->type == SCAN_REQUEST) {
            IdentityMessage resp{};
            resp.type = DRONE_IDENTITY;
            strncpy(resp.identity, "THEGILL", sizeof(resp.identity));
            WiFi.macAddress(resp.mac);
            esp_now_send(mac, reinterpret_cast<const uint8_t *>(&resp), sizeof(resp));
            return;
        }
        if (isEliteControllerIdentity(*msg)) {
            memcpy(g_controllerMac, mac, 6);
            if (!esp_now_is_peer_exist(mac)) {
                esp_now_peer_info_t peerInfo{};
                memcpy(peerInfo.peer_addr, mac, 6);
                peerInfo.channel = 0;
                peerInfo.encrypt = false;
                peerInfo.ifidx = WIFI_IF_STA;
                esp_now_add_peer(&peerInfo);
            }
            IdentityMessage ack{};
            ack.type = DRONE_ACK;
            esp_now_send(mac, reinterpret_cast<const uint8_t *>(&ack), sizeof(ack));
            g_paired = true;
            g_lastPairingAckTime = millis();
            return;
        }
    }

    if (len == static_cast<int>(sizeof(ThegillCommand))) {
        const ThegillCommand *cmd = reinterpret_cast<const ThegillCommand *>(incomingData);
        if (cmd->magic == THEGILL_PACKET_MAGIC) {
            g_lastCommand = *cmd;
            g_lastCommandTime = millis();
        }
    }
}

bool initInternal(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallbac k) {
    (void)tcpPort;
    WiFi.mode(WIFI_AP_STA);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    WiFi.setSleep(false);
    WiFi.softAP(ssid, password);

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    esp_now_peer_info_t peerInfo{};
    memcpy(peerInfo.peer_addr, BroadcastMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;
    if (!esp_now_is_peer_exist(BroadcastMac)) {
        esp_now_add_peer(&peerInfo);
    }

    esp_now_register_recv_cb(recvCallback ? recvCallback : onDataRecv);

    g_paired = false;
    memset(g_controllerMac, 0, sizeof(g_controllerMac));
    memset(&g_lastCommand, 0, sizeof(g_lastCommand));
    g_lastCommandTime = 0;
    g_lastPairingAckTime = 0;
    return true;
}

} // namespace

bool init(const char *ssid, const char *password, int tcpPort) {
    return initInternal(ssid, password, tcpPort, nullptr);
}

bool init(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback) {
    return initInternal(ssid, password, tcpPort, recvCallback);
}

bool receiveCommand(ThegillCommand &cmd) {
    cmd = g_lastCommand;
    return g_paired;
}

bool paired() {
    return g_paired;
}

uint32_t lastCommandTimeMs() {
    return g_lastCommandTime;
}

uint32_t lastPairingAckTimeMs() {
    return g_lastPairingAckTime;
}

} // namespace Comms

