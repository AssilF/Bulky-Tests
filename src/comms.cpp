#include "comms.h"

#include <algorithm>
#include <cstring>
#include <cstdio>
#include <vector>

#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>

namespace Comms {
namespace {

const char *kDefaultControllerType = "controller";
const char *kDefaultControlledType = "controlled";

DeviceRole g_role = DEVICE_ROLE;
char g_platform[sizeof(Identity::platform)] = "Bulky";
char g_customId[sizeof(Identity::customId)] = "Bulky-A10";
char g_deviceTypeOverride[sizeof(Identity::deviceType)] = "";
bool g_hasDeviceTypeOverride = false;

Identity g_selfIdentity{};
Identity g_peerIdentity{};
uint8_t g_peerMac[6] = {0};
bool g_paired = false;
bool g_waitingForAck = false;

uint32_t g_lastPeerActivityMs = 0;
uint32_t g_lastKeepaliveSentMs = 0;
uint32_t g_lastBroadcastMs = 0;
uint32_t g_lastConfirmMs = 0;

std::vector<DiscoveryInfo> g_discovered;
TargetSelector g_targetSelector = nullptr;

ControlPacket g_lastCommand{};
uint32_t g_lastCommandMs = 0;
bool g_hasCommand = false;

bool g_initialized = false;
portMUX_TYPE g_mutex = portMUX_INITIALIZER_UNLOCKED;
esp_now_recv_cb_t g_userRecvCallback = nullptr;

const char *roleDeviceType(DeviceRole role) {
    return role == DeviceRole::Controller ? kDefaultControllerType : kDefaultControlledType;
}

bool isBroadcast(const uint8_t mac[6]) {
    static const uint8_t broadcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    return std::memcmp(mac, broadcast, 6) == 0;
}

void logPacket(const char *prefix, MessageType type, const uint8_t mac[6]) {
    if (type == MessageType::MSG_KEEPALIVE) {
        return;
    }
    Serial.printf("[COMMS] %s type=%u to %s\n", prefix,
                  static_cast<unsigned>(type), macToString(mac).c_str());
}

void resetLinkInternal(const char *reason) {
    bool wasPaired = false;
    uint8_t mac[6] = {0};
    {
        portENTER_CRITICAL(&g_mutex);
        if (!reason) {
            reason = "reset";
        }
        wasPaired = g_paired;
        std::memcpy(mac, g_peerMac, sizeof(mac));
        g_paired = false;
        g_waitingForAck = false;
        std::memset(g_peerMac, 0, sizeof(g_peerMac));
        std::memset(&g_peerIdentity, 0, sizeof(g_peerIdentity));
        g_lastPeerActivityMs = 0;
        g_lastKeepaliveSentMs = 0;
        g_lastConfirmMs = 0;
        g_hasCommand = false;
        g_lastCommandMs = 0;
        portEXIT_CRITICAL(&g_mutex);
    }
    if (wasPaired && !isBroadcast(mac) && esp_now_is_peer_exist(mac)) {
        esp_now_del_peer(mac);
    }
    Serial.printf("[COMMS] Link reset (%s)\n", reason);
}

void fillIdentityStrings(Identity &identity) {
    std::memset(&identity, 0, sizeof(identity));
    const char *deviceType = g_hasDeviceTypeOverride ? g_deviceTypeOverride : roleDeviceType(g_role);
    std::snprintf(identity.deviceType, sizeof(identity.deviceType), "%s", deviceType);
    std::snprintf(identity.platform, sizeof(identity.platform), "%s", g_platform);
    std::snprintf(identity.customId, sizeof(identity.customId), "%s", g_customId);
    WiFi.macAddress(identity.mac);
}

void updateSelfIdentity() {
    fillIdentityStrings(g_selfIdentity);
}

Packet makePacket(MessageType type) {
    Packet packet{};
    packet.version = PROTOCOL_VERSION;
    packet.type = type;
    fillIdentityStrings(packet.id);
    packet.monotonicMs = millis();
    packet.reserved = 0;
    return packet;
}

bool sendPacket(const uint8_t *mac, MessageType type) {
    Packet packet = makePacket(type);
    if (!isBroadcast(mac)) {
        ensurePeer(mac);
    }
    esp_err_t err = esp_now_send(mac, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    if (err != ESP_OK) {
        Serial.printf("[COMMS] Failed to send packet type %u to %s (err=%d)\n",
                      static_cast<unsigned>(type), macToString(mac).c_str(), err);
        return false;
    }
    logPacket("Sent", type, mac);
    return true;
}

void pruneDiscovery(uint32_t nowMs) {
    portENTER_CRITICAL(&g_mutex);
    auto endIt = std::remove_if(g_discovered.begin(), g_discovered.end(), [nowMs](const DiscoveryInfo &info) {
        return nowMs - info.lastSeenMs > DEVICE_TTL_MS;
    });
    if (endIt != g_discovered.end()) {
        g_discovered.erase(endIt, g_discovered.end());
    }
    portEXIT_CRITICAL(&g_mutex);
}

std::vector<DiscoveryInfo> discoverySnapshot() {
    std::vector<DiscoveryInfo> snapshot;
    portENTER_CRITICAL(&g_mutex);
    snapshot = g_discovered;
    portEXIT_CRITICAL(&g_mutex);
    return snapshot;
}

void processPairRequest(const uint8_t *mac, const Packet &packet, uint32_t nowMs) {
    if (g_role != DeviceRole::Controlled) {
        return;
    }
    (void)packet;
    ensurePeer(mac);
    sendPacket(mac, MessageType::MSG_IDENTITY_REPLY);
    portENTER_CRITICAL(&g_mutex);
    g_lastPeerActivityMs = nowMs;
    portEXIT_CRITICAL(&g_mutex);
}

void processIdentityReply(const uint8_t *mac, const Packet &packet, uint32_t nowMs) {
    if (g_role != DeviceRole::Controller) {
        return;
    }
    (void)mac;
    DiscoveryInfo entry{};
    entry.identity = packet.id;
    entry.lastSeenMs = nowMs;

    portENTER_CRITICAL(&g_mutex);
    auto it = std::find_if(g_discovered.begin(), g_discovered.end(), [&](const DiscoveryInfo &existing) {
        return macEqual(existing.identity.mac, packet.id.mac);
    });
    if (it != g_discovered.end()) {
        *it = entry;
    } else {
        g_discovered.push_back(entry);
        Serial.printf("[COMMS] Discovered %s (%s)\n", packet.id.customId, macToString(packet.id.mac).c_str());
    }
    portEXIT_CRITICAL(&g_mutex);
}

void processPairConfirm(const uint8_t *mac, const Packet &packet, uint32_t nowMs) {
    if (g_role != DeviceRole::Controlled) {
        return;
    }
    ensurePeer(mac);
    {
        portENTER_CRITICAL(&g_mutex);
        g_peerIdentity = packet.id;
        std::memcpy(g_peerMac, mac, sizeof(g_peerMac));
        g_paired = true;
        g_lastPeerActivityMs = nowMs;
        g_lastKeepaliveSentMs = nowMs;
        g_waitingForAck = false;
        g_hasCommand = false;
        g_lastCommandMs = 0;
        portEXIT_CRITICAL(&g_mutex);
    }
    Serial.printf("[COMMS] Paired with controller %s\n", packet.id.customId);
    sendPacket(mac, MessageType::MSG_PAIR_ACK);
}

void processPairAck(const uint8_t *mac, const Packet &packet, uint32_t nowMs) {
    if (g_role != DeviceRole::Controller) {
        return;
    }
    ensurePeer(mac);
    {
        portENTER_CRITICAL(&g_mutex);
        g_peerIdentity = packet.id;
        std::memcpy(g_peerMac, mac, sizeof(g_peerMac));
        g_paired = true;
        g_waitingForAck = false;
        g_lastPeerActivityMs = nowMs;
        g_lastKeepaliveSentMs = nowMs;
        g_hasCommand = false;
        g_lastCommandMs = 0;
        portEXIT_CRITICAL(&g_mutex);
    }
    Serial.printf("[COMMS] Pair acknowledged by %s\n", packet.id.customId);
}

void processKeepalive(const uint8_t *mac, const Packet &packet, uint32_t nowMs) {
    (void)packet;
    if (!macEqual(mac, g_peerMac)) {
        return;
    }
    portENTER_CRITICAL(&g_mutex);
    g_lastPeerActivityMs = nowMs;
    portEXIT_CRITICAL(&g_mutex);
}

void handlePacket(const uint8_t *mac, const Packet &packet, uint32_t nowMs) {
    if (packet.version != PROTOCOL_VERSION) {
        Serial.printf("[COMMS] Ignoring packet with mismatched version %u\n", packet.version);
        return;
    }
    switch (packet.type) {
    case MessageType::MSG_PAIR_REQ:
        processPairRequest(mac, packet, nowMs);
        break;
    case MessageType::MSG_IDENTITY_REPLY:
        processIdentityReply(mac, packet, nowMs);
        break;
    case MessageType::MSG_PAIR_CONFIRM:
        processPairConfirm(mac, packet, nowMs);
        break;
    case MessageType::MSG_PAIR_ACK:
        processPairAck(mac, packet, nowMs);
        break;
    case MessageType::MSG_KEEPALIVE:
        processKeepalive(mac, packet, nowMs);
        break;
    }
}

void sendPairRequest(uint32_t nowMs) {
    if (!g_initialized) {
        return;
    }
    if (nowMs - g_lastBroadcastMs < BROADCAST_INTERVAL_MS) {
        return;
    }
    g_lastBroadcastMs = nowMs;
    sendPacket(BroadcastMac, MessageType::MSG_PAIR_REQ);
}

void attemptPairing(uint32_t nowMs) {
    if (g_role != DeviceRole::Controller) {
        return;
    }
    if (g_paired) {
        return;
    }
    if (g_waitingForAck) {
        if (nowMs - g_lastConfirmMs > LINK_TIMEOUT_MS) {
            Serial.println("[COMMS] Pair confirm timeout, retrying");
            portENTER_CRITICAL(&g_mutex);
            g_waitingForAck = false;
            portEXIT_CRITICAL(&g_mutex);
        }
        return;
    }
    std::vector<DiscoveryInfo> snapshot = discoverySnapshot();
    if (snapshot.empty()) {
        return;
    }
    int targetIndex = 0;
    if (g_targetSelector != nullptr) {
        targetIndex = g_targetSelector(snapshot.data(), snapshot.size());
        if (targetIndex < 0 || static_cast<size_t>(targetIndex) >= snapshot.size()) {
            targetIndex = 0;
        }
    }
    const Identity &targetIdentity = snapshot[static_cast<size_t>(targetIndex)].identity;
    sendPacket(targetIdentity.mac, MessageType::MSG_PAIR_CONFIRM);
    portENTER_CRITICAL(&g_mutex);
    g_lastConfirmMs = nowMs;
    g_waitingForAck = true;
    portEXIT_CRITICAL(&g_mutex);
}

void maintainLink(uint32_t nowMs) {
    if (!g_paired) {
        return;
    }
    if (nowMs - g_lastKeepaliveSentMs >= BROADCAST_INTERVAL_MS) {
        sendPacket(g_peerMac, MessageType::MSG_KEEPALIVE);
        portENTER_CRITICAL(&g_mutex);
        g_lastKeepaliveSentMs = nowMs;
        portEXIT_CRITICAL(&g_mutex);
    }
    uint32_t lastActivity = 0;
    {
        portENTER_CRITICAL(&g_mutex);
        lastActivity = g_lastPeerActivityMs;
        portEXIT_CRITICAL(&g_mutex);
    }
    if (lastActivity != 0 && nowMs - lastActivity > LINK_TIMEOUT_MS) {
        resetLinkInternal("timeout");
    }
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    uint32_t nowMs = millis();
    if (len == static_cast<int>(sizeof(Packet))) {
        Packet packet{};
        std::memcpy(&packet, incomingData, sizeof(packet));
        handlePacket(mac, packet, nowMs);
    } else if (len == static_cast<int>(sizeof(ControlPacket))) {
        if (macEqual(mac, g_peerMac)) {
            portENTER_CRITICAL(&g_mutex);
            std::memcpy(&g_lastCommand, incomingData, sizeof(ControlPacket));
            g_lastCommandMs = nowMs;
            g_hasCommand = true;
            g_lastPeerActivityMs = nowMs;
            portEXIT_CRITICAL(&g_mutex);
        }
    }
    if (g_userRecvCallback != nullptr) {
        g_userRecvCallback(mac, incomingData, len);
    }
}

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    (void)mac;
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.printf("[COMMS] Send status: %d\n", status);
    }
}

} // namespace

const uint8_t BroadcastMac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

void setRole(DeviceRole role) {
    g_role = role;
    updateSelfIdentity();
}

DeviceRole getRole() {
    return g_role;
}

void setPlatform(const char *platformName) {
    if (platformName == nullptr) {
        platformName = "";
    }
    std::snprintf(g_platform, sizeof(g_platform), "%s", platformName);
    updateSelfIdentity();
}

void setCustomId(const char *customId) {
    if (customId == nullptr) {
        customId = "";
    }
    std::snprintf(g_customId, sizeof(g_customId), "%s", customId);
    updateSelfIdentity();
}

void setDeviceTypeOverride(const char *deviceTypeName) {
    if (deviceTypeName == nullptr || deviceTypeName[0] == '\0') {
        g_hasDeviceTypeOverride = false;
        g_deviceTypeOverride[0] = '\0';
    } else {
        g_hasDeviceTypeOverride = true;
        std::snprintf(g_deviceTypeOverride, sizeof(g_deviceTypeOverride), "%s", deviceTypeName);
    }
    updateSelfIdentity();
}

void setTargetSelector(TargetSelector selector) {
    g_targetSelector = selector;
}

void fillSelfIdentity(Identity &outIdentity) {
    fillIdentityStrings(outIdentity);
}

String macToString(const uint8_t mac[6]) {
    char buffer[18];
    std::snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buffer);
}

bool macEqual(const uint8_t lhs[6], const uint8_t rhs[6]) {
    return std::memcmp(lhs, rhs, 6) == 0;
}

bool ensurePeer(const uint8_t mac[6]) {
    if (isBroadcast(mac)) {
        if (!esp_now_is_peer_exist(mac)) {
            esp_now_peer_info_t info{};
            std::memcpy(info.peer_addr, mac, 6);
            info.channel = WIFI_CHANNEL;
            info.encrypt = false;
            info.ifidx = WIFI_IF_STA;
            return esp_now_add_peer(&info) == ESP_OK;
        }
        return true;
    }
    if (esp_now_is_peer_exist(mac)) {
        return true;
    }
    esp_now_peer_info_t peer{};
    std::memcpy(peer.peer_addr, mac, 6);
    peer.channel = WIFI_CHANNEL;
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA;
    esp_err_t result = esp_now_add_peer(&peer);
    if (result != ESP_OK) {
        Serial.printf("[COMMS] Failed to add peer %s (err=%d)\n", macToString(mac).c_str(), result);
        return false;
    }
    return true;
}

bool init(const char *ssid, const char *password, int tcpPort) {
    return init(ssid, password, tcpPort, nullptr);
}

bool init(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback) {
    (void)tcpPort;
    g_initialized = false;
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    WiFi.softAP(ssid, password, WIFI_CHANNEL, false, 1);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[COMMS] esp_now_init failed");
        return false;
    }

    updateSelfIdentity();

    g_userRecvCallback = recvCallback;
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    ensurePeer(BroadcastMac);

    portENTER_CRITICAL(&g_mutex);
    g_discovered.clear();
    g_paired = false;
    g_waitingForAck = false;
    std::memset(&g_peerIdentity, 0, sizeof(g_peerIdentity));
    std::memset(g_peerMac, 0, sizeof(g_peerMac));
    g_lastBroadcastMs = millis() - BROADCAST_INTERVAL_MS;
    g_lastKeepaliveSentMs = 0;
    g_lastPeerActivityMs = 0;
    g_lastConfirmMs = 0;
    g_lastCommandMs = 0;
    g_hasCommand = false;
    portEXIT_CRITICAL(&g_mutex);

    g_initialized = true;
    Serial.printf("[COMMS] Initialized (%s mode)\n", roleDeviceType(g_role));
    return true;
}

void loop() {
    if (!g_initialized) {
        return;
    }
    uint32_t nowMs = millis();
    pruneDiscovery(nowMs);
    if (g_role == DeviceRole::Controller) {
        sendPairRequest(nowMs);
        attemptPairing(nowMs);
    }
    maintainLink(nowMs);
}

bool receiveCommand(ControlPacket &cmd, uint32_t *timestampMs) {
    portENTER_CRITICAL(&g_mutex);
    cmd = g_lastCommand;
    if (timestampMs != nullptr) {
        *timestampMs = g_lastCommandMs;
    }
    bool available = g_hasCommand && g_paired;
    portEXIT_CRITICAL(&g_mutex);
    return available;
}

uint32_t lastCommandTimestamp() {
    portENTER_CRITICAL(&g_mutex);
    uint32_t ts = g_lastCommandMs;
    portEXIT_CRITICAL(&g_mutex);
    return ts;
}

LinkStatus getLinkStatus() {
    LinkStatus status;
    portENTER_CRITICAL(&g_mutex);
    status.paired = g_paired;
    status.peerIdentity = g_peerIdentity;
    std::memcpy(status.peerMac, g_peerMac, sizeof(status.peerMac));
    status.lastActivityMs = g_lastPeerActivityMs;
    status.lastCommandMs = g_lastCommandMs;
    portEXIT_CRITICAL(&g_mutex);
    return status;
}

bool paired() {
    portENTER_CRITICAL(&g_mutex);
    bool result = g_paired;
    portEXIT_CRITICAL(&g_mutex);
    return result;
}

} // namespace Comms
