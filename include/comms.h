#ifndef COMMS_H
#define COMMS_H

#include <Arduino.h>
#include <esp_now.h>

// Destination address for ESP-NOW transmissions.
extern uint8_t targetAddress[6];

// ESP-NOW peer information structure.
extern esp_now_peer_info bot;

// Status flags for communication callbacks.
extern bool sent_Status;
extern bool receive_Status;

// Packet index definitions used when packing telemetry.
enum PacketIndex : byte
{
  PACK_TELEMETRY = 0,
  PACK_LINE      = 1,
  PACK_PID       = 2,
  PACK_FIRE      = 3,
};

// Communication data structures shared with the controller.
struct receptionDataPacket
{
  byte Speed;
  byte MotionState;
  byte pitch;
  byte yaw;
  bool bool1[4];
};
extern receptionDataPacket reception;

struct emissionDataPacket
{
  byte INDEX;
  byte statusByte;
  int dataByte[8];
  byte okIndex;
};
extern emissionDataPacket emission;

// ESP-NOW callback executed after packet transmission.
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

// ESP-NOW callback executed when data is received.
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

// Populate emission packet with telemetry.
void packData(byte index);

// Pack telemetry for the given index and transmit via ESP-NOW.
// Returns true if the packet was queued for sending successfully.
bool sendData(byte index);

// Process received packets (stub).
void processData(byte index);

// Helper index used for resending data blocks.
extern byte resendIndex;

#endif // COMMS_H
