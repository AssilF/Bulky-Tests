#include "comms.h"
#include "sensors.h"
#include "line.h"
#include "motion.h"

#define Cm_per_Second_Conversion 10.362

uint8_t targetAddress[6] = {0x78, 0x21, 0x84, 0x7E, 0x68, 0x1C};
esp_now_peer_info bot;

bool sent_Status;
bool receive_Status;

receptionDataPacket reception;
emissionDataPacket emission;

byte resendIndex;

extern int operationMode;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  sent_Status = (status == ESP_NOW_SEND_SUCCESS);
  if(!sent_Status) {
    // preserve the index of the packet that failed so it can be resent
    resendIndex = emission.INDEX;
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&reception, incomingData, sizeof(reception));
  receive_Status=1;
}

void packData(byte index)
{
  switch(index)
  {
    case PACK_TELEMETRY:
      emission.INDEX = index;
      emission.dataByte[0] = linePosition;
      emission.dataByte[1] = front_distance;
      emission.dataByte[2] = bot_distance;
      emission.dataByte[3] = IRBias;
      emission.dataByte[4] = average_count*Cm_per_Second_Conversion;
      emission.dataByte[5] = batteryLevel;
      emission.dataByte[6] = operationMode;
      break;

    case PACK_LINE:
      emission.INDEX=index;
      emission.dataByte[0] = sensor_readings[line_reading1];
      emission.dataByte[1] = sensor_readings[line_reading2];
      emission.dataByte[2] = sensor_readings[line_reading3];
      emission.dataByte[3] = sensor_readings[line_reading4];
      emission.dataByte[4] = lineThresholdsLowers[0];
      emission.dataByte[5] = lineThresholdsLowers[1];
      emission.dataByte[6] = lineThresholdsLowers[2];
      emission.dataByte[7] = lineThresholdsLowers[3];
      break;

    case PACK_PID:
      emission.INDEX= index;
      emission.dataByte[0] = kp*100;
      emission.dataByte[1] = kd*100;
      emission.dataByte[3] = baseSpeed;
      break;

    case PACK_FIRE:
      emission.INDEX= index;
      emission.dataByte[0] = sensor_readings[fire_detection_left];
      emission.dataByte[1] = sensor_readings[fire_detection_right];
      emission.dataByte[3] = fireRange;
      break;

    default:
    break;
  }
}

bool sendData(byte index)
{
  packData(index);
  esp_err_t result = esp_now_send(targetAddress, (uint8_t *)&emission, sizeof(emission));
  if(result != ESP_OK) {
    resendIndex = emission.INDEX;
    return false;
  }
  resendIndex = 0;
  return true;
}

void processData(byte index)
{
  (void)index;
}

