#include <Arduino.h>
#include "crc.h"
#include "serialize.h"
#include <SoftwareSerial.h>

// TODO: Improve robustness of reads

// Reading
unsigned char readRes(Response *res, unsigned char timeout) {
  unsigned char buf[3];
  unsigned char status = blockingRead(buf, 3, timeout);
  if (status != READ_SUCCESS) { // check if read is successful
    return status;
  }
  
  if (buf[2] != CRC8(buf, 2)) { // check if checksum is correct
    return PACKET_BAD_CHECKSUM;
  }

  res->type = buf[0];
  res->id = buf[1];
  return PACKET_OK;
}

unsigned char blockingRead(unsigned char *buf, unsigned char len, unsigned char timeout) {
  unsigned long start = millis();
  unsigned char bytesRead;
  do {
    if (Serial3.available()) {
      bytesRead = Serial3.readBytes(buf, len);
      if (bytesRead == len) return READ_SUCCESS;
      else                  return INVALID_RES;
    }
  } while (millis() < start + timeout);
  return READ_TIMEOUT;
}

void clearSerial() {
  while (Serial3.available()) {
    Serial3.read();
  }
}

// Writing
unsigned char serialize(unsigned char *buf, unsigned char type, unsigned char id, void *data, unsigned char len) {
  buf[0] = type;
  buf[1] = id;
  memcpy(buf + 2, data, len);
  buf[len + 2] = CRC8(buf, len + 2);
  return len + 3; // type + id + crc = 3
}

void send(unsigned char *buf, unsigned char len) {   // unsigned char timeout
  Serial3.write(buf, len);
}

void sendRes(unsigned char type, unsigned char id) {
  unsigned char buf[RES_BUF];
  serialize(buf, type, id, 0, 0);
  send(buf, RES_BUF);
}

void sendSensorData(SensorGroup *sensorData, unsigned char id) {
  unsigned char buf[SENSOR_BUF];
  unsigned char len = serialize(buf, SENSOR_DATA, id, (void *)sensorData, sizeof(SensorGroup));
  send(buf, len);
}

void sendSensorDataDone(unsigned char id) {
  unsigned char buf[SENSOR_BUF];
  SensorGroup sensorData = {0};
  unsigned char len = serialize(buf, DONE, id, (void *) &sensorData, sizeof(SensorGroup));
  send(buf, len);
}

void sendPower(Power *pw, unsigned char id) {
  unsigned char buf[POWER_BUF];
  unsigned char len = serialize(buf, POWER_DATA, id, (void *) pw, sizeof(Power));
  send(buf, len);
}

void sendPowerDone(unsigned char id) {
  unsigned char buf[POWER_BUF];
  Power pw = {0};
  unsigned char len = serialize(buf, DONE, id, (void *) &pw, sizeof(Power));
  send(buf, len);
}
