#ifndef SERIALIZE_H
#define SERIALIZE_H

// Data/ Response type
#define SENSOR_DATA 1
#define HELLO 2
#define HELLO_DONE 3
#define ACK 4
#define NAK 5
#define DONE 6
#define REQUEST_DATA 7
#define REQUEST_POWER 8
#define POWER_DATA 9

// Packet status
#define PACKET_OK 0
#define PACKET_BAD_CHECKSUM 1
#define INVALID_RES 2
#define READ_TIMEOUT 3
#define READ_SUCCESS 4

#define SENSOR_BUF 64
#define POWER_BUF 8 // 8
#define RES_BUF 3
// #define NUM_SENSOR 5

typedef struct Response {
  unsigned char type;
  unsigned char id;
//unsigned char crc;
} Response;


typedef struct Sensor {
  short accelX; // short is signed
  short accelY;
  short accelZ;
  short row0;
  short row1;
  short row2;
} Sensor;

typedef struct SensorGroup {
  Sensor sensor0;
  Sensor sensor1;
  Sensor sensor2;
} SensorGroup;

typedef struct Power {
  float power;
//  float voltage;
//  float current;
//	unsigned char voltage;
//  unsigned char current;
} Power;


// Writing
unsigned char serialize(unsigned char *buf, unsigned char type, unsigned char id, void *data, unsigned char len);
void send(unsigned char *buf, unsigned char len);   // unsigned char timeout
void sendRes(unsigned char type, unsigned char id);
void sendSensorData(SensorGroup *sensorData, unsigned char id);
void sendSensorDataDone(unsigned char id);
void sendPower(Power *pw, unsigned char id);
void sendPowerDone(unsigned char id);



// Reading
unsigned char readRes(Response *res, unsigned char timeout);
unsigned char blockingRead(unsigned char *buf, unsigned char len, unsigned char timeout);
void clearSerial();

#endif
