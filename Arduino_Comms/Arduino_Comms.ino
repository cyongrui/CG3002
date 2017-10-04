#include <Arduino_FreeRTOS.h>
// #include <SoftwareSerial.h>
#include <semphr.h>
#include "serialize.h"
#include "crc.h"

#define LOGGING_ENABLED 1
#define SERIAL_TIMEOUT_PERIOD 1000
#define READ_SENSOR_PERIOD 50
// #define SEND_SENSOR_DATA_PERIOD 50

#define DATABUF_SEM_WAIT (3000 / portTICK_PERIOD_MS)
#define POWER_SEM_WAIT (3000 / portTICK_PERIOD_MS)
#define READ_SENSOR_FREQ (250 / portTICK_PERIOD_MS)
#define READ_POWER_FREQ (1000 / portTICK_PERIOD_MS)
#define LISTEN_FREQ (300 / portTICK_PERIOD_MS)

#define SENSOR_BUF_SIZE 50
#define POWER_BUF_SIZE 10

//SensorData d;

static SensorGroup databuf[SENSOR_BUF_SIZE];
static unsigned char powerBuf[POWER_BUF_SIZE];
static SemaphoreHandle_t bufMutex;
static SemaphoreHandle_t powerMutex;
static SemaphoreHandle_t consoleMutex;

static unsigned int sensorBufEmptyId = 0;
static unsigned int sensorBufFilled = 0;

void log(const char* msg) {
  if (LOGGING_ENABLED) {
    if (xSemaphoreTake(consoleMutex, DATABUF_SEM_WAIT) == pdTRUE) {
      Serial.println(msg);
    }
    xSemaphoreGive(consoleMutex);
  }
}

void log(unsigned char msg) {
  if (LOGGING_ENABLED) {
    if (xSemaphoreTake(consoleMutex, DATABUF_SEM_WAIT) == pdTRUE) {
      Serial.println(msg);
    }
    xSemaphoreGive(consoleMutex);
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  // d.accel0 = 100;
  // d.accel1 = 200;
  // d.accel2 = 300;
  // d.row0 = 400;
  // d.row1 = 500;
  // d.row2 = 0;
  // for (int i = 0; i < 100; i++) {
  //   d.row2 = i;
  //   databuf[i] = d;
  // }

  // function, task name for readability, stack size, NULL, priority
  bufMutex = xSemaphoreCreateMutex();
  if (bufMutex == NULL) {
    Serial.println("Error: Semaphore cannot be created.");
  }

  powerMutex = xSemaphoreCreateMutex();
  if (powerMutex == NULL) {
    Serial.println("Error: Semaphore cannot be created.");
  }

  consoleMutex = xSemaphoreCreateMutex();
  if (consoleMutex == NULL) {
    Serial.println("Error: Semaphore cannot be created.");
  }

  xSemaphoreGive(bufMutex);
  xSemaphoreGive(powerMutex);
  xSemaphoreGive(consoleMutex);

  xTaskCreate(handshake, (const portCHAR *) "HS", 128, NULL , 4, NULL);
  xTaskCreate(readSensor, (const portCHAR *) "RS", 128, NULL , 3, NULL);
  xTaskCreate(readPower, (const portCHAR *) "Read power consumption values", 128, NULL , 2, NULL);
  xTaskCreate(listenForReq,  (const portCHAR *)"Listen to port for request", 256, NULL, 1, NULL);

  vTaskStartScheduler();
}

void handshake(void *pvParameters) {
  Response res;
  bool done = false;
  while (!done) {

    log("Ready for handshake");
    if (readRes(&res, SERIAL_TIMEOUT_PERIOD) == READ_TIMEOUT || res.type != HELLO) {
      clearSerial();
      continue;
    }
    log("Hello received");

    for (int i = 0; i < 3; i++) { // Try to send ACK 3 times
      sendRes(ACK, 0);  // ACK HELLO
      if (readRes(&res, SERIAL_TIMEOUT_PERIOD) == READ_TIMEOUT || res.type != HELLO_DONE) {
        log(res.type);
        log("Fail to get HELLO_DONE");
        continue;
      } else {
        log("Handshake done!");
        done = true;
        break;
      }
    }
  }
  vTaskDelete(NULL);
}

void readSensor(void *pvParameters) {
  TickType_t xLastWakeTime;
  while (1) {
    log("eEntering read sensor");
    xLastWakeTime = xTaskGetTickCount();
    log("Waiting for data mutex");
    if (xSemaphoreTake(bufMutex, DATABUF_SEM_WAIT) == pdTRUE) {
      log("data mutex received");
      SensorGroup sensorData;
      memset(&sensorData, 0xff, sizeof(sensorData));
      sensorData.sensor0.accelX = sensorBufEmptyId;
      databuf[sensorBufEmptyId] = sensorData;
      sensorBufEmptyId = (sensorBufEmptyId + 1) % SENSOR_BUF_SIZE;
      sensorBufFilled += (sensorBufFilled + 1 >= SENSOR_BUF_SIZE) ? 0 :  1;
      
      log("Reading data");
      }
    xSemaphoreGive(bufMutex);
    log("data mutex released");

    vTaskDelayUntil(&xLastWakeTime, READ_SENSOR_FREQ);
    xLastWakeTime = xTaskGetTickCount();
  }
}

void readPower(void *pvParameters) {
  TickType_t xLastWakeTime;
  while (1) {
    xLastWakeTime = xTaskGetTickCount();
    if (xSemaphoreTake(powerMutex, POWER_SEM_WAIT) == pdTRUE) {
      delay(20);

      // Read sensor data here
      log("Reading power");
    }
    xSemaphoreGive(powerMutex);

    vTaskDelayUntil(&xLastWakeTime, READ_POWER_FREQ);
  }
}


void listenForReq(void *pvParameters) {
  Response res;
  unsigned char status;

  while (1) {
    vTaskDelay(LISTEN_FREQ);
    log("Entering listen for req");
    if (Serial1.available() <= 0) {
      continue;
    }
    // Serial1 bytes available in serial

    if (readRes(&res, 0) != PACKET_OK) {
      continue;
    }
    // Packet is legit

    if (res.type == REQUEST_DATA) {
      sendSensorBuf();
      log("Sensor data transfer complete!");
    } else if (res.type == REQUEST_POWER) {
      sendPowerBuf();
      log("Power data transfer complete!");
    } else {
      continue;
    }
  }

}

void sendSensorBuf() {
  Response res;
  unsigned char id = 0;
  log("Sending sensor data");
//  bool done = False;
  while (1) {
    if (xSemaphoreTake(bufMutex, DATABUF_SEM_WAIT) == pdTRUE) {

      // Check if buffer is empty
      if (sensorBufFilled <= 0) {
        while (1) {
          sendSensorDataDone(id);
          if (readRes(&res, SERIAL_TIMEOUT_PERIOD) == PACKET_OK && res.type == ACK && res.id == id) {
            break;
          }
        }
        
      } else {
        // Buffer is not empty, send 1 data
        log("sending 1 data");
        sendSensorData(&databuf[(sensorBufEmptyId - sensorBufFilled) % SENSOR_BUF_SIZE], id);
      }

      
      if (readRes(&res, SERIAL_TIMEOUT_PERIOD) == PACKET_OK && res.type == ACK && res.id == id) {
        log("Data ACK received");
        id++;
        sensorBufFilled--;
        clearSerial();
      } else {
        log("Data ACK not received");
      }
    }
    xSemaphoreGive(bufMutex);
  }
  xSemaphoreGive(bufMutex);

}

void sendPowerBuf() {
  if (xSemaphoreTake(powerMutex, POWER_SEM_WAIT) == pdTRUE) {
    delay(20);
    log("Sending power");
  }
  xSemaphoreGive(powerMutex);
}

void loop() {
  // empty
}
