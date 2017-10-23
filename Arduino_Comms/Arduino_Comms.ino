#define DEBUG

/*****************************************************
  MPU6050
*****************************************************/
#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

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


// MPU6050 accelgyro;
MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define MPU_EN_ARM_L 45
#define MPU_EN_ARM_R 41
#define MPU_EN_TORSO 33 

#ifdef DEBUG
  #define OUTPUT_READABLE_ACCELGYRO
  #define VALUE_TO_MS 1670.0
  #define VALUE_TO_DPS 131.0
  //#define OUTPUT_BINARY_ACCELGYRO
#endif
  
/*****************************************************
  MULTIMETER
*****************************************************/
#define VOLTAGE_REF 4.96  // Reference voltage for anaSerial.print read
const int VOLTMETER_PIN=A0;
const int AMMETER_PIN=A1;

// INA169 constants and variables
const float RS = 0.1; // Shunt resistor value (in ohms)
int inputAmp;
float current;
float tempAmp;

// Voltmeter constants and variables
const float ratio = 2.0; // ratio = 1 / (R2 / (R1 + R2))
int inputVolt;
float voltOut = 0.0;
float voltIn = 0.0;

#define LED_PIN 13 
bool blinkState = false;

unsigned long timeStart, timeEnd;
int count = 0;
const int COUNTER_MAX = 1200;


//SensorData d;

static SensorGroup databuf[SENSOR_BUF_SIZE];
static Power powerBuf[POWER_BUF_SIZE];
static SemaphoreHandle_t bufMutex;
static SemaphoreHandle_t powerMutex;
static SemaphoreHandle_t consoleMutex;

static unsigned int sensorBufEmptyId = 0;
static unsigned int sensorBufFilled = 0;
static unsigned int powerBufEmptyId = 0;
static unsigned int powerBufFilled = 0;

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

// Code to poll accelerometers
SensorGroup pollSensor() {

  SensorGroup sensorData;
  Sensor s;
  digitalWrite(MPU_EN_ARM_L, HIGH); 
  digitalWrite(MPU_EN_ARM_R, LOW); 
  digitalWrite(MPU_EN_TORSO, LOW);
  accelgyro.setXAccelOffset(-2083);
  accelgyro.setYAccelOffset(799);
  accelgyro.setZAccelOffset(1568);
  accelgyro.setXGyroOffset(118);
  accelgyro.setYGyroOffset(15);
  accelgyro.setZGyroOffset(6);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  s.accelX = ax;
  s.accelY = ay;
  s.accelZ = az;
  s.row0 = gx;
  s.row1 = gy;
  s.row2 = gz;
  sensorData.sensor0 = s;
  #ifdef OUTPUT_READABLE_ACCELGYRO
    Serial.print("Left arm a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\n");
  #endif
  
  digitalWrite(MPU_EN_ARM_L, LOW); 
  digitalWrite(MPU_EN_ARM_R, HIGH); 
  digitalWrite(MPU_EN_TORSO, LOW);
  accelgyro.setXAccelOffset(-730);
  accelgyro.setYAccelOffset(802);
  accelgyro.setZAccelOffset(1303);
  accelgyro.setXGyroOffset(87);
  accelgyro.setYGyroOffset(-20);
  accelgyro.setZGyroOffset(28);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  s.accelX = ax;
  s.accelY = ay;
  s.accelZ = az;
  s.row0 = gx;
  s.row1 = gy;
  s.row2 = gz;
  sensorData.sensor1 = s;
  #ifdef OUTPUT_READABLE_ACCELGYRO
    Serial.print("Right arm a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\n");
  #endif
  
  digitalWrite(MPU_EN_ARM_L, LOW); 
  digitalWrite(MPU_EN_ARM_R, LOW); 
  digitalWrite(MPU_EN_TORSO, HIGH);
  accelgyro.setXAccelOffset(390);
  accelgyro.setYAccelOffset(4224);
  accelgyro.setZAccelOffset(2011);
  accelgyro.setXGyroOffset(-37);
  accelgyro.setYGyroOffset(73);
  accelgyro.setZGyroOffset(82);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  s.accelX = ax;
  s.accelY = ay;
  s.accelZ = az;
  s.row0 = gx;
  s.row1 = gy;
  s.row2 = gz;
  sensorData.sensor2 = s;
  #ifdef OUTPUT_READABLE_ACCELGYRO
    Serial.print("Torso a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\n");
  #endif
  
  delay(40);
  
  if(count >= COUNTER_MAX) {
    while(1) { 
      Serial.println("Collection has ended");
      delay(10000);
    }
  }
  count++;

  return sensorData;
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
    
  Serial.begin(115200);
  Serial1.begin(115200);

  // initialize device
  pinMode(MPU_EN_ARM_L, OUTPUT);
  pinMode(MPU_EN_ARM_R, OUTPUT);
  pinMode(MPU_EN_TORSO, OUTPUT);

  // verify connection
  #ifdef DEBUG 
    Serial.print("Testing device connections...\n");
  #endif
  digitalWrite(MPU_EN_ARM_L, HIGH); digitalWrite(MPU_EN_ARM_R, LOW); digitalWrite(MPU_EN_TORSO, LOW);
  accelgyro.initialize();
  #ifdef DEBUG 
    Serial.print(accelgyro.testConnection() ? "MPU6050 Left Arm connection successful\n" : "MPU6050 Left Arm connection failed\n");
  #endif
  digitalWrite(MPU_EN_ARM_L, LOW); digitalWrite(MPU_EN_ARM_R, HIGH); digitalWrite(MPU_EN_TORSO, LOW);
  accelgyro.initialize();
  #ifdef DEBUG 
    Serial.print(accelgyro.testConnection() ? "MPU6050 Right Arm connection successful\n" : "MPU6050 Right Arm connection failed\n");
  #endif
  digitalWrite(MPU_EN_ARM_L, LOW); digitalWrite(MPU_EN_ARM_R, LOW); digitalWrite(MPU_EN_TORSO, HIGH);
  accelgyro.initialize();
  #ifdef DEBUG 
    Serial.print(accelgyro.testConnection() ? "MPU6050 Torso connection successful\n" : "MPU6050 Torso connection failed\n");
  #endif

  // configure LED
  pinMode(LED_PIN, OUTPUT);
  
  for(count = 5; count > 0; count--) {
    Serial.print(count);
    Serial.println(" seconds left to start");
    delay(1000);
  }
  count = 0;

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

  //  SensorGroup sensorData;
  //  memset(&sensorData, 0xee, sizeof(sensorData));
  //  for (int i = 0; i < 5; i++) {
  //      sensorData.sensor0.accelX = i;
  //      databuf[sensorBufEmptyId] = sensorData;
  //      sensorBufEmptyId = (sensorBufEmptyId + 1) % SENSOR_BUF_SIZE;
  //      sensorBufFilled += (sensorBufFilled + 1 >= SENSOR_BUF_SIZE) ? 0 :  1;
  //  }

//  sensorsSetup();
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
    log("!Entering read sensor");
    xLastWakeTime = xTaskGetTickCount();
    log("Waiting for data mutex");
    if (xSemaphoreTake(bufMutex, DATABUF_SEM_WAIT) == pdTRUE) {
      log("data mutex received");
      SensorGroup sensorData = pollSensor();
//      Sensor s;
//      s.accelX = 1;
//      s.accelY = 2;
//      s.accelZ = 3;
//      s.row0 = 4;
//      s.row1 = 5;
//      s.row2 = 6;
//      sensorData.sensor0 = s;
//      sensorData.sensor1 = s;
//      sensorData.sensor2 = s;
      //memset(&sensorData, 0xff, sizeof(sensorData));
      //sensorData.sensor0.accelX = sensorBufEmptyId;
      databuf[sensorBufEmptyId] = sensorData;
      sensorBufEmptyId = (sensorBufEmptyId + 1) % SENSOR_BUF_SIZE;
      sensorBufFilled += (sensorBufFilled + 1 >= SENSOR_BUF_SIZE) ? 0 :  1;

      log("Reading data");
    }
    xSemaphoreGive(bufMutex);
    log("data mutex released");

    vTaskDelayUntil(&xLastWakeTime, READ_SENSOR_FREQ);
  }
}

void readPower(void *pvParameters) {
  TickType_t xLastWakeTime;
  while (1) {
    log("!Entering read power");
    xLastWakeTime = xTaskGetTickCount();
    if (xSemaphoreTake(powerMutex, POWER_SEM_WAIT) == pdTRUE) {
      // Read sensor data here
      log("Power mutex received");
      Power pw;
      pw.power = 10.0;
//      pw.voltage = powerBufEmptyId / 1.0;
//      pw.current = 99.0;
      powerBuf[powerBufEmptyId] = pw;
      powerBufEmptyId = (powerBufEmptyId + 1) % POWER_BUF_SIZE;
      powerBufFilled += (powerBufFilled + 1 >= POWER_BUF_SIZE) ? 0 : 1;

      log("reading power");

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
//          unsigned char test = readRes(&res, SERIAL_TIMEOUT_PERIOD);
//          log(test);
//          Serial.println(res.type);
//          Serial.println(res.id);
//          Serial.println(id);
          //log(res.type);
          //log(res,id);
          if (readRes(&res, SERIAL_TIMEOUT_PERIOD) == PACKET_OK && res.type == ACK && res.id == id) {
          //if (test == PACKET_OK && res.type == ACK && res.id == id) {
            break;
          }
        }
        break;
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
    log("LAST");
    xSemaphoreGive(bufMutex);
  }
  xSemaphoreGive(bufMutex);

}

void sendPowerBuf() {
  Response res;
  unsigned char id = 0;
  log("Sending power...");
  //  bool done = False;
  while (1) {
    if (xSemaphoreTake(powerMutex, POWER_SEM_WAIT) == pdTRUE) {
      // Check if buffer is empty
      if (powerBufFilled <= 0) {
        while (1) {
          sendPowerDone(id);
//          unsigned char test = readRes(&res, SERIAL_TIMEOUT_PERIOD);
//          log(test);
//          Serial.println(res.type);
//          Serial.println(res.id);
//          Serial.println(id);
          //log(res.type);
          //log(res,id);
          if (readRes(&res, SERIAL_TIMEOUT_PERIOD) == PACKET_OK && res.type == ACK && res.id == id) {
          //if (test == PACKET_OK && res.type == ACK && res.id == id) {
            log("inloop");
            break;
          }
        }
        break;
      } else {
        // Buffer is not empty, send 1 data
        log("sending 1 power");
        sendPower(&powerBuf[(powerBufEmptyId - powerBufFilled) % POWER_BUF_SIZE], id);
      }
      unsigned char test = readRes(&res, SERIAL_TIMEOUT_PERIOD);
   //   log(test);
//      Serial.println(res.type);
//      Serial.println(res.id);
//      Serial.println(id);
      //log(res.type);
      //log(res,id);
      //if (readRes(&res, SERIAL_TIMEOUT_PERIOD) == PACKET_OK && res.type == ACK && res.id == id) {
      if (test == PACKET_OK && res.type == ACK && res.id == id) {
        log("Power ACK received");
        id++;
        powerBufFilled--;
        clearSerial();
      } else {
        log("Power ACK not received");
      }
    }
    log("LAST");
    xSemaphoreGive(powerMutex);
  }
  xSemaphoreGive(powerMutex);
}

void loop() {
  // empty
}
