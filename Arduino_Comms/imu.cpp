/*****************************************************
  MPU6050
*****************************************************/
#include "imu.h"

MPU6050 accelgyro(0x69); // <-- use for AD0 high

// MPU6050 accelgyro;
bool activate(uint8_t pin) {
  digitalWrite(MPU_EN_ARM_L, LOW);
  digitalWrite(MPU_EN_ARM_R, LOW);
  digitalWrite(MPU_EN_TORSO, LOW);
  digitalWrite(pin, HIGH);
}

bool initialize(uint8_t pin) {
  activate(pin);
  accelgyro.initialize();
  return accelgyro.testConnection();
}

void readImu(uint8_t pin, Sensor *s) {
  activate(pin);
  if (pin == MPU_EN_ARM_L) {
    accelgyro.setXAccelOffset(-2060);
    accelgyro.setYAccelOffset(640);
    accelgyro.setZAccelOffset(1592);
    accelgyro.setXGyroOffset(115);
    accelgyro.setYGyroOffset(13);
    accelgyro.setZGyroOffset(6);
  } else if (pin == MPU_EN_ARM_R) {
    accelgyro.setXAccelOffset(-730);
    accelgyro.setYAccelOffset(802);
    accelgyro.setZAccelOffset(1303);
    accelgyro.setXGyroOffset(87);
    accelgyro.setYGyroOffset(-20);
    accelgyro.setZGyroOffset(28);
  } else if (pin == MPU_EN_TORSO) {
    accelgyro.setXAccelOffset(588);
    accelgyro.setYAccelOffset(4067);
    accelgyro.setZAccelOffset(2070);
    accelgyro.setXGyroOffset(-38);
    accelgyro.setYGyroOffset(75);
    accelgyro.setZGyroOffset(101);
  }
  accelgyro.getMotion6(&(s->accelX), &(s->accelY), &(s->accelZ), &(s->row0), &(s->row1), &(s->row2));

}

void setup_imu() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize device
  pinMode(MPU_EN_ARM_L, OUTPUT);
  pinMode(MPU_EN_ARM_R, OUTPUT);
  pinMode(MPU_EN_TORSO, OUTPUT);

  Serial.println("initialising");
  // verify connection
  bool good = true;
  good = good && initialize(MPU_EN_ARM_L);
  good = good && initialize(MPU_EN_ARM_R);
  good = good && initialize(MPU_EN_TORSO);
  delay(3000);
  if (good) {
    Serial.println("initialisation done");
  }
}


// Code to poll accelerometers
void pollSensor(SensorGroup *s) {
  Serial.println("polling");
  readImu(MPU_EN_ARM_L, &(s->sensor0));
  readImu(MPU_EN_ARM_R, &(s->sensor1));
  readImu(MPU_EN_TORSO, &(s->sensor2));
}



