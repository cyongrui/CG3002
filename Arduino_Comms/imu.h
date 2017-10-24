#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050.h>
#include "serialize.h"
#ifndef IMU_H
#define IMU_H

#define MPU_EN_ARM_L 45
#define MPU_EN_ARM_R 41
#define MPU_EN_TORSO 33 

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

bool activate(uint8_t pin);

bool initialize(uint8_t pin);

void readImu(uint8_t pin, Sensor *s);

void setup_imu();

void pollSensor(SensorGroup *s);

#endif

