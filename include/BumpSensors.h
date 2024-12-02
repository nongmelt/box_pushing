/***************************************
  ____                       ____
 | __ ) _   _ _ __ ___  _ __/ ___|  ___ _ __  ___  ___  _ __
 |  _ \| | | | '_ ` _ \| '_ \___ \ / _ \ '_ \/ __|/ _ \| '__|
 | |_) | |_| | | | | | | |_) |__) |  __/ | | \__ \ (_) | |
 |____/ \__,_|_| |_| |_| .__/____/ \___|_| |_|___/\___/|_|
                       |_|
****************************************/
#include "Arduino.h"
#include "Configuration.h"
// this #ifndef stops this file
// from being included more than
// once by the compiler.
#ifndef _BUMP_SENSORS_H
#define _BUMP_SENSORS_H

// Class to operate the bump sensor.
class BumpSensors_c {

public:
  float readings[BUMP_SENSORS_NUM];

  // float minimum[BUMP_SENSORS_NUM];
  // float maximum[BUMP_SENSORS_NUM];
  // float range[BUMP_SENSORS_NUM];

  float calibrated[BUMP_SENSORS_NUM];

  BumpSensors_c() {}

  void initialiseForDigital() {
    pinMode(EMIT_PIN, INPUT);
    // Configure the bump sensor pins
    for (int sensor = 0; sensor < BUMP_SENSORS_NUM; sensor++) {
      pinMode(BUMP_SENSOR_PINS[sensor], INPUT);
      // maximum[sensor] = 0.0;
      // minimum[sensor] = DEFAULT_TIMEOUT;
    }

  } // End of initialiseForDigital()

  void readSensors() { readSensorsDigital(); }

  // void calibration() {
  //   readSensors();
  //   for (int sensor = 0; sensor < BUMP_SENSORS_NUM; sensor++) {
  //     float cur_reading = readings[sensor];

  //     if (cur_reading > maximum[sensor]) {
  //       maximum[sensor] = cur_reading;
  //     }

  //     if (cur_reading < minimum[sensor]) {
  //       minimum[sensor] = cur_reading;
  //     }
  //   }
  // }
  void calcCalibrated() { calcCalibratedDigital(); }

  // void postCalibrated() {
  //   for (int sensor = 0; sensor < BUMP_SENSORS_NUM; sensor++) {
  //     range[sensor] = maximum[sensor] - minimum[sensor];
  //   }
  // }

private:
  static const unsigned long DEFAULT_TIMEOUT = 4000;

  void readSensorsDigital() {
    // Enable IR for bump sensors
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, LOW);

    for (int sensor = 0; sensor < BUMP_SENSORS_NUM; sensor++) {
      pinMode(BUMP_SENSOR_PINS[sensor], OUTPUT);
      digitalWrite(BUMP_SENSOR_PINS[sensor], HIGH);
      readings[sensor] = DEFAULT_TIMEOUT;
    }
    delayMicroseconds(10);

    // Capture start time in microseconds
    unsigned long start_time = micros();

    for (int sensor = 0; sensor < BUMP_SENSORS_NUM; sensor++) {
      pinMode(BUMP_SENSOR_PINS[sensor], INPUT);
    }
    while (true) {
      unsigned long time = micros() - start_time;
      if (time >= DEFAULT_TIMEOUT) {
        break;
      }
      for (int sensor = 0; sensor < BUMP_SENSORS_NUM; sensor++) {
        if (digitalRead(BUMP_SENSOR_PINS[sensor]) == LOW &&
            time < readings[sensor]) {
          readings[sensor] = time;
        }
      }
    }

    // Disable it when finished
    pinMode(EMIT_PIN, INPUT);

  } // End of readSensorsDigital()

  void calcCalibratedDigital() {

    readSensorsDigital();

    // Apply calibration values, store in calibrated[]
    for (int sensor = 0; sensor < BUMP_SENSORS_NUM; sensor++) {
      calibrated[sensor] = (readings[sensor] - MINIMUM[sensor]) / RANGE[sensor];
    }
  }
}; // End of BumpSensors_c class defintion

#endif
