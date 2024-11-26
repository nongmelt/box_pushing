/***************************************
  ___ __  __ _   _
 |_ _|  \/  | | | |
  | || |\/| | | | |
  | || |  | | |_| |
 |___|_|  |_|\___/

****************************************/
#include "Arduino.h"
#include "Configuration.h"
#include "Magnetometer.h"

#include <LSM6.h>
#include <Wire.h>

#ifndef _IMU_H
#define _IMU_H

class IMU_c {
public:
  LSM6 imu;

  float readings[6];
  float g_mean[3];
  float calibrated[6];
  float prev_a_calibrated[3];
  float prev_a_reading[3];
  float prev_g_calibrated[3];
  float n;

  IMU_c() {}

  bool initialise() {

    if (!imu.init()) {
      return false;
    } else {

      imu.enableDefault();

      update_time = millis();
      n = 0.0f;

      for (size_t i = 0; i < 3; i++) {
        g_mean[i] = 0.0;
        prev_a_calibrated[i] = 0;
        prev_a_reading[i] = 0;
      }

      return true;
    }
  } // End of initialise()

  void getReadings() {
    imu.read();

    readings[0] = imu.a.x * 0.061 / 1000 * 9.807; // m/s^2
    readings[1] = imu.a.y * 0.061 / 1000 * 9.807; // m/s^2
    readings[2] = imu.a.z * 0.061 / 1000 * 9.807; // m/s^2
    readings[3] = imu.g.x * 8.75 / 1000;          // deg/s
    readings[4] = imu.g.y * 8.75 / 1000;          // deg/s
    readings[5] = imu.g.z * 8.75 / 1000;          // deg/s
  }

  void calibration() {
    getReadings();
    n++;
    for (size_t i = 0; i < 3; i++) {
      g_mean[i] = g_mean[i] - (g_mean[i] / n) + ((float)readings[i + 3] / n);
    }
  }

  void calcCalibrated() {
    getReadings();
    for (size_t i = 0; i < 3; i++) {
      calibrated[i] =
          highPassFilter(readings[i], prev_a_reading[i], prev_a_calibrated[i]);
      prev_a_calibrated[i] = calibrated[i];
      prev_a_reading[i] = readings[i];
      calibrated[i + 3] =
          lowPassFilter(readings[i + 3] - g_mean[i], prev_g_calibrated[i]);
      prev_g_calibrated[i] = calibrated[i + 3];
    }
  }

  void update() {
    unsigned long elapsed_time = millis() - update_time;
    if (elapsed_time >= IMU_UPDATE_INTERVAL_MS) {
      calcCalibrated();
      update_time = millis();
    }
  }

private:
  unsigned long update_time;
  static constexpr float LPF_ALPHA = 0.1;
  static constexpr float HPF_ALPHA = 0.8;

  float lowPassFilter(float cur, float prev) {
    return (LPF_ALPHA * cur + (1.0 - LPF_ALPHA) * prev);
  }

  float highPassFilter(float cur, float prev, float filtered) {
    return HPF_ALPHA * (filtered + cur - prev);
  }
};

#endif