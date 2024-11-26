/***************************************
   ___  _
  / _ \| |__  ___  ___ _ ____   _____ _ __
 | | | | '_ \/ __|/ _ \ '__\ \ / / _ \ '__|
 | |_| | |_) \__ \  __/ |   \ V /  __/ |
  \___/|_.__/|___/\___|_|    \_/ \___|_|

****************************************/

#include "Arduino.h"
#include "Buzzer.h"
#include "Configuration.h"
#include "IMU.h"
#include "Magnetometer.h"

#ifndef _OBSERVER_H
#define _OBSERVER_H

class Observer_c {
public:
  IMU_c imu;
  Magnetometer_c mag;
  Buzzer_c buzzer;

  float roll = 0.0, pitch = 0.0, yaw = 0.0;
  float prev_yaw = 0.0;
  float roll_g = 0.0, pitch_g = 0.0, yaw_g = 0.0;

  unsigned long update_time;

  Observer_c() {}

  void initialise() {
    Wire.begin();
    while (!imu.initialise()) {
      delay(1000);
    }

    while (!mag.initialise()) {
      delay(1000);
    }

    buzzer.initialise();
  }

  void calibration() {
    unsigned long calibration_time = millis();

    while (millis() - calibration_time <= 3000) {
      mag.calibration();
      imu.calibration();
      delay(100);
    }
    mag.postCalibrated();
    buzzer.setBeepOnce(3, 250);
  }

  void update() {
    buzzer.update();
    imu.update();

    unsigned long elapsed_time = millis() - update_time;
    if (elapsed_time >= HEADING_UPDATE_INTERVAL_MS) {
      mag.calcCalibrated();
      getHeading(elapsed_time);
      update_time = millis();
    }
  }

  void getHeading(unsigned long dt) {
    yaw_g += imu.calibrated[5] * dt / 1000.0;
    // float roll_acc = atan2(imu.calibrated[1], imu.calibrated[2]); // rad
    // float pitch_acc =
    //     atan2(-imu.calibrated[0], sqrtf(imu.calibrated[1] * imu.calibrated[1]
    //     +
    //                                     imu.calibrated[2] *
    //                                     imu.calibrated[2]));

    // float roll_mag = mag.calibrated[0] * cos(roll_acc) +
    //                  mag.calibrated[1] * sin(roll_acc) * sin(pitch_acc) +
    //                  mag.calibrated[2] * sin(roll_acc) * cos(pitch_acc);
    // float pitch_mag =
    //     mag.calibrated[1] * cos(pitch_acc) - mag.calibrated[2] *
    //     sin(pitch_acc);
    // float yaw_mag = -mag.calibrated[0] * sin(roll_acc) +
    //                 mag.calibrated[1] * cos(roll_acc) * sin(pitch_acc) +
    //                 mag.calibrated[2] * cos(roll_acc) * cos(pitch_acc);

    // float yaw_acc = atan2(-pitch_mag, roll_mag);
    // roll_g += imu.calibrated[3] * dt / 1000.0;
    // pitch_g += imu.calibrated[4] * dt / 1000.0;
    // yaw_g += imu.calibrated[5] * dt / 1000.0;

    // roll = ALPHA * roll_g + (1 - ALPHA) * roll_acc;
    // pitch = ALPHA * pitch_g + (1 - ALPHA) * pitch_acc;
    // yaw = ALPHA * yaw_g + (1 - ALPHA) * yaw_acc;
  }

private:
  constexpr static float ALPHA = 1.0;
};

#endif