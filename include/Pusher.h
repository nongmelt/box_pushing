/***************************************
  ____            _
 |  _ \ _   _ ___| |__   ___ _ __
 | |_) | | | / __| '_ \ / _ \ '__|
 |  __/| |_| \__ \ | | |  __/ |
 |_|    \__,_|___/_| |_|\___|_|

****************************************/

#include "Arduino.h"
#include "Configuration.h"
#include "Encoders.h"
#include "IMU.h"
#include "Kinematics.h"
#include "Motors.h"
#include "PID.h"

#ifdef IMPROVEMENT
#include "BumpSensors.h"

#endif

#ifndef _PUSHER_H
#define _PUSHER_H

class Pusher_c {
public:
  // Controller gains
  PID_c left_pid;
  PID_c right_pid;
  // PID_c resist_pid;

  Motors_c motors;

  Kinematics_c pose;

  // IMU_c imu;

  float desired_left_speed = 0.0f;
  float desired_right_speed = 0.0f;
  const float MM_PER_COUNT = (2.0 * WHEEL_RADIUS * PI) / COUNT_PER_REV;

  unsigned long update_time;
  unsigned long pose_update_time;

#ifdef IMPROVEMENT
  BumpSensors_c bump_sensors;
  unsigned long bump_sensor_update_time;

  PID_c bump_pid;
#endif

  Pusher_c() {}

  void initialise() {
    setupEncoder0();
    setupEncoder1();

    // Wire.begin();
    // while (!imu.initialise()) {
    //   delay(1000);
    // }

    left_pid.initialise(K_P, K_I, K_D);
    right_pid.initialise(K_P, K_I, K_D);
    // resist_pid.initialise(K_P_GYRO, K_I_GYRO, K_D_GYRO);

    left_pid.reset();
    right_pid.reset();
    // resist_pid.reset();

#ifdef IMPROVEMENT
    bump_pid.initialise(K_P_BUMP, K_I_BUMP, K_D_BUMP);
    bump_pid.reset();
    bump_sensor_update_time = millis();
#endif

    pose.initialise(0.0f, 0.0f, 0.0f);

    pose_update_time = millis();
    update_time = millis();
  }

  void setDesiredSpeed(float left_speed, float right_speed) {
    desired_left_speed = left_speed;
    desired_right_speed = right_speed;
  }

  void update() {
    // imu.update();
    if (millis() - update_time >= PID_UPDATE_INTERVAL_MS) {
      update_time = millis();

      float l_pwm = left_pid.update(desired_left_speed, pose.speed_left);
      float r_pwm = right_pid.update(desired_right_speed, pose.speed_right);
      // float gyro_correction = resist_pid.update(0.0f, imu.calibrated[5]);

      // l_pwm -= gyro_correction;
      // r_pwm += gyro_correction;

#ifdef IMPROVEMENT

      float bump_correction = bump_pid.update(
          0.0f, bump_sensors.calibrated[0] - bump_sensors.calibrated[1]);

      l_pwm += bump_correction;
      r_pwm -= bump_correction;
#endif

      motors.setPWM(l_pwm, r_pwm);
    }

    if (millis() - pose_update_time >= POSE_EST_INTERVAL_MS) {
      pose_update_time = millis();
      pose.update();
    }

#ifdef IMPROVEMENT
    if (millis() - bump_sensor_update_time >= BUMP_SENSOR_UPDATE_INTERVAL_MS) {
      bump_sensors.calcCalibrated();
      bump_sensor_update_time = millis();
    }
#endif
  }
};

#endif