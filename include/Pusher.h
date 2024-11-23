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

  Motors_c motors;

  Kinematics_c pose;

  float desired_left_speed = 0.0f;
  float desired_right_speed = 0.0f;

  unsigned long update_time;
  unsigned long pose_update_time;

#ifdef IMPROVEMENT
  BumpSensors_c bump_sensors;
  unsigned long bump_sensor_update_time;

  PID_c v_c_pid;
  PID_c w_c_pid;
#endif

  Pusher_c() {}

  void initialise() {
    setupEncoder0();
    setupEncoder1();

    left_pid.initialise(K_P, K_I, K_D);
    right_pid.initialise(K_P, K_I, K_D);

    left_pid.reset();
    right_pid.reset();

#ifdef IMPROVEMENT
    v_c_pid.initialise(K_P_V_BUMP, K_I_V_BUMP, K_D_V_BUMP);
    w_c_pid.initialise(K_P_R_BUMP, K_I_R_BUMP, K_D_R_BUMP);
    v_c_pid.reset();
    w_c_pid.reset();
    bump_sensor_update_time = millis();
#endif

    pose.initialise(0.0f, 0.0f, 0.0f);

    pose_update_time = millis();
    update_time = millis();
  }

  void setDesiredSpeed(float left_speed, float right_speed) {
    desired_left_speed = max(BASE_SPEED, left_speed);
    desired_right_speed = max(BASE_SPEED, right_speed);
  }

  void update() {
    if (millis() - update_time >= PID_UPDATE_INTERVAL_MS) {
      update_time = millis();

#ifdef IMPROVEMENT
      float r = sqrtf(powf(GOAL_DISTANCE - pose.x, 2) + pow(0.0f - pose.y, 2));

      // BASE SPEED for bumper controller
      // Assuming that leader goes straight
      float v_c = K1_PTC * r;

      v_c += v_c_pid.update(
          BUMP_THRESHOLD,
          (bump_sensors.calibrated[0] + bump_sensors.calibrated[1]) / 2.0f);
      float w_c = w_c_pid.update(0.0f, bump_sensors.calibrated[1] -
                                           bump_sensors.calibrated[0]);

      setDesiredSpeed(v_c - w_c * WHEEL_RADIUS, v_c + w_c * WHEEL_RADIUS);

      Serial.print("\n");

#endif

      float l_pwm = left_pid.update(desired_left_speed, pose.speed_left);
      float r_pwm = right_pid.update(desired_right_speed, pose.speed_right);

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