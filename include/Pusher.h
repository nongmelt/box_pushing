/***************************************
  ____            _
 |  _ \ _   _ ___| |__   ___ _ __
 | |_) | | | / __| '_ \ / _ \ '__|
 |  __/| |_| \__ \ | | |  __/ |
 |_|    \__,_|___/_| |_|\___|_|

****************************************/

#include "Arduino.h"
#include "BumpSensors.h"
#include "Configuration.h"
#include "Encoders.h"
#include "IMU.h"
#include "Kinematics.h"
#include "Motors.h"
#include "PID.h"

#ifdef IMPROVEMENT
#include "PointTrackingController.h"
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

  BumpSensors_c bump_sensors;
  unsigned long bump_sensor_update_time;

#ifdef BUMPER_CONTROLLER

  PID_c v_c_pid;
  PID_c w_c_pid;
#endif

#ifdef IMPROVEMENT
  PointTrackingController_c ptc;
  PID_c adaptive_gain_pid;
#endif

  Pusher_c() {}

  void initialise() {
    setupEncoder0();
    setupEncoder1();

    left_pid.initialise(K_P, K_I, K_D);
    right_pid.initialise(K_P, K_I, K_D);

    left_pid.reset();
    right_pid.reset();

#ifdef BUMPER_CONTROLLER
    v_c_pid.initialise(K_P_V_BUMP, K_I_V_BUMP, K_D_V_BUMP);
    w_c_pid.initialise(K_P_R_BUMP, K_I_R_BUMP, K_D_R_BUMP);
    v_c_pid.reset();
    w_c_pid.reset();
#endif

#ifdef IMPROVEMENT
    ptc.initialise(K1_PTC, K2_PTC);
    adaptive_gain_pid.initialise(K_P_BUMP, K_I_BUMP, K_D_BUMP);

#endif

    pose.initialise(0.0f, 0.0f, 0.0f);

    pose_update_time = millis();
    update_time = millis();
    bump_sensor_update_time = millis();
  }

  void setDesiredSpeed(float left_speed, float right_speed) {
    desired_left_speed = max(BASE_SPEED, left_speed);
    desired_right_speed = max(BASE_SPEED, right_speed);
  }

  void update() {
    if (millis() - update_time >= PID_UPDATE_INTERVAL_MS) {
      update_time = millis();

#ifdef BUMPER_CONTROLLER
      float r = sqrtf(powf(GOAL_DISTANCE - pose.x, 2) + pow(0.0f - pose.y, 2));

      float v_c = DEMAND_SPEED;

      v_c += v_c_pid.update(
          BUMP_THRESHOLD,
          (bump_sensors.calibrated[0] + bump_sensors.calibrated[1]) / 2.0f);
      float w_c = w_c_pid.update(0.0f, bump_sensors.calibrated[1] -
                                           bump_sensors.calibrated[0]);

      setDesiredSpeed(v_c - w_c * WHEEL_RADIUS, v_c + w_c * WHEEL_RADIUS);

#endif

#ifdef IMPROVEMENT
      float gain = adaptive_gain_pid.update(
          K1_PTC, bump_sensors.calibrated[0] -
                      bump_sensors.calibrated[1]); // 01 right, 10 left
      ptc.setGain(K1_PTC + gain, K2_PTC);
      ptc.calculateDesiredSpeed(pose.x, pose.y, pose.theta, GOAL_PTC_DISTANCE,
                                0.0f);
      setDesiredSpeed(ptc.desired_left_speed, ptc.desired_right_speed);

#endif

      float l_pwm = left_pid.update(desired_left_speed, pose.speed_left);
      float r_pwm = right_pid.update(desired_right_speed, pose.speed_right);

      motors.setPWM(l_pwm, r_pwm);
    }

    if (millis() - pose_update_time >= POSE_EST_INTERVAL_MS) {
      pose_update_time = millis();
      pose.update();
    }

    if (millis() - bump_sensor_update_time >= BUMP_SENSOR_UPDATE_INTERVAL_MS) {
      bump_sensors.calcCalibrated();
      bump_sensor_update_time = millis();
    }
  }
};

#endif