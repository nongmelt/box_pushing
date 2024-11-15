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
#include "Kinematics.h"
#include "Motors.h"
#include "PID.h"

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
  const float MM_PER_COUNT = (2.0 * WHEEL_RADIUS * PI) / COUNT_PER_REV;

  unsigned long update_time;
  unsigned long pose_update_time;

  Pusher_c() {}

  void initialise() {
    setupEncoder0();
    setupEncoder1();

    left_pid.initialise(K_P, K_I, K_D);
    right_pid.initialise(K_P, K_I, K_D);

    left_pid.reset();
    right_pid.reset();

    pose.initialise(0.0f, 0.0f, 0.0f);

    Serial.print("pose");
    Serial.print(pose.x);
    Serial.print(",");
    Serial.println(pose.y);
    pose_update_time = millis();
    update_time = millis();
  }

  void setDesiredSpeed(float left_speed, float right_speed) {
    desired_left_speed = left_speed;
    desired_right_speed = right_speed;
  }

  void update() {
    if (millis() - update_time >= PID_UPDATE_INTERVAL_MS) {
      update_time = millis();

      float l_pwm = left_pid.update(desired_left_speed, pose.speed_left);
      float r_pwm = right_pid.update(desired_right_speed, pose.speed_right);

      motors.setPWM(l_pwm, r_pwm);
    }

    if (millis() - pose_update_time >= POSE_EST_INTERVAL_MS) {
      pose_update_time = millis();
      pose.update();
    }
  }
};

#endif