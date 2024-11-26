/***************************************


****************************************/
#include "Arduino.h"
#include "Configuration.h"

#ifndef _POINT_TRACKING_CONTROLLER_H
#define _POINT_TRACKING_CONTROLLER_H

class PointTrackingController_c {
public:
  // Controller gains
  float K1 = 0.0f; // Gain for linear velocity
  float K2 = 0.0f; // Gain for angular velocity

  float desired_left_speed = 0.0f;
  float desired_right_speed = 0.0f;

  PointTrackingController_c() {}

  void initialise(float k1, float k2) {
    K1 = k1;
    K2 = k2;
  }

  void setGain(float k1, float k2) { initialise(k1, k2); }

  void calculateDesiredSpeed(float cur_x, float cur_y, float cur_theta,
                             float target_x, float target_y) {
    float r = sqrtf(powf(target_x - cur_x, 2) + pow(target_y - cur_y, 2));
    float phi = cur_theta - atan2(target_y - cur_y, target_x - cur_x);

    while (phi > PI)
      phi -= 2 * PI;
    while (phi < -PI)
      phi += 2 * PI;

    float v_c = K1 * r * cos(phi);
    float w_c = -K1 * sin(phi) * cos(phi) - K2 * phi;

    desired_left_speed = (v_c - w_c * WHEEL_RADIUS);
    desired_right_speed = (v_c + w_c * WHEEL_RADIUS);
  }
};

#endif