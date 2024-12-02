/************************************
  ____             __ _                       _   _
 / ___|___  _ __  / _(_) __ _ _   _ _ __ __ _| |_(_) ___  _ __
| |   / _ \| '_ \| |_| |/ _` | | | | '__/ _` | __| |/ _ \| '_ \
| |__| (_) | | | |  _| | (_| | |_| | | | (_| | |_| | (_) | | | |
 \____\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\__|_|\___/|_| |_|
                        |___/
*************************************/
#include "Arduino.h"

#ifndef _CONFIGURATION_H
#define _CONFIGURATION_H

// Experimentation parameters
const int GOAL_DISTANCE = 1000;     // mm
const int GOAL_PTC_DISTANCE = 1220; // mm
const float TOTAL_TIME = 8000.0f;   // ms
const float BASE_SPEED = 0.20f;     // mm/ms
const float DEMAND_SPEED = 0.35f;   // mm/ms, initial speed

#define PUSHER
// #define OBSERVER

// Result
#ifdef PUSHER

// #define BUMPER_CONTROLLER
// #define POINT_TRACKING
#define IMPROVEMENT

struct Result {
  float x;
  float y;
  float theta;
  float left_speed;
  float right_speed;
  float left_bump;
  float right_bump;
};

const uint8_t MAX_RESULTS = 65;

// PID controller for motors speed
#define K_P 35.0f
#define K_D 0.0f
#define K_I 0.5f

#ifdef BUMPER_CONTROLLER

#define K_P_V_BUMP 0.085f // 0.085f - bumper
#define K_I_V_BUMP 0.0f   // 0.0f - bumper
#define K_D_V_BUMP 0.0f

#define K_P_R_BUMP 0.021875f // 0.021875f - bumper
#define K_I_R_BUMP 0.0f      // 0.0f - bumper
#define K_D_R_BUMP 0.0f

#define BUMP_THRESHOLD 0.12f
#endif

#ifdef POINT_TRACKING
#define K1_PTC 0.00065f // 0.00065f - ptc
#define K2_PTC 0.04f    // 0.04f - ptc
#endif

#ifdef IMPROVEMENT
#define K1_PTC 0.00039f // 0.000375f
#define K2_PTC 0.04f    // 0.04f

#define K_P_BUMP 0.0005f
#define K_I_BUMP 0.0f
#define K_D_BUMP 0.0f
#endif

#endif

#ifdef OBSERVER

struct Result {
  float heading;
};

const uint8_t MAX_RESULTS = 250;

#endif

Result results[MAX_RESULTS];

// PIN definitions.
#define BUZZER_PIN 6

#define LED_YELLOW_PIN LED_BUILTIN
#define LED_GREEN_PIN 30
#define LED_RED_PIN 17

#define MOTOR_LEFT_PWM_PIN 10
#define MOTOR_LEFT_DIR_PIN 16
#define MOTOR_RIGHT_PWM_PIN 9
#define MOTOR_RIGHT_DIR_PIN 15

#define BUMP_LEFT_PIN A6
#define BUMP_RIGHT_PIN 5

#define BUMP_SENSORS_NUM 2
// Left, Right
const int BUMP_SENSOR_PINS[BUMP_SENSORS_NUM] = {BUMP_LEFT_PIN, BUMP_RIGHT_PIN};
// const float RANGE[BUMP_SENSORS_NUM] = {782.4, 916.0};
// const float MINIMUM[BUMP_SENSORS_NUM] = {506.4, 704.0};

const float RANGE[BUMP_SENSORS_NUM] = {1392.8, 1490.4};
const float MINIMUM[BUMP_SENSORS_NUM] = {632.8, 667.2};

// Motor parameters
// It is a good idea to limit the maximum power
// sent to the motors. Using #define means we
// can set this value just once here, and it
// can be used in many places in the code below.
#define MOTOR_MAXIMUM_PWM 180.0f
#define MOTOR_MINIMUM_PWM 0.0f
#define MOTOR_FWD LOW
#define MOTOR_REV HIGH

// Line sensors parameters
// We will use all 5 line sensors (DN1 - 5)
// and so define a constant here, rather than
// type '5' in lots of places.
#define LINE_SENSORS_NUM 5
// This is the pin used to turn on the infra-
// red LEDs.
#define EMIT_PIN 11
// Pin definitions
// This time, we will use an array to store the
// pin definitions.  This is a bit like a list.
// This way, we can either loop through the
// list automatically, or we can ask for a pin
// by indexing, e.g. sensor_pins[0] is A11: leftmost,
// sensors_pins[1] is A0.
const int LINE_SENSOR_PINS[LINE_SENSORS_NUM] = {A11, A0, A2, A3, A4};

// Robot dimensions.  You will need
// to calibrate these to get the best
// performance. (see Lab sheet 4)
#define COUNT_PER_REV 358.3 // From documentation - correct.
#define WHEEL_RADIUS 16.0   // mm
#define WHEEL_SEP 43.5      // mm, from centre of robot to wheel centre
const float MM_PER_COUNT = (2.0 * WHEEL_RADIUS * PI) / COUNT_PER_REV;

// Update rates and durations for operations
#define SPEED_EST_INTERVAL_MS 10
#define PID_UPDATE_INTERVAL_MS 50
#define CALIBRATION_INTERVAL_MS 10
#define POSE_EST_INTERVAL_MS 10
#define BUMP_SENSOR_UPDATE_INTERVAL_MS 10
#define IMU_UPDATE_INTERVAL_MS 100
#define HEADING_UPDATE_INTERVAL_MS 105

// Finite State Machine Configuration
static const uint16_t DEFAULT_IDLE_TIMEOUT = 2000;
static const uint32_t DEFAULT_SEARCH_TIMEOUT = 120000; // 2 mins

template <typename T> T clamp(const T &value, const T &min, const T &max) {
  return (value < min) ? min : (value > max) ? max : value;
}

#endif