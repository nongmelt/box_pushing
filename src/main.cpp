/******************************************
 ______       _                _
(_____ \     | |           _  (_)
 _____) )___ | |__   ___ _| |_ _  ____
|  __  // _ \|  _ \ / _ (_   _) |/ ___)
| |  \ \ |_| | |_) ) |_| || |_| ( (___
|_|   |_\___/|____/ \___/  \__)_|\____)

  ______
 / _____)             _
( (____  _   _  ___ _| |_ _____ ____   ___
 \____ \| | | |/___|_   _) ___ |    \ /___)
 _____) ) |_| |___ | | |_| ____| | | |___ |
(______/ \__  (___/   \__)_____)_|_|_(___/
        (____/
 EMATM0054/53: University of Bristol.
 https://github.com/paulodowd/EMATM0054_53
*******************************************/

#include "Arduino.h"
#include "BumpSensors.h"
#include "Buzzer.h"
#include "Encoders.h"
#include "IMU.h"
#include "Kinematics.h"
#include "Magnetometer.h"
#include "Motors.h"
#include "PID.h"
#include "Wire.h"

#ifdef ENABLE_SAR
#include "RobotFSM.h"

RobotFSM_c robot;

#endif

void getHeading(unsigned long dt);
void displayUpdate();

Motors_c motors;
BumpSensors_c bump_sensors;
Magnetometer_c mag;

unsigned long bump_sensor_calibration_time;
unsigned long bump_sensor_update_time;
unsigned long update_time;
unsigned long motor_time;
unsigned long last_display_update_time;
unsigned long pid_update_time;

int results_index = 0;
float results_interval_mm;
float record_results_ds;

int pwm = 10;
constexpr float ALPHA = 0.9;

float roll = 0.0, pitch = 0.0, yaw = 0.0;
float roll_g = 0.0, pitch_g = 0.0, yaw_g = 0.0;

const int GOAL_DISTANCE = 500; // mm

PID_c left_pid;
PID_c right_pid;
PID_c rotate_resist_pid;
PID_c bump_pid;
Kinematics_c pose;

int state = 0;

float demand = 0.5; // count / ms

IMU_c imu;

#ifdef ENABLE_DISPLAY
#include <PololuOLED.h>
PololuSH1106 display(1, 30, 0, 17, 13);
#endif

void setup() {

  Serial.begin(9600);
  delay(2000);

  left_pid.initialise(5.0f, 0.1f, 0.0);
  right_pid.initialise(5.0f, 0.1f, 0.0);
  rotate_resist_pid.initialise(1.8f, 0.0f, 0.0f);
  bump_pid.initialise(30.0f, 0.1f, 0.0f);

  left_pid.reset();
  right_pid.reset();
  rotate_resist_pid.reset();
  bump_pid.reset();

  setupEncoder0();
  setupEncoder1();
  pose.initialise(0.0f, 0.0f, 0.0f);

  // Wire.begin();
  // while (!imu.initialise()) {
  //   delay(1000);
  // }

  // while (!mag.initialise()) {
  //   delay(1000);
  // }

  // unsigned long calibration_time = millis();

  // while (millis() - calibration_time <= 5000) {
  //   mag.calibration();
  //   delay(100);
  // }
  // mag.postCalibrated();

  // motors.initialise();
  // bump_sensors.initialiseForDigital();

  // Buzzer_c buzzer;
  // buzzer.initialise();

  // buzzer.setBeepOnce(1, 250);
  // bump_sensor_calibration_time = millis();
  // while (millis() - bump_sensor_calibration_time < 3000) {
  //   bump_sensors.calibration();
  //   buzzer.update();
  //   delay(10);
  // }
  // buzzer.setBeepOnce(1, 1000);
  // delay(1000);
  // buzzer.reset();
  // delay(2000);
  // buzzer.setBeepOnce(1, 250);

  // bump_sensor_calibration_time = millis();
  // while (millis() - bump_sensor_calibration_time < 3000) {
  //   bump_sensors.calibration();
  //   buzzer.update();
  //   delay(10);
  // }

  // bump_sensors.postCalibrated();

  // buzzer.setBeepOnce(1, 1000);
  // delay(2500);
  // buzzer.reset();
  // bump_sensor_update_time = millis();
  // update_time = millis();
  // motor_time = millis();
  // pid_update_time = millis();

  // motors.setPWM(pwm, pwm);
  // motors.setForwards();

  results_interval_mm = ((float)GOAL_DISTANCE / (float)MAX_RESULTS);
  record_results_ds = pose.x;

#ifdef ENABLE_SAR
  robot.initialise();
  robot.setState(RobotFSM_c::State::IDLE);
#endif

#ifdef ENABLE_DISPLAY
  display.noAutoDisplay();
  display.setLayout21x8();

  last_display_update_time = millis();
#endif
}

// put your main code here, to run repeatedly:
void loop() {
#ifdef ENABLE_SAR
  robot.update();
#endif
  imu.update();

  float elapsed_distance;
  elapsed_distance = pose.x - record_results_ds;

  if (elapsed_distance > results_interval_mm && state == 0) {

    // Move time stamp forwards for next
    // iteration.
    record_results_ds = pose.x;

    // Let's be safe and check we haven't
    // filled up the results array already.
    if (results_index < MAX_RESULTS) {

      results[results_index].x = pose.x;
      results[results_index].y = pose.y;
      results[results_index].theta = pose.theta;

      // Increment result index for next time.
      results_index++;
    }
  }

  // elapsed_time = millis() - update_time;
  // if (elapsed_time >= 10) {
  //   getHeading(elapsed_time);
  //   update_time = millis();
  // }

  // if (millis() - bump_sensor_update_time >= BUMP_SENSOR_UPDATE_INTERVAL_MS &&
  //     state == 0) {
  //   bump_sensors.calcCalibrated();
  //   pose.update();
  //   bump_sensor_update_time = millis();

  //   // Serial.print(pwm);

  //   // Serial.print(bump_sensors.calibrated[0] * 10.0, 4);
  //   // Serial.print(",");
  //   // Serial.print(bump_sensors.calibrated[1] * 10.0, 4);
  //   // Serial.print("\n");
  // }

  if (millis() - pid_update_time >= PID_UPDATE_INTERVAL_MS) {
    pid_update_time = millis();
    float l_pwm = left_pid.update(demand, pose.speed_left);
    float r_pwm = right_pid.update(demand, pose.speed_right);
    // float rotate_correction = rotate_resist_pid.update(0.0f,
    // imu.calibrated[5]);
    float rotate_correction = 0.0f;
    // float bump_correction = bump_pid.update(
    //     0.0f, bump_sensors.calibrated[0] - bump_sensors.calibrated[1]);
    float bump_correction = 0.0f;

    motors.setPWM(l_pwm - rotate_correction + bump_correction,
                  r_pwm + rotate_correction - bump_correction);
  }

  if (state == 1) {

    int result;
    Serial.print(":x, y, theta\n");
    for (result = 0; result < MAX_RESULTS; result++) {
      Serial.print(results[result].x);
      Serial.print(",");
      Serial.print(results[result].y);
      Serial.print(",");
      Serial.print(results[result].theta);
      Serial.print("\n");
    }
  } else {
    if (pose.x >= GOAL_DISTANCE) {
      state = 1;
      motors.setStop();
    }
  }

#ifdef ENABLE_DISPLAY
  if (millis() - last_display_update_time >= DISPLAY_INTERVAL_MS) {
    displayUpdate();
    last_display_update_time = millis();
  }
#endif
}

void getHeading(unsigned long dt) {
  float roll_acc = atan2(imu.calibrated[1], imu.calibrated[2]);
  float pitch_acc =
      atan2(-imu.calibrated[0], sqrtf(imu.calibrated[1] * imu.calibrated[1] +
                                      imu.calibrated[2] * imu.calibrated[2]));

  float roll_mag = mag.calibrated[0] * cos(roll_acc) +
                   mag.calibrated[1] * sin(roll_acc) * sin(pitch_acc) +
                   mag.calibrated[2] * sin(roll_acc) * cos(pitch_acc);
  float pitch_mag =
      mag.calibrated[1] * cos(pitch_acc) - mag.calibrated[2] * sin(pitch_acc);
  float yaw_mag = -mag.calibrated[0] * sin(roll_acc) +
                  mag.calibrated[1] * cos(roll_acc) * sin(pitch_acc) +
                  mag.calibrated[2] * cos(roll_acc) * cos(pitch_acc);

  float yaw_acc = atan2(-pitch_mag, roll_mag);
  roll_g += imu.calibrated[3] * dt / 1000;
  pitch_g += imu.calibrated[4] * dt / 1000;
  yaw_g += imu.calibrated[5] * dt / 1000;

  roll = ALPHA * roll_g + (1 - ALPHA) * roll_acc;
  pitch = ALPHA * pitch_g + (1 - ALPHA) * pitch_acc;
  yaw = ALPHA * yaw_g + (1 - ALPHA) * yaw_acc;
}

#ifdef ENABLE_DISPLAY
void displayUpdate() {
  display.gotoXY(0, 0);
  display.print("yaw:");
  display.print(yaw);
  display.gotoXY(0, 2);
  display.print("g_z:");
  display.print(imu.calibrated[5]);
  display.gotoXY(0, 4);
  display.print("bl:");
  display.print(bump_sensors.calibrated[0]);
  display.gotoXY(0, 5);
  display.print("mx:");
  display.print(bump_sensors.maximum[0]);
  display.gotoXY(10, 5);
  display.print("mn:");
  display.print(bump_sensors.minimum[0]);
  display.gotoXY(0, 6);
  display.print("bl:");
  display.print(bump_sensors.calibrated[1]);
  display.gotoXY(0, 7);
  display.print("mx:");
  display.print(bump_sensors.maximum[1]);
  display.gotoXY(10, 7);
  display.print("mn:");
  display.print(bump_sensors.minimum[1]);
  display.display();
}
#endif