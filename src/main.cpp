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
#include "Motors.h"
#include "PID.h"

#ifdef ENABLE_SAR
#include "RobotFSM.h"

RobotFSM_c robot;

#endif

Motors_c motors;
BumpSensors_c bump_sensors;

unsigned long bump_sensor_calibration_time;
unsigned long bump_sensor_update_time;
unsigned long update_time;
unsigned long motor_time;

int results_index = 0;
unsigned long results_interval_ms;
unsigned long record_results_ts;

int pwm = 10;

PID_c left_pid;
PID_c right_pid;
Kinematics_c pose;

int state = 0;

float demand = 0.3;

IMU_c imu;
// The setup() function runs only once when the
// robot is powered up (either by plugging in
// the USB cable, or activating the motor power.
// Use this function to do "once only" setup
// and configuration of your robot.
void setup() {

  // Activates the Serial port, and the delay
  // is used to wait for the connection to be
  // established.
  Serial.begin(9600);
  delay(3000);

  left_pid.initialise(50.0f, 0.1f, 0.0);
  right_pid.initialise(50.0f, 0.1f, 0.0);

  left_pid.reset();
  right_pid.reset();

  setupEncoder0();
  setupEncoder1();
  pose.initialise(0.0f, 0.0f, 0.0f);
  while (!imu.initialise()) {
    delay(1000);
  }

  unsigned long calibration_time = millis();

  while (millis() - calibration_time <= 10000) {
    imu.calibration();
    delay(100);
  }

  motors.initialise();
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
  // delay(5000);
  // buzzer.reset();
  // bump_sensor_update_time = millis();
  update_time = millis();
  motor_time = millis();

  // motors.setPWM(pwm, pwm);
  // motors.setForwards();

  results_interval_ms = ((float)90 * 1000 / (float)MAX_RESULTS);
  record_results_ts = millis();

#ifdef ENABLE_SAR
  robot.initialise();
  robot.setState(RobotFSM_c::State::IDLE);
#endif
}

// put your main code here, to run repeatedly:
void loop() {
#ifdef ENABLE_SAR
  robot.update();
#endif
  imu.update();

  unsigned long elapsed_time;
  elapsed_time = millis() - record_results_ts;

  if (elapsed_time > results_interval_ms && state == 0) {

    // Move time stamp forwards for next
    // iteration.
    record_results_ts = millis();

    // Let's be safe and check we haven't
    // filled up the results array already.
    if (results_index < MAX_RESULTS) {

      results[results_index].pwm = pwm;
      results[results_index].left_speed = pose.speed_left;
      results[results_index].right_speed = pose.speed_right;

      // Increment result index for next time.
      results_index++;
    }
  }

  if (millis() - bump_sensor_update_time >= BUMP_SENSOR_UPDATE_INTERVAL_MS &&
      state == 0) {
    // bump_sensors.calcCalibrated();
    pose.update();
    bump_sensor_update_time = millis();

    // Serial.print(pwm);

    // Serial.print(bump_sensors.calibrated[0] * 10.0, 4);
    // Serial.print(",");
    // Serial.print(bump_sensors.calibrated[1] * 10.0, 4);
    // Serial.print("\n");
  }

  // if (millis() - update_time >= PID_UPDATE_INTERVAL_MS) {
  //   update_time = millis();
  //   float l_pwm = left_pid.update(demand, pose.speed_left);
  //   float r_pwm = right_pid.update(demand, pose.speed_right);

  //   motors.setPWM(l_pwm, r_pwm);
  //   Serial.print(demand * 10.0, 4);
  //   Serial.print(",");
  //   Serial.print(pose.speed_left * 10.0, 4);
  //   Serial.print(",");
  //   Serial.print(pose.speed_right * 10.0, 4);
  //   // Serial.print(",");
  //   // Serial.print(left_pid.p_term, 4);
  //   // Serial.print(",");
  //   // Serial.print(left_pid.i_term, 4);
  //   Serial.print("\n");
  // }

  // if (state == 1) {

  //   int result;
  //   Serial.print(":pwm, left_speed, right_speed\n");
  //   for (result = 0; result < MAX_RESULTS; result++) {
  //     Serial.print(results[result].pwm);
  //     Serial.print(",");
  //     Serial.print(results[result].left_speed);
  //     Serial.print(",");
  //     Serial.print(results[result].right_speed);
  //     Serial.print("\n");
  //   }
  // } else {
  //   if (millis() - motor_time >= 90.0 * 1000) {
  //     state = 1;
  //     motors.setStop();
  //   } else if (millis() - update_time >= 5.0 * 1000) {
  //     pwm += 10;
  //     motors.setPWM(pwm, pwm);
  //     motors.setForwards();
  //     update_time = millis();
  //   }
  // }
}