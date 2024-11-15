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
#include "Buzzer.h"
#include "PointTrackingController.h"

// void displayUpdate();

// BumpSensors_c bump_sensors;
// Magnetometer_c mag;

unsigned long update_time;
// unsigned long motor_time;
// unsigned long last_display_update_time;

#ifdef PUSHER
#include "Pusher.h"

Pusher_c pusher;

float results_interval_mm;
float record_results_ds;

#endif

#ifdef OBSERVER
#include "Observer.h"

Observer_c observer;

unsigned long results_interval_ms;
unsigned long record_results_ts;

#endif

int results_index = 0;

// PID_c rotate_resist_pid;
// PID_c bump_pid;

// PointTrackingController_c ptc;

uint8_t state = 0;

#ifdef ENABLE_DISPLAY
#include <PololuOLED.h>
PololuSH1106 display(1, 30, 0, 17, 13);
#endif

void setup() {

  Serial.begin(9600);
  delay(2000);

  // rotate_resist_pid.initialise(1.8f, 0.0f, 0.0f);
  // bump_pid.initialise(30.0f, 0.1f, 0.0f);
  // ptc.initialise(0.0005f, 0.01f);

#ifdef PUSHER
  pusher.initialise();

  results_interval_mm = ((float)GOAL_DISTANCE / (float)MAX_RESULTS);
  record_results_ds = pusher.pose.x;
  pusher.setDesiredSpeed(DEMAND_SPEED, DEMAND_SPEED);

#ifdef IMPROVEMENT

  pusher.bump_sensors.initialiseForDigital();

  Buzzer_c buzzer;
  buzzer.initialise();

  buzzer.setBeepOnce(3, 250);
  unsigned long bump_sensor_calibration_time = millis();
  while (millis() - bump_sensor_calibration_time < 3000) {
    pusher.bump_sensors.calibration();
    buzzer.update();
    delay(10);
  }
  buzzer.setBeepOnce(2, 1000);
  delay(1000);
  buzzer.reset();

  pusher.bump_sensors.postCalibrated();

#endif
#endif

#ifdef OBSERVER
  observer.initialise();

  results_interval_ms = (TOTAL_TIME / (float)MAX_RESULTS);
  record_results_ts = millis();

  observer.calibration();
#endif

  // rotate_resist_pid.reset();
  // bump_pid.reset();

  // ptc.calculateDesiredSpeed(pose.x, pose.y, pose.theta, GOAL_DISTANCE,
  //                           GOAL_DISTANCE);
  // demand_left = ptc.desired_left_speed;
  // demand_right = ptc.desired_right_speed;

  // Buzzer_c buzzer;
  // buzzer.initialise();

  // bump_sensor_update_time = millis();
  // update_time = millis();
  // motor_time = millis();
  // pid_update_time = millis();

  // motors.setPWM(pwm, pwm);
  // motors.setForwards();

#ifdef ENABLE_DISPLAY
  display.noAutoDisplay();
  display.setLayout21x8();

  last_display_update_time = millis();
#endif
}

// put your main code here, to run repeatedly:
void loop() {

#ifdef PUSHER
  float elapsed_distance = pusher.pose.x - record_results_ds;

  if (elapsed_distance > results_interval_mm && state == 0) {

    // Move time stamp forwards for next
    // iteration.
    record_results_ds = pusher.pose.x;

    // Let's be safe and check we haven't
    // filled up the results array already.
    if (results_index < MAX_RESULTS) {

      results[results_index].x = pusher.pose.x;
      results[results_index].y = pusher.pose.y;
      results[results_index].theta = pusher.pose.theta;

      // Increment result index for next time.
      results_index++;
    }
  }

  if (state == 0) {
    if (pusher.pose.x > GOAL_DISTANCE) {
      state = 1;
      pusher.motors.setStop();
    } else {
      pusher.update();
    }

  } else if (state == 1) {

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
  }

#endif

#ifdef OBSERVER
  unsigned long elapsed_time = millis() - record_results_ts;

  if (elapsed_time > results_interval_ms && state == 0) {

    // Move time stamp forwards for next
    // iteration.
    record_results_ts = millis();

    // Let's be safe and check we haven't
    // filled up the results array already.
    if (results_index < MAX_RESULTS) {

      results[results_index].heading = observer.yaw;

      // Increment result index for next time.
      results_index++;
    }
  }
  if (state == 0) {

    // if (results_index >= MAX_RESULTS) {
    //   state = 1;
    // }
    observer.update();

    Serial.print(observer.roll);
    Serial.print(",");
    Serial.print(observer.pitch);
    Serial.print(",");
    Serial.print(observer.yaw);
    Serial.print("\n");
  } else if (state == 1) {
    int result;
    Serial.print(":heading\n");
    for (result = 0; result < MAX_RESULTS; result++) {
      Serial.print(results[result].heading);
      Serial.print("\n");
    }
  }
#endif

#ifdef ENABLE_DISPLAY
  if (millis() - last_display_update_time >= DISPLAY_INTERVAL_MS) {
    displayUpdate();
    last_display_update_time = millis();
  }
#endif
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