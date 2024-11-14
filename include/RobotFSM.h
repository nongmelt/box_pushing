/*************************************
  ____   ___  ____   ___ _____   _____ ____  __  __
 |  _ \ / _ \| __ ) / _ \_   _| |  ___/ ___||  \/  |
 | |_) | | | |  _ \| | | || |   | |_  \___ \| |\/| |
 |  _ <| |_| | |_) | |_| || |   |  _|  ___) | |  | |
 |_| \_\\___/|____/ \___/ |_|   |_|   |____/|_|  |_|

*************************************/

#include "Arduino.h"
#include "Configuration.h"

// #include "Buzzer.h"       // Labsheet 0
#include "Kinematics.h" // Labsheet 4
// #include "LED.h"          // Labsheet 0
#include "Motors.h" // Labsheet 1
#include "PID.h"    // Labsheet 1 - Advanced

// Encoders.h does not need modifying.
#include "Encoders.h" // For encoder counts

#ifdef ENABLE_DISPLAY
#include <PololuOLED.h>
PololuSH1106 display(1, 30, 0, 17, 13);
#endif

#ifndef _ROBOT_FSM_H
#define _ROBOT_FSM_H

class RobotFSM_c {

public:
  // Buzzer_c buzzer;
  // LEDYellow_c led_y;

  Motors_c motors;

  Kinematics_c pose;

  // PID Controller
  PID_c left_pid;
  PID_c right_pid;

  enum class State {
    IDLE,
    CALIBRATION,
    EXPERIMENT,
    COMPLETED,
  };

  // Function pointer type for command callbacks
  typedef void (RobotFSM_c::*CommandCallBack)(float);
  typedef bool (RobotFSM_c::*CompletedCallBack)();

  struct Command {
    CommandCallBack cb;
    CompletedCallBack comp;
    float value;
  };

  // Function pointer type for state callbacks
  typedef void (RobotFSM_c::*StateCallBack)();

  RobotFSM_c() : current_state(State::IDLE) {}

  void initialise() {

    // buzzer.initialise();
    // led_y.initialise();

    motors.initialise();

    setupEncoder0();
    setupEncoder1();

    left_pid.initialise(K_P, K_D, K_I);
    right_pid.initialise(K_P, K_D, K_I);

    // Setup the pose (kinematics). This is calling
    // a function within the Kinematics_c class.
    // You can review this within Kinematics.h
    // This is used in Labsheet 4.
    pose.initialise(0.0f, 0.0f, 0.0f);

    // Remember to reset your PID if you have
    // used any delay()
    left_pid.reset();
    right_pid.reset();

    last_pid_update_time = millis();
    last_pose_update_time = millis();

#ifdef ENABLE_DISPLAY
    display.noAutoDisplay();
    display.setLayout21x8();

    last_display_update_time = millis();
#endif
  }

  void setState(State new_state) {
    if (current_state != new_state) {
      StateCallBack exit_cb = getExitCallback(current_state);
      if (exit_cb) {
        (this->*exit_cb)();
      }

      current_state = new_state;

      StateCallBack enter_cb = getEnterCallback(current_state);
      if (enter_cb) {
        (this->*enter_cb)();
      }

      last_update_time = millis();
      enter_state_time = millis();
    }
  }

  void update() {
    // buzzer.update();
    // led_y.update();

    if (millis() - last_pose_update_time >= POSE_EST_INTERVAL_MS) {
      pose.update();
      last_pose_update_time = millis();
    }

    if (millis() - last_pid_update_time >= PID_UPDATE_INTERVAL_MS) {
      last_pid_update_time = millis();
    }
#ifdef ENABLE_DISPLAY
    if (millis() - last_display_update_time >= DISPLAY_INTERVAL_MS) {
      displayUpdate();
      last_display_update_time = millis();
    }
#endif

    StateCallBack update_cb = getUpdateCallback(current_state);

    if (update_cb) {
      (this->*update_cb)();
    }
  }

private:
  State current_state;
  // refresh when enter state, for timer in each state
  unsigned long last_update_time;
  // refresh when enter state, keep track elapsed time
  unsigned long enter_state_time;
  unsigned long last_pid_update_time;
  unsigned long last_pose_update_time;

#ifdef ENABLE_DISPLAY
  unsigned long last_display_update_time;

  const char *stateToString(State state) {
    if (state == State::IDLE) {
      return "IDLE";
    } else if (state == State::CALIBRATION) {
      return "CALIBRATION";
    } else if (state == State::EXPERIMENT) {
      return "EXPERIMENT";
    } else if (state == State::COMPLETED) {
      return "COMPLETED";
    }
    return "UNK";
  }
#endif

  StateCallBack getUpdateCallback(State state) {
    if (state == State::IDLE) {
      return &RobotFSM_c::onIdle;
    } else if (state == State::CALIBRATION) {
      return &RobotFSM_c::onCalibration;
    } else if (state == State::EXPERIMENT) {
      return &RobotFSM_c::onExperiment;
    } else if (state == State::COMPLETED) {
      return &RobotFSM_c::onCompleted;
    }
    return nullptr;
  }

  StateCallBack getEnterCallback(State state) {
    if (state == State::CALIBRATION) {
      return &RobotFSM_c::enterCalibration;
    }
    return nullptr;
  }

  StateCallBack getExitCallback(State state) {
    if (state == State::CALIBRATION) {
      return &RobotFSM_c::exitCalibration;
    }
    return nullptr;
  }

// debugging function
#ifdef ENABLE_DISPLAY
  void displayUpdate() {
    display.gotoXY(0, 0);
    display.print("x:");
    display.print(pose.x);
    display.gotoXY(10, 0);
    display.print("y:");
    display.print(pose.y);
    display.gotoXY(0, 1);
    display.print("th:");
    display.print(pose.theta * RAD_TO_DEG);
    display.gotoXY(17, 7);
    display.print(motors.modeToString(motors.current_mode));
    display.gotoXY(0, 7);
    display.print(stateToString(current_state));
    display.display();
  }
#endif

  void onIdle() {
    if (millis() - enter_state_time >= DEFAULT_IDLE_TIMEOUT) {
      setState(State::CALIBRATION);
    }
  }

  void enterCalibration() {}

  void onCalibration() {}

  void exitCalibration() {}

  void onExperiment() {}

  void onCompleted() {}
};

#endif