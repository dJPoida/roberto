/*============================================================================*\
 * Roberto Drive and LED Controller - Drive Control
 * Peter Eldred 2024-01
 * 
 * Handles the drive motor control functions
 *============================================================================*/
#include <Arduino.h>
#include <FastLED.h>
#include "config.h"
#include "driveControl.h"

/**
 * Initialise
 */
bool DriveControl::init() {
  #ifdef SERIAL_DEBUG
  Serial.print("Initialising Drive Controller...");
  #endif

  // Set the defaults
  driveSpeedVal = 0;
  steerVal = 0;
  strafeVal = 0;

  // Init the pins
  pinMode(PIN_MOTOR_FL_PWM, OUTPUT);
  pinMode(PIN_MOTOR_FL_FWD, OUTPUT);
  pinMode(PIN_MOTOR_FL_REV, OUTPUT);
  pinMode(PIN_MOTOR_FR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_FR_FWD, OUTPUT);
  pinMode(PIN_MOTOR_FR_REV, OUTPUT);
  pinMode(PIN_MOTOR_RL_PWM, OUTPUT);
  pinMode(PIN_MOTOR_RL_FWD, OUTPUT);
  pinMode(PIN_MOTOR_RL_REV, OUTPUT);
  pinMode(PIN_MOTOR_RR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_RR_FWD, OUTPUT);
  pinMode(PIN_MOTOR_RR_REV, OUTPUT);

  #ifdef SERIAL_DEBUG
  Serial.println(" done.");
  #endif

  return true;  
}


/**
 * Constructor
 */
DriveControl::DriveControl(){
}


/**
 * Emergency stop - clear all input states
 */
void DriveControl::emergencyStop() {
  #ifdef SERIAL_DEBUG
  Serial.println("== EMERGENCY STOP ==");
  #endif

  driveSpeedVal = 0;
  steerVal = 0;
  strafeVal = 0;
}


/**
 * Stop all the wheels
 */
void DriveControl::stopAllWheels() {
  stopWheel(WHEEL_FL);
  stopWheel(WHEEL_FR);
  stopWheel(WHEEL_RL);
  stopWheel(WHEEL_RR);
}


/**
 * Stop a specific wheel (WHEEL_FL / WHEEL_FR / WHEEL_RL / WHEEL_RR)
 */
void DriveControl::stopWheel(byte wheelNumber) {
  analogWrite(pinPWM[wheelNumber], 0);
  digitalWrite(pinFWD[wheelNumber], 0);
  digitalWrite(pinREV[wheelNumber], 0);
}

/**
 * Spin a specific wheel WHEEL_FL / WHEEL_FR / WHEEL_RL / WHEEL_RR)
 */
void DriveControl::spinWheel(byte wheelNumber, bool dir, byte speed) {
  analogWrite(pinPWM[wheelNumber], speed);
  digitalWrite(pinFWD[wheelNumber], dir ? 1 : 0);
  digitalWrite(pinREV[wheelNumber], dir ? 0 : 1);
}

/**
 * Set the Drive Speed Value
 */
void DriveControl::setSpeed(int val) {
  int newDriveSpeedVal = max(min(val, 10), -10);
  if (newDriveSpeedVal != driveSpeedVal) {
    driveSpeedVal = newDriveSpeedVal;
    Serial.print("Set Drive Speed: ");
    Serial.println(driveSpeedVal);

    // Reset the steering if stopping
    if (driveSpeedVal == 0) {
      setSteer(0);
    }
  }
};


/**
 * Set the Strafe Value
 */
void DriveControl::setStrafe(int val) {
  int newStrafeVal = max(min(val, 10), -10);
  if (newStrafeVal != strafeVal) {
    strafeVal = newStrafeVal;
    Serial.print("Set Strafe: ");
    Serial.println(strafeVal);
  }
};


/**
 * Set the Steer Value
 */
void DriveControl::setSteer(int val) {
  int newSteerVal = max(min(val, 10), -10);
  if (newSteerVal != steerVal) {
    steerVal = newSteerVal;
    Serial.print("Set Steer: ");
    Serial.println(steerVal);
  }
};


/**
 * Evaluate the input states and apply the values to the wheels
 */
void DriveControl::drive() {
  int driveSpeed_fl = 0;
  int driveSpeed_fr = 0;
  int driveSpeed_rl = 0;
  int driveSpeed_rr = 0;

  // Apply the simple speed value
  driveSpeed_fl = driveSpeedVal;
  driveSpeed_fr = driveSpeedVal;
  driveSpeed_rl = driveSpeedVal;
  driveSpeed_rr = driveSpeedVal;

  // Apply the strafe values
  if (strafeVal != 0) {
    driveSpeed_fl = driveSpeed_fl + strafeVal;
    driveSpeed_fr = driveSpeed_fr - strafeVal;
    driveSpeed_rl = driveSpeed_rl - strafeVal;
    driveSpeed_rr = driveSpeed_rr + strafeVal;
  }

  // Apply the steering (only when not strafing)
  if ((strafeVal == 0) && (steerVal != 0)) {
    // Turning on the spod?
    if (driveSpeedVal == 0) {
      driveSpeed_fl = steerVal;
      driveSpeed_fr = -steerVal;
      driveSpeed_rl = steerVal;
      driveSpeed_rr = -steerVal;
    } 
    
    // Skid steering (influencing the forward direction)
    else {
      driveSpeed_fl = constrain(driveSpeed_fl + steerVal, driveSpeed_fl > 0 ? 0 : -10, driveSpeed_fl > 0 ? 10 : 0);
      driveSpeed_fr = constrain(driveSpeed_fr - steerVal, driveSpeed_fr > 0 ? 0 : -10, driveSpeed_fr > 0 ? 10 : 0);
      driveSpeed_rl = constrain(driveSpeed_rl + steerVal, driveSpeed_rl > 0 ? 0 : -10, driveSpeed_rl > 0 ? 10 : 0);
      driveSpeed_rr = constrain(driveSpeed_rr - steerVal, driveSpeed_rr > 0 ? 0 : -10, driveSpeed_rr > 0 ? 10 : 0);
    }
  }

  // Calculate the spin directions of the wheels
  bool dir_fl = driveSpeed_fl >= 0 ? 1 : 0;
  bool dir_fr = driveSpeed_fr >= 0 ? 1 : 0;
  bool dir_rl = driveSpeed_rl >= 0 ? 1 : 0;
  bool dir_rr = driveSpeed_rr >= 0 ? 1 : 0;

  // Constrain the speeds
  driveSpeed_fl = constrain(abs(driveSpeed_fl), 0, 10);
  driveSpeed_fr = constrain(abs(driveSpeed_fr), 0, 10);
  driveSpeed_rl = constrain(abs(driveSpeed_rl), 0, 10);
  driveSpeed_rr = constrain(abs(driveSpeed_rr), 0, 10);

  // Apply the values
  if (driveSpeed_fl > 0) {
    spinWheel(WHEEL_FL, dir_fl, constrain(map(driveSpeed_fl, 0, 10, PWM_MOTOR_MIN, PWM_MOTOR_MAX) + PWM_MOTOR_FL_BIAS, 0, PWM_MOTOR_MAX));
  } else {
    stopWheel(WHEEL_FL);
  }
  if (driveSpeed_fr > 0) {
    spinWheel(WHEEL_FR, dir_fr, constrain(map(driveSpeed_fr, 0, 10, PWM_MOTOR_MIN, PWM_MOTOR_MAX) + PWM_MOTOR_FR_BIAS, 0, PWM_MOTOR_MAX));
  } else {
    stopWheel(WHEEL_FR);
  }
  if (driveSpeed_rl > 0) {
    spinWheel(WHEEL_RL, dir_rl, constrain(map(driveSpeed_rl, 0, 10, PWM_MOTOR_MIN, PWM_MOTOR_MAX) + PWM_MOTOR_RL_BIAS, 0, PWM_MOTOR_MAX));
  } else {
    stopWheel(WHEEL_RL);
  }
  if (driveSpeed_rr > 0) {
    spinWheel(WHEEL_RR, dir_rr, constrain(map(driveSpeed_rr, 0, 10, PWM_MOTOR_MIN, PWM_MOTOR_MAX) + PWM_MOTOR_RR_BIAS, 0, PWM_MOTOR_MAX));
  } else {
    stopWheel(WHEEL_RR);
  }
}