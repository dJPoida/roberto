/*============================================================================*\
 * Roberto Drive and LED Controller - Battery Meter LED Control
 * Peter Eldred 2024-01
 * 
 * Handles the drive motor control functions
\*============================================================================*/
#ifndef DRIVECONTROL_H
#define DRIVECONTROL_H

#include <Arduino.h>
#include "helpers.h"

class DriveControl {
  public:
    DriveControl();
    bool init();

    void emergencyStop();
    void stopWheel(byte wheelNumber);
    void stopAllWheels();
    void spinWheel(byte wheelNumber, bool dir, byte speed);
    void setSpeed(int val);
    void setStrafe(int val);
    void setSteer(int val);
    void drive();

  private:
    // Control States
    int driveSpeedVal = 0;   // Forward / Backward speed from -10 -> 0 -> 10 
    int strafeVal = 0;       // Left / Right strafe speed from -10 -> 0 -> 10
    int steerVal = 0;        // The amount of skid/tank steering to apply to the forward/backward speed from -10 -> 0 -> 10

    // Indexable Pin Arrays
    byte pinPWM[4] = {PIN_MOTOR_FL_PWM, PIN_MOTOR_FR_PWM, PIN_MOTOR_RL_PWM, PIN_MOTOR_RR_PWM};
    byte pinFWD[4] = {PIN_MOTOR_FL_FWD, PIN_MOTOR_FR_FWD, PIN_MOTOR_RL_FWD, PIN_MOTOR_RR_FWD};
    byte pinREV[4] = {PIN_MOTOR_FL_REV, PIN_MOTOR_FR_REV, PIN_MOTOR_RL_REV, PIN_MOTOR_RR_REV};
};

#endif