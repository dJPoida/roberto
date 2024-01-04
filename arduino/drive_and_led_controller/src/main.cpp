/*============================================================================*\
 * Roberto Drive and LED Controller - Helpers
 * Peter Eldred 2023-01
 * 
 * Helper functions, types and classes for the application
\*============================================================================*/
#include <Arduino.h>

#include "config.h"
#include "helpers.h"
#include "batteryMeterLEDs.h"
#include "driveControl.h"

BatteryMeterLEDs batteryMeterLeds = BatteryMeterLEDs();   // The Battery Meter LED Strip
DriveControl driveControl = DriveControl();               // The Drive Controller

void setup() {
  #ifdef SERIAL_DEBUG  
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Roberto Drive Control");
  #endif

  setPWMFrequencies();

  batteryMeterLeds.init();
  driveControl.init();

  #ifdef SERIAL_DEBUG  
  Serial.println("Roberto Drive Control Ready.");
  Serial.println(" W = Forward");
  Serial.println(" S = Reverse");
  Serial.println(" A = Steer Left");
  Serial.println(" D = Steer Right");
  Serial.println(" Q = Strafe Left");
  Serial.println(" E = Strafe Right");
  Serial.println("=======================");
  Serial.println(" SPACE = EMERGENCY STOP");
  #endif
}

/**
 * Main Loop
 */
void loop() {
  unsigned long currentMillis = millis();
  // readSerialInput();
  driveControl.drive();
  batteryMeterLeds.run(currentMillis);
}


/**
 * Read the Serial input buffer
 * This is a temporary measure until I2C communication is established
 */
// void readSerialInput() {
  // if (Serial.available() > 0) {
  //   // read the incoming byte:
  //   byte incomingByte = Serial.read();
  
  //   switch (incomingByte) {
  //     // Emergency Stop
  //     case ' ':
  //       emergencyStop();
  //       break;

  //     // Forward
  //     case 'w':
  //     case 'W':
  //       setSpeed(driveSpeedVal+1);
  //       break;

  //     // Reverse
  //     case 's':
  //     case 'S':
  //       setSpeed(driveSpeedVal-1);
  //       break;

  //     // Strafe Right
  //     case 'e':
  //     case 'E':
  //       setStrafe(strafeVal+1);
  //       break;

  //     // Strafe Left
  //     case 'q':
  //     case 'Q':
  //       setStrafe(strafeVal-1);
  //       break;

  //     // Steer Right
  //     case 'd':
  //     case 'D':
  //       setSteer(steerVal+1);
  //       break;

  //     // Steer Left
  //     case 'a':
  //     case 'A':
  //       setSteer(steerVal-1);
  //       break;
  //   }
  // }
// };