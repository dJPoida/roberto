/*============================================================================*\
 * Roberto Drive and LED Controller - Helpers
 * Peter Eldred 2023-01
 * 
 * Helper functions, types and classes for the application
\*============================================================================*/
#include <Arduino.h>

#include "main.h"
#include "config.h"
#include "helpers.h"
#include "batteryMeterLEDs.h"
#include "driveControl.h"
#include "i2cComms.h"

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
  initI2CComms(handleHostConnectionChanged);

  #ifdef SERIAL_DEBUG  
  Serial.println("Roberto Drive Control Ready.");
  #endif
}

/**
 * Main Loop
 */
void loop() {
  unsigned long currentMillis = millis();
  
  bool connected = checkConnectionToHost(currentMillis);
  
  // Only perform some core functions if connected to the host
  if (connected) {
    batteryMeterLeds.setLevelFromBatteryPercent(getBatteryLevel());
    driveControl.drive();
  }

  // Update the flashing state of the Battery indication LEDs
  batteryMeterLeds.run(currentMillis);

  // Give it some breathing room
  delay(1);
}

void handleHostConnectionChanged(bool hostConnected) {
  // Reset the known state of the battery level
  if (hostConnected) {
    batteryMeterLeds.setLevelFromBatteryPercent(getBatteryLevel());
  } else {
    batteryMeterLeds.setLevel(BL_UNKNOWN);
  }
  
  #ifdef SERIAL_DEBUG
  Serial.print("Connection to host");
  if (!hostConnected) Serial.println(" lost!");
  if (hostConnected) Serial.println(" restored.");
  #endif
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