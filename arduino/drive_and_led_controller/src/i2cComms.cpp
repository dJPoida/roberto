/*============================================================================*\
 * Roberto Drive and LED Controller - I2C Comms
 * Peter Eldred 2024-01
 * 
 * Handles the I2C communications with the Raspberry Pi host
\*============================================================================*/
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "i2cComms.h"

volatile unsigned long lastHeartbeatReceived = 0;
volatile bool connectedToHost = false;
volatile byte batteryLevel = 255;

void (*hostConnectionChangedEventHandler)(bool);

/**
 * Initialise
 */
bool initI2CComms(void (*_hostConnectionChangedEventHandler)(bool)) {
  #ifdef SERIAL_DEBUG
  Serial.print("Initialising I2C Communication Controller...");
  #endif

   // Join I2C bus as slave with address 8
  Wire.begin(I2C_ADDRESS);
  
  // Call receiveEvent when data received                
  Wire.onReceive(handleReceive);
  hostConnectionChangedEventHandler = _hostConnectionChangedEventHandler;

  #ifdef SERIAL_DEBUG
  Serial.println(" done.");
  #endif

  return true;  
}

// Function that executes whenever data is received from master
void handleReceive(int bytecount) {
  // TODO: sink the message if the bytecount isn't correct

  // Get the command
  char command = Wire.read();
  switch (command) {
    case I2C_MSG_HEARTBEAT:
      handleHeartbeatReceived();
      break;

    case I2C_MSG_BATTERY_LEVEL_UPDATE:
      // 2nd byte is the battery level percentage 0-100
      byte newBatteryLevel = Wire.read();
      handleBatteryLevelReceived(newBatteryLevel);
      break;
  }
}

/**
 * Fired when a heartbeat is received from the host
 */
void handleHeartbeatReceived() {
  lastHeartbeatReceived = millis();
}

/**
 * Fired when a battery level is received from the host
 */
void handleBatteryLevelReceived(byte newBatteryLevel) {
  batteryLevel = newBatteryLevel;

  #ifdef SERIAL_DEBUG
  Serial.print("Battery Level: ");
  Serial.println(batteryLevel, DEC);
  #endif
}

/**
 * Fired by the main unit on every loop to monitor the connection to the host
 */
bool checkConnectionToHost(unsigned long currentMillis) {
  // If currently connected, check to see if the last heartbeat was too long ago
  if (connectedToHost && ((lastHeartbeatReceived + I2C_HEARTBEAT_TIMEOUT) < currentMillis)) {
    connectedToHost = false;
    hostConnectionChangedEventHandler(false);
  } 
  
  // If not connected, check to see if a heartbeat was received recently
  else if (!connectedToHost && ((lastHeartbeatReceived + I2C_HEARTBEAT_TIMEOUT) > currentMillis)) {
    connectedToHost = true;
    hostConnectionChangedEventHandler(true);
  }

  return connectedToHost;
}

/**
 * Fired by the main unit to get the battery level and apply it to the LEDs
 */
byte getBatteryLevel() {
  return batteryLevel;
}