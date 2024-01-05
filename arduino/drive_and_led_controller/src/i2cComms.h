/*============================================================================*\
 * Roberto Drive and LED Controller - I2C Comms
 * Peter Eldred 2024-01
 * 
 * Handles the I2C communications with the Raspberry Pi host
\*============================================================================*/
#ifndef I2CCOMMS_H
#define I2CCOMMS_H

#include <Arduino.h>
#include "helpers.h"

bool initI2CComms(void (*hostConnectionChangedEventHandler)(bool));
void handleReceive(int bytecount);
void sendHeartbeat();
void handleHeartbeatReceived();
void handleBatteryLevelReceived(byte newBatteryLevel);
bool checkConnectionToHost(unsigned long currentMillis);
byte getBatteryLevel();

#endif