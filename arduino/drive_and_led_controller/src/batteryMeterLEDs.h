/*============================================================================*\
 * Roberto Drive and LED Controller - Battery Meter LED Control
 * Peter Eldred 2024-01
 * 
 * Simple controller for the Battery Level LEDs connected to the drive hardware
 * controller.
\*============================================================================*/
#ifndef BATTMETERLEDS_H
#define BATTMETERLEDS_H

#include <Arduino.h>
#include <FastLED.h>
#include "helpers.h"

class BatteryMeterLEDs {
  public:
    BatteryMeterLEDs();
    bool init();

    void run(unsigned long currentMillis);
    void setMode(BatteryLEDsMode newMode);

  private:
    CRGB battery_meter_leds[BATTERY_METER_LEDS_COUNT];  // LED array
    bool flashStateOn;                                  // For flashing modes, whether the LEDs are "ON" or "OFF"
    unsigned long lastCurrentMillis;                    // The last time the run() method was called
    BatteryLEDsMode mode = BLM_UNKNOWN;                 // The current mode of the LED strip

    void updateLEDs();   // Apply the current state to the LED Strip
};

#endif