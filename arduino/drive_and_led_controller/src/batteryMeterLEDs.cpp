/*============================================================================*\
 * Roberto Drive and LED Controller - Battery Meter LED Control
 * Peter Eldred 2024-01
 * 
 * Simple controller for the Battery Level LEDs connected to the drive hardware
 * controller.
\*============================================================================*/
#include <Arduino.h>
#include <FastLED.h>
#include "config.h"
#include "batteryMeterLEDs.h"

/**
 * Constructor
 */
BatteryMeterLEDs::BatteryMeterLEDs(){
}


/**
 * Initialise
 */
bool BatteryMeterLEDs::init() {
  #ifdef SERIAL_DEBUG
  Serial.print("Initialising Battery Meter LEDs...");
  #endif

  // Set the initial state
  level = BL_UNKNOWN;
  flashStateOn = true;
  lastCurrentMillis = millis();
  
  // Initialise the strip
  FastLED.addLeds<NEOPIXEL, PIN_BATTERY_METER_LEDS>(battery_meter_leds, BATTERY_METER_LEDS_COUNT);
  FastLED.setBrightness(BATTERY_METER_LEDS_BRIGHTNESS);
  updateLEDs();
  
  #ifdef SERIAL_DEBUG
  Serial.println(" done.");
  #endif

  return true;
}


/**
 * Set the level of the LED strip
 */
void BatteryMeterLEDs::setLevel(BatteryLevel newLevel) {
  if (newLevel != level) {
    level = newLevel;
    lastCurrentMillis = millis();
    updateLEDs();
  }
}


/**
 * Set the mode of the LED strip
 */
void BatteryMeterLEDs::setLevelFromBatteryPercent(byte batteryPercent) {
  BatteryLevel newLevel = level;

  // greater than 100 is basically unknown because the real value should be between 0 and 100
  if (batteryPercent > 100) {
    newLevel = BL_UNKNOWN;
  }
  else if (batteryPercent > BATTERY_LEVEL_FULL) {
    newLevel = BL_FULL;
  }
  else if (batteryPercent > BATTERY_LEVEL_HIGH) {
    newLevel = BL_HIGH;
  }
  else if (batteryPercent > BATTERY_LEVEL_MED) {
    newLevel = BL_MEDIUM;
  }
  else if (batteryPercent > BATTERY_LEVEL_LOW) {
    newLevel = BL_LOW;
  }
  else if (batteryPercent >= 0) {
    newLevel = BL_DANGER;
  }

  setLevel(newLevel);
}


/**
 * Called by the timer to make sure that all LEDs blink in unison
 * A cycle duration is defined in the config.h
 */
void BatteryMeterLEDs::run(unsigned long currentMillis) {
    switch (level) {
      case BL_UNKNOWN:
      case BL_DANGER:
        if ((currentMillis - lastCurrentMillis) > BATTERY_METER_LEDS_FLASH_DURATION) {
          flashStateOn = !flashStateOn;
          updateLEDs();

          // Store the current millis for the next flash state
          lastCurrentMillis = currentMillis;
        }
        break;
      default:
        break;
    }
}


/**
 * Update the state of the LEDs based on the current level
 */
void BatteryMeterLEDs::updateLEDs() {
  switch (level) {
    case BL_UNKNOWN:
      battery_meter_leds[0] = flashStateOn ? CRGB::Yellow : CRGB:: Black;
      battery_meter_leds[1] = flashStateOn ? CRGB::Yellow : CRGB:: Black;
      battery_meter_leds[2] = flashStateOn ? CRGB::Yellow : CRGB:: Black;
      break;
    case BL_FULL:
      battery_meter_leds[0] = CRGB::Green;
      battery_meter_leds[1] = CRGB::Green;
      battery_meter_leds[2] = CRGB::Green;
      break;          
    case BL_HIGH:
      battery_meter_leds[0] = CRGB::Green;
      battery_meter_leds[1] = CRGB::Green;
      battery_meter_leds[2] = CRGB::Black;
      break;          
    case BL_MEDIUM:
      battery_meter_leds[0] = CRGB::Orange;
      battery_meter_leds[1] = CRGB::Orange;
      battery_meter_leds[2] = CRGB::Black;
      break;          
    case BL_LOW:
      battery_meter_leds[0] = CRGB::Red;
      battery_meter_leds[1] = CRGB::Black;
      battery_meter_leds[2] = CRGB::Black;
      break;          
    case BL_DANGER:
      battery_meter_leds[0] = flashStateOn ? CRGB::Red : CRGB:: Black;
      battery_meter_leds[1] = flashStateOn ? CRGB::Red : CRGB:: Black;
      battery_meter_leds[2] = flashStateOn ? CRGB::Red : CRGB:: Black;
      break;
    default:
      break;
  }

  FastLED.show();
}
