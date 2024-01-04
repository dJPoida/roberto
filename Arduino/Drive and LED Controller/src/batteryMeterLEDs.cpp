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
  mode = BLM_UNKNOWN;
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
 * Set the mode of the LED strip
 */
void BatteryMeterLEDs::setMode(BatteryLEDsMode newMode) {
  if (newMode != mode) {
    mode = newMode;
    lastCurrentMillis = millis();
    updateLEDs();
  }
}


/**
 * Called by the timer to make sure that all LEDs blink in unison
 * A cycle duration is defined in the config.h
 */
void BatteryMeterLEDs::run(unsigned long currentMillis) {
    switch (mode) {
      case BLM_UNKNOWN:
      case BLM_DANGER:
        if ((currentMillis - lastCurrentMillis) > BATTERY_METER_LEDS_FLASH_DURATION) {
          flashStateOn = !flashStateOn;
          updateLEDs();

          // Store the current millis for the next cycle
          lastCurrentMillis = currentMillis;
        }
        break;
      default:
        break;
    }
}


/**
 * Update the state of the LEDs based on the current mode
 */
void BatteryMeterLEDs::updateLEDs() {
  switch (mode) {
    case BLM_UNKNOWN:
      battery_meter_leds[0] = flashStateOn ? CRGB::Orange : CRGB:: Black;
      battery_meter_leds[1] = flashStateOn ? CRGB::Orange : CRGB:: Black;
      battery_meter_leds[2] = flashStateOn ? CRGB::Orange : CRGB:: Black;
      break;
    case BLM_FULL:
      battery_meter_leds[0] = CRGB::Green;
      battery_meter_leds[1] = CRGB::Green;
      battery_meter_leds[2] = CRGB::Green;
      break;          
    case BLM_HIGH:
      battery_meter_leds[0] = CRGB::Green;
      battery_meter_leds[1] = CRGB::Green;
      battery_meter_leds[2] = CRGB::Black;
      break;          
    case BLM_MEDIUM:
      battery_meter_leds[0] = CRGB::Orange;
      battery_meter_leds[1] = CRGB::Orange;
      battery_meter_leds[2] = CRGB::Black;
      break;          
    case BLM_LOW:
      battery_meter_leds[0] = CRGB::Red;
      battery_meter_leds[1] = CRGB::Black;
      battery_meter_leds[2] = CRGB::Black;
      break;          
    case BLM_DANGER:
      battery_meter_leds[0] = flashStateOn ? CRGB::Red : CRGB:: Black;
      battery_meter_leds[1] = flashStateOn ? CRGB::Red : CRGB:: Black;
      battery_meter_leds[2] = flashStateOn ? CRGB::Red : CRGB:: Black;
      break;
    default:
      break;
  }

  FastLED.show();
}

/**
 * Evaluate the current mode and apply the current cycle number
 */
// void BatteryMeterLEDs::_applyCycle() {
  // bool newState = _state;
  // if (_cycleNo >= 0) {
  //   switch (_mode) {
  //     case LED_FLASH:
  //       newState = flash_values[_cycleNo];
  //       break;
  //     case LED_FLASH_FAST:
  //       newState = flash_fast_values[_cycleNo];
  //       break;
  //     case LED_FLASH_REGISTER:
  //       newState = flash_register_values[_cycleNo];
  //       break;
  //     default:
  //       break;
  //   }
  // }

  // if (newState != _state) {
  //   _state = newState;
  //   digitalWrite(_gpioNo, _state);
  // }
// }