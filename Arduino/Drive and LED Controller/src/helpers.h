/*============================================================================*\
 * Roberto Drive and LED Controller - Helpers
 * Peter Eldred 2023-01
 * 
 * Helper functions, types and classes for the application
\*============================================================================*/

#ifndef HELPERS_H
#define HELPERS_H

// Used to keep track of the mode the LED is in
enum BatteryLEDsMode {
  BLM_UNKNOWN,    // Flashing Orange - no data has been provided or the connection has been lost with the host
  BLM_FULL,       // > %80 All Green
  BLM_HIGH,       // > %60 Two Green + One Off
  BLM_MEDIUM,     // > %30 Two Orange + One Off
  BLM_LOW,        // > %10 One Red
  BLM_DANGER      // < %10 All flashing red
};

/**
 * Set the frequencies of the various PWM channels
 * This helps with power generated from the motor drivers which respond differently to slower PWM frequencies
 */
void setPWMFrequencies();

#endif