/*============================================================================*\
 * Roberto Drive and LED Controller - Helpers
 * Peter Eldred 2023-01
 * 
 * Helper functions, types and classes for the application
\*============================================================================*/

#ifndef HELPERS_H
#define HELPERS_H

// Used to keep track of the level that the battery should be showing
enum BatteryLevel {
  BL_UNKNOWN,    // Flashing Orange - no data has been provided or the connection has been lost with the host
  BL_FULL,       // > %80 All Green
  BL_HIGH,       // > %60 Two Green + One Off
  BL_MEDIUM,     // > %30 Two Orange + One Off
  BL_LOW,        // > %10 One Red
  BL_DANGER      // < %10 All flashing red
};

/**
 * Set the frequencies of the various PWM channels
 * This helps with power generated from the motor drivers which respond differently to slower PWM frequencies
 */
void setPWMFrequencies();


#endif