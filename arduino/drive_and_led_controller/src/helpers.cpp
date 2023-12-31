/*============================================================================*\
 * Roberto Drive and LED Controller - Helpers
 * Peter Eldred 2023-01
 * 
 * Helper functions, types and classes for the application
\*============================================================================*/
#include <Arduino.h>
#include "helpers.h"

/**
 * Set the frequencies of the various PWM channels
 * This helps with power generated from the motor drivers which respond differently to slower PWM frequencies
 */
void setPWMFrequencies() {
  // WARNING: PLAYING WITH TIMER1 will affect the millis() function

  //---------------------------------------------- Set PWM frequency for D5 & D6 -------------------------------
  // TCCR0B = TCCR0B & B11111000 | B00000001; // set timer 0 divisor to 1 for PWM frequency of 62500.00 Hz
  // TCCR0B = TCCR0B & B11111000 | B00000010; // set timer 0 divisor to 8 for PWM frequency of 7812.50 Hz
  // TCCR0B = TCCR0B & B11111000 | B00000011; // set timer 0 divisor to 64 for PWM frequency of 976.56 Hz
  // TCCR0B = TCCR0B & B11111000 | B00000100; // set timer 0 divisor to 256 for PWM frequency of 244.14 Hz
  // TCCR0B = TCCR0B & B11111000 | B00000101; // set timer 0 divisor to 1024 for PWM frequency of 61.04 Hz

  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
  // TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
  // TCCR1B = TCCR1B & B11111000 | B00000010; // set timer 1 divisor to 8 for PWM frequency of 3921.16 Hz
  // TCCR1B = TCCR1B & B11111000 | B00000011; // set timer 1 divisor to 64 for PWM frequency of 490.20 Hz
  // TCCR1B = TCCR1B & B11111000 | B00000100; // set timer 1 divisor to 256 for PWM frequency of 122.55 Hz
  // TCCR1B = TCCR1B & B11111000 | B00000101; // set timer 1 divisor to 1024 for PWM frequency of 30.64 Hz

  //---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------
  // TCCR2B = TCCR2B & B11111000 | B00000001; // set timer 2 divisor to 1 for PWM frequency of 31372.55 Hz
  // TCCR2B = TCCR2B & B11111000 | B00000010; // set timer 2 divisor to 8 for PWM frequency of 3921.16 Hz
  // TCCR2B = TCCR2B & B11111000 | B00000011; // set timer 2 divisor to 32 for PWM frequency of 980.39 Hz
  // TCCR2B = TCCR2B & B11111000 | B00000100; // set timer 2 divisor to 64 for PWM frequency of 490.20 Hz
  // TCCR2B = TCCR2B & B11111000 | B00000101; // set timer 2 divisor to 128 for PWM frequency of 245.10 Hz
  // TCCR2B = TCCR2B & B11111000 | B00000110; // set timer 2 divisor to 256 for PWM frequency of 122.55 Hz
  TCCR2B = TCCR2B & B11111000 | B00000111; // set timer 2 divisor to 1024 for PWM frequency of 30.64 Hz
}