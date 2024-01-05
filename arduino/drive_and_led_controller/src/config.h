/*============================================================================*\
 * Roberto Drive and LED Controller - Config
 * Peter Eldred 2023-01
 * 
 * All of the system config goes here
\*============================================================================*/
#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"

// Comment out this line to remove USB serial debug from all files
// #define SERIAL_DEBUG

// I2C Comms (Many of these must match on the host)
#define I2C_ADDRESS 0x8                       // Address to assign to this device
#define I2C_HEARTBEAT_TIMEOUT 1000            // How long should we go without receiving a heartbeat before we assume disconnection from the host (ms)
#define I2C_MSG_HEARTBEAT 'H'                 // Leading character to be interpreted as a heartbeat
#define I2C_MSG_BATTERY_LEVEL_UPDATE 'B'      // Leading character to be interpreted as a battery level update message
#define I2C_TERMINATOR 0x0D                   // Character to be interpreted as the terminator for a string

// Battery Meter LEDs
#define PIN_BATTERY_METER_LEDS A3
#define BATTERY_METER_LEDS_COUNT 3
#define BATTERY_METER_LEDS_BRIGHTNESS 50 // 0->255
#define BATTERY_METER_LEDS_FLASH_DURATION 500 // How long each "ON" / "OFF" cycle should last (Note - this is affected by the timer changes and does not represent actual milliseconds)

// Battery Level Percentages
#define BATTERY_LEVEL_FULL 80
#define BATTERY_LEVEL_HIGH 60
#define BATTERY_LEVEL_MED 30
#define BATTERY_LEVEL_LOW 10

// Front Left Motor
#define PIN_MOTOR_FL_PWM 6  // Enable / PWM SPEED control
#define PIN_MOTOR_FL_FWD 10 // Forward Direction
#define PIN_MOTOR_FL_REV 9  // Back Direction 

// Front Right Motor
#define PIN_MOTOR_FR_PWM 11 // Enable / PWM SPEED control
#define PIN_MOTOR_FR_FWD A1 // Forward Direction
#define PIN_MOTOR_FR_REV A0 // Back Direction

// Rear Left Motor
#define PIN_MOTOR_RL_PWM 3  // Enable / PWM SPEED control
#define PIN_MOTOR_RL_FWD 7  // Forward Direction
#define PIN_MOTOR_RL_REV 2  // Back Direction

// Rear Right Motor
#define PIN_MOTOR_RR_PWM 5  // Enable / PWM SPEED control
#define PIN_MOTOR_RR_FWD 4  // Forward Direction
#define PIN_MOTOR_RR_REV 8  // Back Direction 

// Motor PWM Values
#define PWM_MOTOR_MIN 40    // The minium PWM value to provide to the motor controllers (0-255)
#define PWM_MOTOR_MAX 255   // The maximum PWM value to provide to the motor controllers (0-255)
#define PWM_MOTOR_FL_BIAS 0 // How much more PWM to offset the FL Motor by (due to various factors like motor quality)
#define PWM_MOTOR_FR_BIAS 0 // How much more PWM to offset the FR Motor by (due to various factors like motor quality)
#define PWM_MOTOR_RL_BIAS 2 // How much more PWM to offset the RL Motor by (due to various factors like motor quality)
#define PWM_MOTOR_RR_BIAS 5 // How much more PWM to offset the RR Motor by (due to various factors like motor quality)

// Wheels
#define WHEEL_FL 0    // Front Left
#define WHEEL_FR 1    // Front Right
#define WHEEL_RL 2    // Rear Left
#define WHEEL_RR 3    // Rear Right


#endif