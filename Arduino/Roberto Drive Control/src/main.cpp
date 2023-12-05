#include <Arduino.h>

// Front Left Motor
#define PIN_FL_PWM 6  // Enable / PWM SPEED control
#define PIN_FL_FWD 10 // Forward Direction
#define PIN_FL_REV 9  // Back Direction 
// Front Right Motor
#define PIN_FR_PWM 11 // Enable / PWM SPEED control
#define PIN_FR_FWD A1 // Forward Direction
#define PIN_FR_REV A0 // Back Direction 
// Rear Left Motor
#define PIN_RL_PWM 3  // Enable / PWM SPEED control
#define PIN_RL_FWD 7  // Forward Direction
#define PIN_RL_REV 2  // Back Direction 
// Rear Right Motor
#define PIN_RR_PWM 5  // Enable / PWM SPEED control
#define PIN_RR_FWD 4  // Forward Direction
#define PIN_RR_REV 8  // Back Direction 

// Wheels
#define WHEEL_FL 0    // Front Left
#define WHEEL_FR 1    // Front Right
#define WHEEL_RL 2    // Rear Left
#define WHEEL_RR 3    // Rear Right

// Function Declarations
void readSerialInput();
void drive();
void stopAll();
void stopWheel(byte);
void spinWheel(byte, bool, byte);
void setSpeed(byte);
void setStrafe(byte);
void setSteer(byte);

byte pinPWM[4] = {PIN_FL_PWM, PIN_FR_PWM, PIN_RL_PWM, PIN_RR_PWM};
byte pinFWD[4] = {PIN_FL_FWD, PIN_FR_FWD, PIN_RL_FWD, PIN_RR_FWD};
byte pinREV[4] = {PIN_FL_REV, PIN_FR_REV, PIN_RL_REV, PIN_RR_REV};

// Control States (10 = 0, 0 = -10, 20 = 10)
byte speedVal = 10;   // Forward / Backward speed from 0 - 10 - 20 
byte strafeVal = 10;  // Left / Right strafe speed from 0 - 10 - 20
byte steerVal = 10;   // The amount of skid/tank steering to apply to the forward/backward speed from 0 - 10 - 20

void setup() {
  Serial.begin(9600);
  Serial.println("Init Pins...");

  pinMode(PIN_FL_PWM, OUTPUT);
  pinMode(PIN_FL_FWD, OUTPUT);
  pinMode(PIN_FL_REV, OUTPUT);
  pinMode(PIN_FR_PWM, OUTPUT);
  pinMode(PIN_FR_FWD, OUTPUT);
  pinMode(PIN_FR_REV, OUTPUT);
  pinMode(PIN_RL_PWM, OUTPUT);
  pinMode(PIN_RL_FWD, OUTPUT);
  pinMode(PIN_RL_REV, OUTPUT);
  pinMode(PIN_RR_PWM, OUTPUT);
  pinMode(PIN_RR_FWD, OUTPUT);
  pinMode(PIN_RR_REV, OUTPUT);

  Serial.println("Roberto Drive Control Ready.");
  Serial.println(" W = Forward");
  Serial.println(" S = Reverse");
  Serial.println(" A = Strafe Left");
  Serial.println(" D = Strafe Right");
  Serial.println(" Q = Steer Left");
  Serial.println(" E = Steer Right");
}

void loop() {
  readSerialInput();
  drive();
  // // Front Left
  // spinWheel(FL, 1, 128);
  // delay(1000);
  // stopAll();
  // delay(100);
  // spinWheel(FL, 0, 128);
  // delay(1000);
  // stopAll();

  // // Front Right
  // spinWheel(FR, 1, 128);
  // delay(1000);
  // stopAll();
  // delay(100);
  // spinWheel(FR, 0, 128);
  // delay(1000);
  // stopAll();

  // // Rear Left
  // spinWheel(RL, 1, 128);
  // delay(1000);
  // stopAll();
  // delay(100);
  // spinWheel(RL, 0, 128);
  // delay(1000);
  // stopAll();

  // // Rear Right
  // spinWheel(RR, 1, 128);
  // delay(1000);
  // stopAll();
  // delay(100);
  // spinWheel(RR, 0, 128);
  // delay(1000);
  // stopAll();

  // Move forward
  // spinWheel(FL, 1, 128);
  // spinWheel(FR, 1, 128);
  // spinWheel(RL, 1, 128);
  // spinWheel(RR, 1, 128);
  // delay(1000);

  // // Stop
  // stopAll();
  // delay(2000);

  // // Turn Right
  // spinWheel(FL, 1, 128);
  // spinWheel(FR, 1, 128);
  // spinWheel(RL, 0, 128);
  // spinWheel(RR, 0, 128);
  // delay(500);

  // // Stop
  // stopAll();
  // delay(2000);
}

/**
 * Stop all the wheels
 */
void stopAll() {
  stopWheel(WHEEL_FL);
  stopWheel(WHEEL_FR);
  stopWheel(WHEEL_RL);
  stopWheel(WHEEL_RR);
}

/**
 * Stop a specific wheel (FL/FR/RL/RR)
 */
void stopWheel(byte wheelNumber) {
  digitalWrite(pinPWM[wheelNumber], 0);
  digitalWrite(pinFWD[wheelNumber], 0);
  digitalWrite(pinREV[wheelNumber], 0);
}

/**
 * Spin a specific wheel (FL/FR/RL/RR)
 */
void spinWheel(byte wheelNumber, bool dir, byte speed) {
  digitalWrite(pinPWM[wheelNumber], speed);
  digitalWrite(pinFWD[wheelNumber], dir ? 1 : 0);
  digitalWrite(pinREV[wheelNumber], dir ? 0 : 1);
}

/**
 * Set the Speed Value
 */
void setSpeed(byte val) {
  byte newSpeedVal = max(min(val, 20), 0);
  if (newSpeedVal != speedVal) {
    speedVal = newSpeedVal;
    Serial.print("Speed: ");
    Serial.println(speedVal);
  }
};

/**
 * Set the Strafe Value
 */
void setStrafe(byte val) {
  byte newStrafeVal = max(min(val, 20), 0);
  if (newStrafeVal != strafeVal) {
    strafeVal = newStrafeVal;
    Serial.print("Strafe: ");
    Serial.println(strafeVal);
  }
};

/**
 * Set the Steer Value
 */
void setSteer(byte val) {
  byte newSteerVal = max(min(val, 20), 0);
  if (newSteerVal != steerVal) {
    steerVal = newSteerVal;
    Serial.print("Steer: ");
    Serial.println(steerVal);
  }
};

/**
 * Evaluate the input states and apply the values to the wheels
 */
void drive() {

}

/**
 * Read the Serial input buffer
 * This is a temporary measure until I2C communication is established
 */
void readSerialInput() {
  if (Serial.available() > 0) {
    // read the incoming byte:
    byte incomingByte = Serial.read();
  
    switch (incomingByte) {
      // Forward
      case 'w':
      case 'W':
        setSpeed(speedVal+1);
        break;

      // Reverse
      case 's':
      case 'S':
        setSpeed(speedVal-1);
        break;

      // Strafe Right
      case 'd':
      case 'D':
        setStrafe(strafeVal+1);
        break;

      // Strafe Left
      case 'a':
      case 'A':
        setStrafe(strafeVal-1);
        break;

      // Steer Right
      case 'e':
      case 'E':
        setSteer(steerVal+1);
        break;

      // Steer Left
      case 'q':
      case 'Q':
        setSteer(steerVal-1);
        break;
    }
  }
};
