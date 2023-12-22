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

#define PWM_MIN 40    // The minium PWM value to provide to the motor controllers (0-255)
#define PWM_MAX 255   // The maximum PWM value to provide to the motor controllers (0-255)

#define PWM_FL_BIAS 0 // How much more PWM to offset the FL Motor by (due to various factors like motor quality)
#define PWM_FR_BIAS 0 // How much more PWM to offset the FR Motor by (due to various factors like motor quality)
#define PWM_RL_BIAS 2 // How much more PWM to offset the RL Motor by (due to various factors like motor quality)
#define PWM_RR_BIAS 5 // How much more PWM to offset the RR Motor by (due to various factors like motor quality)

// Wheels
#define WHEEL_FL 0    // Front Left
#define WHEEL_FR 1    // Front Right
#define WHEEL_RL 2    // Rear Left
#define WHEEL_RR 3    // Rear Right

// Function Declarations
void readSerialInput();
void emergencyStop();
void drive();
void stopAll();
void stopWheel(byte);
void spinWheel(byte, bool, byte);
void setSpeed(int);
void setStrafe(int);
void setSteer(int);
void initPins();
void setPWMFrequencies();

byte pinPWM[4] = {PIN_FL_PWM, PIN_FR_PWM, PIN_RL_PWM, PIN_RR_PWM};
byte pinFWD[4] = {PIN_FL_FWD, PIN_FR_FWD, PIN_RL_FWD, PIN_RR_FWD};
byte pinREV[4] = {PIN_FL_REV, PIN_FR_REV, PIN_RL_REV, PIN_RR_REV};

// Control States
int speedVal = 0;   // Forward / Backward speed from -10 -> 0 -> 10 
int strafeVal = 0;  // Left / Right strafe speed from -10 -> 0 -> 10
int steerVal = 0;   // The amount of skid/tank steering to apply to the forward/backward speed from -10 -> 0 -> 10

void setup() {
  Serial.begin(9600);
  Serial.println("Roberto Drive Control");
  Serial.println("Init Pins...");

  initPins();
  setPWMFrequencies();

  Serial.println("Roberto Drive Control Ready.");
  Serial.println(" W = Forward");
  Serial.println(" S = Reverse");
  Serial.println(" A = Steer Left");
  Serial.println(" D = Steer Right");
  Serial.println(" Q = Strafe Left");
  Serial.println(" E = Strafe Right");
  Serial.println("=======================");
  Serial.println(" SPACE = EMERGENCY STOP");
}

/**
 * Main Loop
 */
void loop() {
  readSerialInput();
  drive();
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
 * Stop a specific wheel (WHEEL_FL / WHEEL_FR / WHEEL_RL / WHEEL_RR)
 */
void stopWheel(byte wheelNumber) {
  analogWrite(pinPWM[wheelNumber], 0);
  digitalWrite(pinFWD[wheelNumber], 0);
  digitalWrite(pinREV[wheelNumber], 0);
}

/**
 * Spin a specific wheel WHEEL_FL / WHEEL_FR / WHEEL_RL / WHEEL_RR)
 */
void spinWheel(byte wheelNumber, bool dir, byte speed) {
  analogWrite(pinPWM[wheelNumber], speed);
  digitalWrite(pinFWD[wheelNumber], dir ? 1 : 0);
  digitalWrite(pinREV[wheelNumber], dir ? 0 : 1);
}

/**
 * Emergy stop - clear all input states
 */
void emergencyStop() {
  Serial.println("== EMERGENCY STOP ==");
  speedVal = 0;
  steerVal = 0;
  strafeVal = 0;
}

/**
 * Set the Speed Value
 */
void setSpeed(int val) {
  int newSpeedVal = max(min(val, 10), -10);
  if (newSpeedVal != speedVal) {
    speedVal = newSpeedVal;
    Serial.print("Set Speed: ");
    Serial.println(speedVal);

    // Reset the steering if stopping
    if (speedVal == 0) {
      setSteer(0);
    }
  }
};

/**
 * Set the Strafe Value
 */
void setStrafe(int val) {
  int newStrafeVal = max(min(val, 10), -10);
  if (newStrafeVal != strafeVal) {
    strafeVal = newStrafeVal;
    Serial.print("Set Strafe: ");
    Serial.println(strafeVal);
  }
};

/**
 * Set the Steer Value
 */
void setSteer(int val) {
  int newSteerVal = max(min(val, 10), -10);
  if (newSteerVal != steerVal) {
    steerVal = newSteerVal;
    Serial.print("Set Steer: ");
    Serial.println(steerVal);
  }
};

/**
 * Evaluate the input states and apply the values to the wheels
 */
void drive() {
  int speed_fl = 0;
  int speed_fr = 0;
  int speed_rl = 0;
  int speed_rr = 0;

  // Apply the simple speed value
  speed_fl = speedVal;
  speed_fr = speedVal;
  speed_rl = speedVal;
  speed_rr = speedVal;

  // Apply the strafe values
  if (strafeVal != 0) {
    speed_fl = speed_fl + strafeVal;
    speed_fr = speed_fr - strafeVal;
    speed_rl = speed_rl - strafeVal;
    speed_rr = speed_rr + strafeVal;
  }

  // Apply the steering (only when not strafing)
  if ((strafeVal == 0) && (steerVal != 0)) {
    // Turning on the spod?
    if (speedVal == 0) {
      speed_fl = steerVal;
      speed_fr = -steerVal;
      speed_rl = steerVal;
      speed_rr = -steerVal;
    } 
    
    // Skid steering (influencing the forward direction)
    else {
      speed_fl = constrain(speed_fl + steerVal, speed_fl > 0 ? 0 : -10, speed_fl > 0 ? 10 : 0);
      speed_fr = constrain(speed_fr - steerVal, speed_fr > 0 ? 0 : -10, speed_fr > 0 ? 10 : 0);
      speed_rl = constrain(speed_rl + steerVal, speed_rl > 0 ? 0 : -10, speed_rl > 0 ? 10 : 0);
      speed_rr = constrain(speed_rr - steerVal, speed_rr > 0 ? 0 : -10, speed_rr > 0 ? 10 : 0);
    }
  }

  // Calculate the spin directions of the wheels
  bool dir_fl = speed_fl >= 0 ? 1 : 0;
  bool dir_fr = speed_fr >= 0 ? 1 : 0;
  bool dir_rl = speed_rl >= 0 ? 1 : 0;
  bool dir_rr = speed_rr >= 0 ? 1 : 0;

  // Constrain the speeds
  speed_fl = constrain(abs(speed_fl), 0, 10);
  speed_fr = constrain(abs(speed_fr), 0, 10);
  speed_rl = constrain(abs(speed_rl), 0, 10);
  speed_rr = constrain(abs(speed_rr), 0, 10);

  // Apply the values
  if (speed_fl > 0) {
    spinWheel(WHEEL_FL, dir_fl, constrain(map(speed_fl, 0, 10, PWM_MIN, PWM_MAX) + PWM_FL_BIAS, 0, PWM_MAX));
  } else {
    stopWheel(WHEEL_FL);
  }
  if (speed_fr > 0) {
    spinWheel(WHEEL_FR, dir_fr, constrain(map(speed_fr, 0, 10, PWM_MIN, PWM_MAX) + PWM_FR_BIAS, 0, PWM_MAX));
  } else {
    stopWheel(WHEEL_FR);
  }
  if (speed_rl > 0) {
    spinWheel(WHEEL_RL, dir_rl, constrain(map(speed_rl, 0, 10, PWM_MIN, PWM_MAX) + PWM_RL_BIAS, 0, PWM_MAX));
  } else {
    stopWheel(WHEEL_RL);
  }
  if (speed_rr > 0) {
    spinWheel(WHEEL_RR, dir_rr, constrain(map(speed_rr, 0, 10, PWM_MIN, PWM_MAX) + PWM_RR_BIAS, 0, PWM_MAX));
  } else {
    stopWheel(WHEEL_RR);
  }
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
      // Emergency Stop
      case ' ':
        emergencyStop();
        break;

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
      case 'e':
      case 'E':
        setStrafe(strafeVal+1);
        break;

      // Strafe Left
      case 'q':
      case 'Q':
        setStrafe(strafeVal-1);
        break;

      // Steer Right
      case 'd':
      case 'D':
        setSteer(steerVal+1);
        break;

      // Steer Left
      case 'a':
      case 'A':
        setSteer(steerVal-1);
        break;
    }
  }
};

/**
 * Initialise the GPIO Pins
 */
void initPins(){
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
}

/**
 * Set the frequecies of the various PWM channels
 * This helps with power generated from the motor drivers which respond differently to slower PWM frequencies
 */
void setPWMFrequencies() {
  //---------------------------------------------- Set PWM frequency for D5 & D6 -------------------------------
  // TCCR0B = TCCR0B & B11111000 | B00000001; // set timer 0 divisor to 1 for PWM frequency of 62500.00 Hz
  // TCCR0B = TCCR0B & B11111000 | B00000010; // set timer 0 divisor to 8 for PWM frequency of 7812.50 Hz
  // TCCR0B = TCCR0B & B11111000 | B00000011; // set timer 0 divisor to 64 for PWM frequency of 976.56 Hz
  // TCCR0B = TCCR0B & B11111000 | B00000100; // set timer 0 divisor to 256 for PWM frequency of 244.14 Hz
  TCCR0B = TCCR0B & B11111000 | B00000101; // set timer 0 divisor to 1024 for PWM frequency of 61.04 Hz

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