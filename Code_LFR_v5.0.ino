#include <EEPROM.h>

const int sensorPins[6] = {A0, A1, A2, A3, A4, A5};

// TB6612FNG Pins
const int PWMA = 10;  // Left Motor Speed
const int AIN1 = 8;  // Left Motor Dir 1
const int AIN2 = 9;  // Left Motor Dir 2
const int PWMB = 5; // Right Motor Speed
const int BIN1 = 7;  // Right Motor Dir 1
const int BIN2 = 6;  // Right Motor Dir 2
const int STBY = 4;  // Standby pin

// UI Pins
const int BTN_MODE = 2;
const int BTN_CONFIRM = 3;

const int LED_CALIB = 11;
const int LED_RUN = 12;
const int LED_ACTIVE = 13;

// VARIABLES & SETTINGS

int sensorMin[6] = {1023, 1023, 1023, 1023, 1023, 1023};
int sensorMax[6] = {0, 0, 0, 0, 0, 0};
int sensorValues[6];

// --- SENSOR THRESHOLDS ---
const int THRESHOLD_INTERSECTION = 700; // Used to detect T, +, and sharp corners
const int THRESHOLD_TURN = 500;         // Used by turn functions to find the line
const int THRESHOLD_LINE = 50;          // Used in readLine() to filter out noise
const int LINE_CENTER = 2500;           // The ideal center position for the PID loop

// --- MOVEMENT TIMERS (ms) ---
const int TIME_CROSSROAD_PUSH = 50;     // Pushing forward to verify a crossroad
const int TIME_TURN_ALIGN = 30;         // Pushing forward before pivoting 
const int TIME_UTURN_DELAY = 100;       // Buffer before scanning for the line on a 180
const int TIME_SPIN_TIMEOUT = 1000;     // Max time allowed to spin before killing motors
const unsigned long gapTime = 150;      // Max time allowed to coast over a dashed line

// --- MOVEMENT SPEEDS ---
const int SPEED_CROSSROAD_CHECK = 100;  // Speed when creeping forward to check intersections
const int SPEED_TURN = 150;             // Speed used for hard left/right pivots
const int baseSpeed = 150;              // Target 0.6 - 0.8 m/s
const int maxSpeed = 255;

// State Machine
int currentState = 0; // 0 = Calib Idle, 1 = Run Idle
bool isRunning = false;
bool isCalibrating = false;

// Button Debouncing & Edge Detection (millis)
unsigned long lastModeDebounce = 0;
unsigned long lastConfirmDebounce = 0;
const unsigned long debounceDelay = 50; 

int lastModeState = HIGH;
int currentModeState = HIGH;
int lastConfirmState = HIGH;
int currentConfirmState = HIGH;

// PD Variables (Scaled by 1000 for Integer Math)
long Kp = 1500;  // equivalent to 1.5
long Kd = 20000; // equivalent to 20.0
long lastError = 0;

// Calibration Settings
int calibSpeed = 150;           // Motor speed during calibration spin
unsigned long calibTime = 3000; // Time (in ms) to complete two full 360-degree spins

// Macros for fast ADC
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void setup() {
  // 1. Overclock the ADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

  // 2. Pin Modes
  for (int i = 0; i < 6; i++) pinMode(sensorPins[i], INPUT);
  
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH); 

  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_CONFIRM, INPUT_PULLUP);
  
  pinMode(LED_CALIB, OUTPUT);
  pinMode(LED_RUN, OUTPUT);
  pinMode(LED_ACTIVE, OUTPUT);

  // 3. Load EEPROM Data
  EEPROM.get(0, sensorMin);
  EEPROM.get(12, sensorMax);

  // Default Boot State
  digitalWrite(LED_CALIB, HIGH);
  digitalWrite(LED_RUN, LOW);
  digitalWrite(LED_ACTIVE, LOW);
}

void loop() {
  // UI STATE MACHINE (NON-BLOCKING)
  // The 'if' condition completely locks out button reads if the bot is active.
  if (!isRunning && !isCalibrating) {
    
    // MODE BUTTON LOGIC
    int readingMode = digitalRead(BTN_MODE);
    if (readingMode != lastModeState) { lastModeDebounce = millis(); }
    
    if ((millis() - lastModeDebounce) > debounceDelay) {
      if (readingMode != currentModeState) {
        currentModeState = readingMode;
        // Edge Detection: Only trigger when transitioning from HIGH to LOW (Pressed)
        if (currentModeState == LOW) {
          currentState = !currentState; 
          digitalWrite(LED_CALIB, currentState == 0 ? HIGH : LOW);
          digitalWrite(LED_RUN, currentState == 1 ? HIGH : LOW);
        }
      }
    }
    lastModeState = readingMode;

    // -- CONFIRM BUTTON LOGIC --
    int readingConfirm = digitalRead(BTN_CONFIRM);
    if (readingConfirm != lastConfirmState) { lastConfirmDebounce = millis(); }

    if ((millis() - lastConfirmDebounce) > debounceDelay) {
      if (readingConfirm != currentConfirmState) {
        currentConfirmState = readingConfirm;
        if (currentConfirmState == LOW) {
          
          if (currentState == 0) {
            isCalibrating = true;
            digitalWrite(LED_ACTIVE, HIGH);
            calibrateSensors();
            isCalibrating = false;
            digitalWrite(LED_ACTIVE, LOW);
            
            currentState = 1; // Auto switch to Run Mode
            digitalWrite(LED_CALIB, LOW);
            digitalWrite(LED_RUN, HIGH);
          } else {
            isRunning = true;
            digitalWrite(LED_ACTIVE, HIGH);
            delay(500); // 500ms safety buffer to remove hand before motors fire
          }
        }
      }
    }
    lastConfirmState = readingConfirm;
  }

  // --- RACE EXECUTION ---
  if (isRunning) {
    executeRaceLogic();
  }
}

// CORE ALGORITHMS

void executeRaceLogic() {
  long position = readLine();

  // 1. MEMORY: Snapshot the intersection BEFORE pushing forward
  bool leftWasBlack = (sensorValues[0] > THRESHOLD_INTERSECTION);
  bool rightWasBlack = (sensorValues[5] > THRESHOLD_INTERSECTION);

  // --- INTERSECTION DETECTION ---
  if (leftWasBlack || rightWasBlack) {
    // Push forward ONCE to align the wheels and clear the horizontal line
    moveMotors(baseSpeed, baseSpeed);
    delay(TIME_TURN_ALIGN);
    
    // Re-read the track from this new vantage point
    position = readLine();
    
    // 2. VISION: Check what is actually ahead of us now
    bool straightExists = (sensorValues[2] > THRESHOLD_LINE || sensorValues[3] > THRESHOLD_LINE);
    
    // For the stop box, we check if the extreme sensors STILL see black after moving forward
    bool stillLeft = (sensorValues[0] > THRESHOLD_INTERSECTION);
    bool stillRight = (sensorValues[5] > THRESHOLD_INTERSECTION);

    // A. END OF TRACK (All Black Box)
    if (leftWasBlack && rightWasBlack && straightExists && stillLeft && stillRight) {
      stopMotors();
      isRunning = false; 
      digitalWrite(LED_ACTIVE, LOW);
      return; 
    }

    // B. STRAIGHT PRIORITY (+ Crossroads, side-branches)
    if (straightExists) {
      // The track continues forward. Let the PID handle it!
      return; 
    }

    // C. T-JUNCTION (Priority Left)
    // We use our MEMORY from before the push to know it was a T!
    if (leftWasBlack && rightWasBlack) {
      executeTurn('L');
      return;
    }

    // D. 90-DEGREE CORNERS
    if (leftWasBlack) {
      executeTurn('L');
      return;
    }
    if (rightWasBlack) {
      executeTurn('R');
      return;
    }
  }

  // --- LINE LOST RECOVERY (Dead Ends & Dashed Lines) ---
  if (position == -1) { 
    if (lastError < 800 && lastError > -800) {
      // We were going straight. It is a DASHED LINE OR DEAD END.
      moveMotors(baseSpeed, baseSpeed);
      unsigned long gapTimer = millis();
      
      while (readLine() == -1) {
        if (millis() - gapTimer > gapTime) { 
          // Timer expired. It's a DEAD END.
          executeTurn('U');
          return;
        }
      }
      // Loop broke early? It found the line again (Dashed line). Resume PID.
      
    } else if (lastError < 0) {
      // SHARP ANGLE (Left Pivot)
      moveMotors(-baseSpeed, baseSpeed);
      unsigned long spinTimer = millis();
      while (readLine() == -1) { 
        if (millis() - spinTimer > TIME_SPIN_TIMEOUT) { stopMotors(); isRunning = false; return; }
      }
    } else {
      // SHARP ANGLE (Right Pivot)
      moveMotors(baseSpeed, -baseSpeed);
      unsigned long spinTimer = millis();
      while (readLine() == -1) {
        if (millis() - spinTimer > TIME_SPIN_TIMEOUT) { stopMotors(); isRunning = false; return; }
      }
    }
    return;
  }

  // --- PD LOOP ---
  long error = position - LINE_CENTER;
  long motorSpeed = (Kp * error + Kd * (error - lastError)) / 1000;
  lastError = error;

  int leftMotorSpeed = constrain(baseSpeed + motorSpeed, -255, maxSpeed);
  int rightMotorSpeed = constrain(baseSpeed - motorSpeed, -255, maxSpeed);
  moveMotors(leftMotorSpeed, rightMotorSpeed);
}

// SENSOR & MATH FUNCTIONS

// Helper function to get a single calibrated sensor reading safely
// By adding '= -1', the function works normally if I don't provide a second number!
int readCalibratedSensor(int index, int rawVal = -1) {
  // If we didn't pass a raw value, go read it from the hardware
  if (rawVal == -1) {
    rawVal = analogRead(sensorPins[index]);
  }
  
  int val = map(rawVal, sensorMin[index], sensorMax[index], 1000, 0);
  return constrain(val, 0, 1000);
}

long readLine() {
  long weightedSum = 0;
  long sum = 0;
  bool onLine = false;

  for (int i = 0; i < 6; i++) {
    // 1. Read hardware exactly ONCE
    int rawValue = analogRead(sensorPins[i]);
    
    // 2. Pass that raw value into the helper function so it doesn't read it again!
    sensorValues[i] = readCalibratedSensor(i, rawValue);

    // 3. Compare raw reading against the true mid-point
    if (sensorValues[i] > 50) { 
      onLine = true;
      weightedSum += (long)sensorValues[i] * (i * 1000); 
      sum += sensorValues[i];
    }
  }

  if (!onLine) return -1;
  return weightedSum / sum;
}

void calibrateSensors() {
  // 1. Reset previous minimums and maximums
  for (int i = 0; i < 6; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }
  
  // 2. Start the double-spin using your variable speed
  moveMotors(calibSpeed, -calibSpeed); 
  
  unsigned long startTime = millis();
  
  // 3. Spin for the exact duration of your variable time
  while (millis() - startTime < calibTime) { 
    for (int i = 0; i < 6; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
  }
  stopMotors();
  
  // 5. Save all two arrays to permanent memory
  EEPROM.put(0, sensorMin);
  EEPROM.put(12, sensorMax);
}
// MOTOR CONTROL

void moveMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(PWMA, leftSpeed);

  if (rightSpeed >= 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(PWMB, rightSpeed);
}

void stopMotors() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
}

void executeTurn(char dir) {
  unsigned long turnTimer = millis();

  if (dir == 'L') {
    moveMotors(-SPEED_TURN, SPEED_TURN);
    
    // 1. Clear the intersection line
    while (readCalibratedSensor(0) > THRESHOLD_TURN || readCalibratedSensor(1) > THRESHOLD_TURN) {
      if (millis() - turnTimer > TIME_SPIN_TIMEOUT) { stopMotors(); isRunning = false; return; }
    }
    // 2. Lock onto the new line (waits for BOTH sensors to square up)
    while (readCalibratedSensor(2) < THRESHOLD_TURN || readCalibratedSensor(3) < THRESHOLD_TURN) {
      if (millis() - turnTimer > TIME_SPIN_TIMEOUT) { stopMotors(); isRunning = false; return; }
    }
    
  } else if (dir == 'R') {
    moveMotors(SPEED_TURN, -SPEED_TURN);
    
    while (readCalibratedSensor(4) > THRESHOLD_TURN || readCalibratedSensor(5) > THRESHOLD_TURN) {
      if (millis() - turnTimer > TIME_SPIN_TIMEOUT) { stopMotors(); isRunning = false; return; }
    }
    while (readCalibratedSensor(2) < THRESHOLD_TURN || readCalibratedSensor(3) < THRESHOLD_TURN) {
      if (millis() - turnTimer > TIME_SPIN_TIMEOUT) { stopMotors(); isRunning = false; return; }
    }
    
  } else if (dir == 'U') {
    moveMotors(SPEED_TURN, -SPEED_TURN);
    delay(TIME_UTURN_DELAY); // Blind spot delay so it doesn't falsely trigger instantly
    
    while (readCalibratedSensor(2) < THRESHOLD_TURN || readCalibratedSensor(3) < THRESHOLD_TURN) {
      if (millis() - turnTimer > TIME_SPIN_TIMEOUT) { stopMotors(); isRunning = false; return; }
    }
  }
  
  stopMotors(); // Brief electromagnetic brake to kill rotational momentum before PID takes over
}
