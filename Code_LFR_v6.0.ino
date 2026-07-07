#include <EEPROM.h>

const int sensorPins[6] = {A0, A1, A2, A3, A4, A5};

// TB6612FNG Pins
const int PWMA = 10; // Left Motor Speed
const int AIN1 = 8;  // Left Motor Dir 1
const int AIN2 = 9;  // Left Motor Dir 2
const int PWMB = 5;  // Right Motor Speed
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

// --- WIDENED SENSOR THRESHOLDS (Docile Mode) ---
const int THRESHOLD_INTERSECTION = 650; 
const int THRESHOLD_TURN = 450;         
const int THRESHOLD_LINE = 100;         
const int LINE_CENTER = 2500;           

// --- MOVEMENT TIMERS (ms) ---
// Calibrated for 16 GA Motors & 18cm Lever Arm
const int TIME_BRAKE = 60;          // Instant momentum kill
const int TIME_ALIGN_LEVER = 450;   // Time to creep exactly 18cm forward
const int TIME_GAP_COAST = 200;     // Coast for exactly 6cm (ignores 15cm traps)
const int TIME_SPIN_TIMEOUT = 2500; 

// --- MOVEMENT SPEEDS (Tank Mode) ---
const int SPEED_CREEP = 70;         // Braking zone speed
const int SPEED_TURN = 110;         // Controlled pivot speed
const int baseSpeed = 100;          // Target 0.5 - 0.6 m/s
const int maxSpeed = 160;           // Capped maximum

// --- STATE MACHINE (The Maze Brain) ---
int currentState = 0; 
bool isRunning = false;
bool isCalibrating = false;
bool isInverted = false;
bool isBacktracking = false; // TRUE when returning from a dead end
bool inDetour = false; // TRUE when navigating around a known trap

// UI Debounce Variables
unsigned long lastModeDebounce = 0;
unsigned long lastConfirmDebounce = 0;
const unsigned long debounceDelay = 50; 
int lastModeState = HIGH, currentModeState = HIGH;
int lastConfirmState = HIGH, currentConfirmState = HIGH;

// --- DOCILE PD LOOP TUNING ---
// Kp is slightly stronger to hug the line, Kd is slashed to stop twitching
long Kp = 2000;  // 2.0
long Kd = 8000;  // 8.0
long lastError = 0;

int calibSpeed = 90;
unsigned long calibTime = 3000;

void setup() {
  for (int i = 0; i < 6; i++) pinMode(sensorPins[i], INPUT);
  
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH); 

  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_CONFIRM, INPUT_PULLUP);
  pinMode(LED_CALIB, OUTPUT);
  pinMode(LED_RUN, OUTPUT);
  pinMode(LED_ACTIVE, OUTPUT);

  EEPROM.get(0, sensorMin);
  EEPROM.get(12, sensorMax);

  digitalWrite(LED_CALIB, HIGH);
  digitalWrite(LED_RUN, LOW);
  digitalWrite(LED_ACTIVE, LOW);
}

void loop() {
  if (!isRunning && !isCalibrating) {
    int readingMode = digitalRead(BTN_MODE);
    if (readingMode != lastModeState) { lastModeDebounce = millis(); }
    if ((millis() - lastModeDebounce) > debounceDelay) {
      if (readingMode != currentModeState) {
        currentModeState = readingMode;
        if (currentModeState == LOW) {
          currentState = !currentState;
          digitalWrite(LED_CALIB, currentState == 0 ? HIGH : LOW);
          digitalWrite(LED_RUN, currentState == 1 ? HIGH : LOW);
        }
      }
    }
    lastModeState = readingMode;

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
            currentState = 1; 
            digitalWrite(LED_CALIB, LOW);
            digitalWrite(LED_RUN, HIGH);
          } else {
            isRunning = true;
            digitalWrite(LED_ACTIVE, HIGH);
            delay(500); 
          }
        }
      }
    }
    lastConfirmState = readingConfirm;
  }

  if (isRunning) {
    executeRaceLogic();
  }
}

void executeRaceLogic() {
  long position = readLine();

  // 1. INVERTED TRACK CHECK
  if (sensorValues[0] > THRESHOLD_INTERSECTION && sensorValues[5] > THRESHOLD_INTERSECTION && 
      sensorValues[2] < 300 && sensorValues[3] < 300) {
    delay(5); readLine();
    if (sensorValues[0] > THRESHOLD_INTERSECTION && sensorValues[5] > THRESHOLD_INTERSECTION && 
        sensorValues[2] < 300 && sensorValues[3] < 300) {
      isInverted = !isInverted;
      return; 
    }
  }

  // 2. ACUTE ANGLE SWEEP (Widened High-Speed Net)
  // If the center is lost, but ANY outer sensor spikes, brake and pivot
  if (sensorValues[2] < THRESHOLD_LINE && sensorValues[3] < THRESHOLD_LINE) {
    
    // Check both inner [1] and extreme outer [0] for Left Sweep
    if (sensorValues[1] > THRESHOLD_TURN || sensorValues[0] > THRESHOLD_TURN) {
      executeTurn('A', 'L'); 
      return;
    } 
    // Check both inner [4] and extreme outer [5] for Right Sweep
    else if (sensorValues[4] > THRESHOLD_TURN || sensorValues[5] > THRESHOLD_TURN) {
      executeTurn('A', 'R'); 
      return;
    }
  }

  // 3. INTERSECTION DETECTION (The Braking Zone)
  bool leftWasBlack = (sensorValues[0] > THRESHOLD_INTERSECTION && sensorValues[1] > THRESHOLD_INTERSECTION);
  bool rightWasBlack = (sensorValues[5] > THRESHOLD_INTERSECTION && sensorValues[4] > THRESHOLD_INTERSECTION);

  if (leftWasBlack || rightWasBlack) {
    
    // ACTIVE BRAKING: Kill the 16 GA motor inertia instantly
    moveMotors(-50, -50); 
    delay(TIME_BRAKE);
    
    // --- THE SMART LEVER ARM PUSH (Trap Immunity) ---
    // Instead of a blind delay, we actively scan the track while creeping 18cm.
    unsigned long pushTimer = millis();
    unsigned long lineLostTimer = 0;
    bool currentlyLost = false;
    bool straightIsTrap = false;

    moveMotors(SPEED_CREEP, SPEED_CREEP); // Start the 18cm push
    
    while (millis() - pushTimer < TIME_ALIGN_LEVER) {
      long currentPos = readLine(); // Continuously scan
      
      if (currentPos == -1) {
        // The line is gone. Start the gap timer.
        if (!currentlyLost) {
          currentlyLost = true;
          lineLostTimer = millis();
        } else if (millis() - lineLostTimer > TIME_GAP_COAST) {
          // We have driven over 5cm of pure white space during the push!
          // We caught the 15cm illegal island gap.
          straightIsTrap = true;
        }
      } else {
        // We found the line again (could be a legal short dash). Reset timer.
        currentlyLost = false; 
      }
    }
    
    // The 18cm push is finished. Take the final snapshot.
    position = readLine();
    
    bool straightExists = (sensorValues[2] > THRESHOLD_LINE || sensorValues[3] > THRESHOLD_LINE);
    bool stillLeft = (sensorValues[0] > THRESHOLD_INTERSECTION);
    bool stillRight = (sensorValues[5] > THRESHOLD_INTERSECTION);

    // --- THE TRAP OVERRIDE ---
    // If the 5cm white-space limit was breached during the push, destroy the Straight option.
    if (straightIsTrap) {
      straightExists = false; 
      inDetour = true; // Raise the flag
    }

    // A. STOP BOX
    if (leftWasBlack && rightWasBlack && straightExists && stillLeft && stillRight) {
      stopMotors();
      isRunning = false; 
      digitalWrite(LED_ACTIVE, LOW);
      return; 
    }

    // B. THE MAZE SOLVER (Straight Priority w/ Backtrack Memory)
    if (straightExists) {
      if (isBacktracking) {
        // We hit a dead end and U-turned. "Straight" is where we came from!
        // Ignore straight and process Right-Hand Rule.
        if (rightWasBlack) { 
          executeTurn('J', 'R'); 
          isBacktracking = false; // Reset memory
          return; 
        }
        else if (leftWasBlack) { 
          executeTurn('J', 'L'); 
          isBacktracking = false; // Reset memory
          return; 
        }
      } else {
        // Normal Straight Priority. Let PID handle it.
        return;
      }
    }

    // C. DETOUR EXIT OVERRIDE ---
    // If we are in a detour, and hit a T-Junction (Left + Right, no Straight)
    if (inDetour && !straightExists && leftWasBlack && rightWasBlack) {
      executeTurn('J', 'L'); // Force Left to rejoin the main track
      inDetour = false;      // Detour complete, drop the flag
      return;
    }

    // D. RIGHT-HAND PRIORITY
    if (rightWasBlack) {
      executeTurn('J', 'R');
      isBacktracking = false;
      return;
    }

    // E. LEFT CORNERS (Lowest Priority)
    if (leftWasBlack) {
      executeTurn('J', 'L');
      isBacktracking = false;
      return;
    }
  }

  // 4. GAP & DEAD-END VERIFIER
  if (position == -1) { 
    moveMotors(SPEED_CREEP, SPEED_CREEP); // Drop to 6cm coast speed
    unsigned long creepTimer = millis();
    
    while (readLine() == -1) {
      if (millis() - creepTimer > TIME_GAP_COAST) { 
        // 6cm crossed without finding a line. True Dead End.
        executeTurn('U', 'U');
        isBacktracking = true; // Raise the Maze Solver flag!
        return;
      }
    }
    // Loop broke? It was a dashed line. Resume normal driving.
    return;
  }

  // 5. DOCILE PD LOOP
  long error = position - LINE_CENTER;
  long motorSpeed = (Kp * error + Kd * (error - lastError)) / 1000;
  lastError = error;

  int leftMotorSpeed = constrain(baseSpeed + motorSpeed, -255, maxSpeed);
  int rightMotorSpeed = constrain(baseSpeed - motorSpeed, -255, maxSpeed);
  moveMotors(leftMotorSpeed, rightMotorSpeed);
}

// --- CORE FUNCTIONS ---

int readCalibratedSensor(int index, int rawVal = -1){
  if (rawVal == -1) { rawVal = analogRead(sensorPins[index]); }
  int val;
  if (isInverted) { val = map(rawVal, sensorMin[index], sensorMax[index], 1000, 0); } 
  else { val = map(rawVal, sensorMin[index], sensorMax[index], 0, 1000); }
  return constrain(val, 0, 1000);
}

long readLine(){
  long weightedSum = 0;
  long sum = 0;
  bool onLine = false;

  for (int i = 0; i < 6; i++) {
    int rawValue = analogRead(sensorPins[i]);
    sensorValues[i] = readCalibratedSensor(i, rawValue);
    if (sensorValues[i] > THRESHOLD_LINE) { 
      onLine = true;
      weightedSum += (long)sensorValues[i] * (i * 1000); 
      sum += sensorValues[i];
    }
  }
  if (!onLine) return -1;
  return weightedSum / sum;
}

void calibrateSensors(){
  for (int i = 0; i < 6; i++) { sensorMin[i] = 1023; sensorMax[i] = 0; }
  moveMotors(calibSpeed, -calibSpeed);
  unsigned long startTime = millis();
  
  while (millis() - startTime < calibTime) { 
    for (int i = 0; i < 6; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
  }
  stopMotors();
  EEPROM.put(0, sensorMin);
  EEPROM.put(12, sensorMax);
}

void moveMotors(int leftSpeed, int rightSpeed){
  if (leftSpeed >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); } 
  else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); leftSpeed = -leftSpeed; }
  analogWrite(PWMA, leftSpeed);

  if (rightSpeed >= 0) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); } 
  else { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); rightSpeed = -rightSpeed; }
  analogWrite(PWMB, rightSpeed);
}

void stopMotors(){
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
}

void executeTurn(char type, char dir){
  unsigned long turnTimer = millis();

  // TYPE 'A': Acute Angles (Pivot slightly slower to avoid overshooting)
  if (type == 'A') {
    if (dir == 'L') { moveMotors(-SPEED_TURN, SPEED_TURN); }
    else if (dir == 'R') { moveMotors(SPEED_TURN, -SPEED_TURN); }
    
    // Just lock onto the line instantly, no need to clear an intersection line
    while (readCalibratedSensor(2) < THRESHOLD_TURN && readCalibratedSensor(3) < THRESHOLD_TURN) {
      if (millis() - turnTimer > TIME_SPIN_TIMEOUT) { stopMotors(); isRunning = false; return; }
    }
  } 
  
  // TYPE 'J' or 'U': Junctions & U-Turns
  else {
    if (dir == 'L') {
      moveMotors(-SPEED_TURN, SPEED_TURN);
      while (readCalibratedSensor(0) > THRESHOLD_TURN && readCalibratedSensor(1) > THRESHOLD_TURN) {} // Clear horizontal
      while (readCalibratedSensor(2) < THRESHOLD_TURN && readCalibratedSensor(3) < THRESHOLD_TURN) {
        if (millis() - turnTimer > TIME_SPIN_TIMEOUT) { stopMotors(); isRunning = false; return; }
      }
    } else if (dir == 'R') {
      moveMotors(SPEED_TURN, -SPEED_TURN);
      while (readCalibratedSensor(4) > THRESHOLD_TURN && readCalibratedSensor(5) > THRESHOLD_TURN) {} 
      while (readCalibratedSensor(2) < THRESHOLD_TURN && readCalibratedSensor(3) < THRESHOLD_TURN) {
        if (millis() - turnTimer > TIME_SPIN_TIMEOUT) { stopMotors(); isRunning = false; return; }
      }
    } else if (dir == 'U') {
      moveMotors(SPEED_TURN, -SPEED_TURN);
      delay(200); // 17cm width takes longer to 180 pivot
      while (readCalibratedSensor(2) < THRESHOLD_TURN && readCalibratedSensor(3) < THRESHOLD_TURN) {
        if (millis() - turnTimer > TIME_SPIN_TIMEOUT) { stopMotors(); isRunning = false; return; }
      }
    }
  }
  
  stopMotors(); // Brief electromagnetic brake
  delay(20);
}