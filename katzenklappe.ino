// --- Adjustable settings ---
const unsigned long DEBOUNCE_DELAY   = 50;    // ms for button/sensor debounce
const int PWM_MAX                   = 200;   // full speed PWM (0–255)
const int PWM_SLOW                  =  80;   // slow speed for braking & rollback
const int ACC_STEP                  =   5;   // PWM increment per step
const int ACC_DELAY                 =  20;   // ms delay between PWM steps
const unsigned long OPEN_HOLD_TIME  = 10000; // ms to stay open before closing

// --- Pin assignments ---
// Motor driver (L298N channel A) per datasheet default mapping
const int pwmPin         = 10;  // ENA (PWM speed control)
const int motorIn1Pin    = 9;   // IN1 → Direction input
const int motorIn2Pin    = 8;   // IN2 → Direction input

// Sensor pins
const int pirInsidePin   = 2;   // Interrupt 0 (PIR inside)
const int pirOutsidePin  = 3;   // Interrupt 1 (PIR outside)
const int beamMiddlePin  = 4;   // Safety light barrier in doorway center
const int limitOpenPin   = 5;   // End switch fully open
const int limitClosePin  = 6;   // End switch fully closed
const int slowClosePin   = 7;   // Braking barrier near closed
const int slowOpenPin    = 8;   // Braking barrier near open

// ACS712 current sensor (optional)
const int acsPin         = A0;  // Analog input from ACS712 module
const float ACS_ZERO     = 2.5; // Zero-current voltage (V)
const float ACS_SENS     = 0.066; // Sensitivity V/A (66mV/A for 30A version)
const float CURRENT_LIMIT = 2.5;  // A threshold for stall/endstop detection

// --- States for the door FSM ---
enum DoorState { CLOSED, OPENING, OPEN, CLOSING };
volatile DoorState state = CLOSED;
unsigned long openStartTime = 0;

// --- Debounce helper ---
bool readDebounced(int pin) {
  bool first  = digitalRead(pin);
  delay(DEBOUNCE_DELAY);
  bool second = digitalRead(pin);
  return (first == second) ? first : readDebounced(pin);
}

// --- Read current via ACS712 ---
float readCurrent() {
  int raw = analogRead(acsPin);
  float voltage = (raw / 1023.0) * 5.0;
  float current = (voltage - ACS_ZERO) / ACS_SENS;
  return current;
}

// --- Motor control: positive = open, negative = close ---
void drive(int speed) {
  if (speed > 0) {
    digitalWrite(motorIn1Pin, HIGH);
    digitalWrite(motorIn2Pin, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(motorIn1Pin, LOW);
    digitalWrite(motorIn2Pin, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    analogWrite(pwmPin, 0);
  }
}

// --- Emergency rollback: fully open slowly on obstruction or stall ---
void safetyRollback() {
  drive(0);
  // Slowly open until fully open limit switch
  while (readDebounced(limitOpenPin) == HIGH) {
    drive(PWM_SLOW);
    if (readCurrent() > CURRENT_LIMIT) continue;
  }
  drive(0);
  state = OPEN;
  openStartTime = millis();
}

// --- Interrupt routine to trigger opening ---
void startOpening() {
  if (state == CLOSED) state = OPENING;
}

// --- Perform initial close-then-open sequence ---
void initialSequence() {
  // Ensure door fully closed
  drive(0);
  delay(500);
  // Close slowly until closed limit
  while (readDebounced(limitClosePin) == HIGH) {
    drive(-PWM_SLOW);
  }
  drive(0);
  delay(500);
  // Open slowly until open limit
  while (readDebounced(limitOpenPin) == HIGH) {
    drive(PWM_SLOW);
  }
  drive(0);
  // Set to OPEN state for normal hold-close cycle
  state = OPEN;
  openStartTime = millis();
}

void setup() {
  // Motor driver pins
  pinMode(pwmPin,       OUTPUT);
  pinMode(motorIn1Pin,  OUTPUT);
  pinMode(motorIn2Pin,  OUTPUT);

  // Sensor pins with pull-ups
  pinMode(pirInsidePin,   INPUT_PULLUP);
  pinMode(pirOutsidePin,  INPUT_PULLUP);
  pinMode(beamMiddlePin,  INPUT_PULLUP);
  pinMode(limitOpenPin,   INPUT_PULLUP);
  pinMode(limitClosePin,  INPUT_PULLUP);
  pinMode(slowOpenPin,    INPUT_PULLUP);
  pinMode(slowClosePin,   INPUT_PULLUP);

  // ACS712 current sensor pin
  pinMode(acsPin, INPUT);

  // Attach PIR interrupts
  attachInterrupt(digitalPinToInterrupt(pirInsidePin),  startOpening, FALLING);
  attachInterrupt(digitalPinToInterrupt(pirOutsidePin), startOpening, FALLING);

  // Run initial door cycle
  initialSequence();
}

void loop() {
  switch (state) {
    case CLOSED:
      drive(0);
      break;

    case OPENING:
      // 1) Gentle acceleration
      for (int p = 0; p <= PWM_MAX; p += ACC_STEP) { drive(p); delay(ACC_DELAY); }
      // 2) Full speed until 'slow open' barrier or stall
      while (readDebounced(slowOpenPin) == HIGH) {
        drive(PWM_MAX);
        if (readDebounced(beamMiddlePin) == LOW || readCurrent() > CURRENT_LIMIT) {
          safetyRollback(); return;
        }
      }
      // 3) Decelerate to slow speed
      for (int p = PWM_MAX; p >= PWM_SLOW; p -= ACC_STEP) { drive(p); delay(ACC_DELAY); }
      // 4) Slow approach to open limit
      while (readDebounced(limitOpenPin) == HIGH) {
        drive(PWM_SLOW);
        if (readDebounced(beamMiddlePin) == LOW || readCurrent() > CURRENT_LIMIT) {
          safetyRollback(); return;
        }
      }
      drive(0);
      state = OPEN;
      openStartTime = millis();
      break;

    case OPEN:
      drive(0);
      if (millis() - openStartTime >= OPEN_HOLD_TIME) state = CLOSING;
      break;

    case CLOSING:
      // 1) Gentle acceleration (closing)
      for (int p = 0; p <= PWM_MAX; p += ACC_STEP) { drive(-p); delay(ACC_DELAY); }
      // 2) Full speed until 'slow close' barrier or stall
      while (readDebounced(slowClosePin) == HIGH) {
        drive(-PWM_MAX);
        if (readDebounced(beamMiddlePin) == LOW || readCurrent() > CURRENT_LIMIT) {
          safetyRollback(); return;
        }
      }
      // 3) Decelerate to slow speed
      for (int p = PWM_MAX; p >= PWM_SLOW; p -= ACC_STEP) { drive(-p); delay(ACC_DELAY); }
      // 4) Slow approach to close limit
      while (readDebounced(limitClosePin) == HIGH) {
        drive(-PWM_SLOW);
        if (readDebounced(beamMiddlePin) == LOW || readCurrent() > CURRENT_LIMIT) {
          safetyRollback(); return;
        }
      }
      drive(0);
      state = CLOSED;
      break;
  }
}
