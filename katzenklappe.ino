#include <Arduino.h>

// --- Adjustable settings ---
const unsigned long DEBOUNCE_DELAY    = 50;    // ms for button debouncing
const unsigned long OPEN_HOLD_TIME   = 10000; // ms to stay open before closing
const unsigned long BLINK_DURATION   = 5000;  // ms to blink LED after trigger
const unsigned long BLINK_INTERVAL   = 250;   // ms blink toggle interval
const unsigned long SERIAL_INTERVAL  = 500;   // ms between serial reports
const unsigned long SAFETY_TIMEOUT   = 5000;  // ms max for safety rollback
const unsigned long BRAKE_DELAY      = 150;   // ms between brake PWM steps for smoother ramp-down

const int PWM_MAX        = 255;  // max PWM duty (0–255)
const int PWM_SLOW       = 50;   // slow speed threshold for braking
const int PWM_EMERGENCY  = 120;  // emergency opening speed
const int ACC_STEP       = 1;    // small PWM step for smooth ramp (smaller = slower ramp)

// --- Pin assignments ---
const int pwmPin         = 10;  // ENA (PWM speed control)
const int motorIn1Pin    = 9;   // IN1 direction
const int motorIn2Pin    = 11;  // IN2 direction
const int ledPin         = LED_BUILTIN;

// Motion sensors (AM312, active HIGH)
const int motionInsidePin  = 2;
const int motionOutsidePin = 3;

// Other switches (active LOW)
const int swMiddlePin    = 4;
const int swOpenPin      = 5;
const int swClosePin     = 6;
const int swSlowOpenPin  = 7;
const int swSlowClosePin = 8;

// Door FSM states
enum DoorState { CLOSED, OPENING, OPEN, CLOSING };
DoorState state = CLOSED;

// Phases for ramp control
enum Phase { RAMP_UP, RUN, BRAKE, FINAL };
Phase phase;

// Ramp segment definition
struct Segment { int startPWM, endPWM; unsigned long duration; };

const Segment segments[] = {
  {   0,  20, 1000 },
  {  15,  30, 1000 },
  {  25,  60, 1000 },
  {  50, 100, 1000 },
  {  90, 150, 1000 },
  { 150, 255, 1000 }
};
const size_t numSegments = sizeof(segments)/sizeof(segments[0]);

// Ramp control variables
size_t segIndex       = 0;
int   pwmValue        = 0;
unsigned long lastStepTime = 0;
unsigned long stepDelay    = 0;

// Timing variables
unsigned long motionTime    = 0;
unsigned long openStartTime = 0;
unsigned long lastSerial    = 0;

// Debounce helper class for mechanical switches
class Debounce {
public:
  Debounce(uint8_t p): pin(p), lastState(HIGH), lastReading(HIGH), lastTime(0) {}
  void begin() { pinMode(pin, INPUT_PULLUP); lastState = digitalRead(pin); lastReading = lastState; }
  bool read() {
    bool reading = digitalRead(pin);
    if (reading != lastReading) {
      lastTime = millis();
      lastReading = reading;
    }
    if ((millis() - lastTime) > DEBOUNCE_DELAY) {
      if (reading != lastState) {
        lastState = reading;
      }
    }
    return lastState;
  }
private:
  uint8_t pin;
  bool    lastState;
  bool    lastReading;
  unsigned long lastTime;
};

Debounce dbMiddle(swMiddlePin), dbOpen(swOpenPin), dbClose(swClosePin), dbSlowOpen(swSlowOpenPin), dbSlowClose(swSlowClosePin);

// Motor drive
void drive(int speed) {
  if (speed > 0) {
    digitalWrite(motorIn1Pin, HIGH);
    digitalWrite(motorIn2Pin, LOW);
    analogWrite(pwmPin, speed);
    Serial.print("Motor Opening PWM="); Serial.println(speed);
  } else if (speed < 0) {
    digitalWrite(motorIn1Pin, LOW);
    digitalWrite(motorIn2Pin, HIGH);
    analogWrite(pwmPin, -speed);
    Serial.print("Motor Closing PWM="); Serial.println(-speed);
  } else {
    analogWrite(pwmPin, 0);
//    Serial.println("Motor Stop");
  }
}

// Initial calibration: blocking at startup
void initialSequence() {
  Serial.println("Initial sequence: Closing to home");
  while (dbClose.read() == HIGH) {
    drive(-PWM_SLOW);
  }
  drive(0);
  Serial.println("Closed limit reached");
  delay(200);
  Serial.println("Initial sequence: Opening to end");
  while (dbOpen.read() == HIGH) {
    drive(PWM_SLOW);
  }
  drive(0);
  Serial.println("Open limit reached");
  state = OPEN;
  openStartTime = millis();
}

// Safety rollback: only on CLOSING, with timeout
void safetyRollback() {
  Serial.println("Safety rollback triggered");
  unsigned long start = millis();
  drive(0);
  if (state == CLOSING) {
    while ((millis() - start) < SAFETY_TIMEOUT && dbOpen.read() == HIGH) {
      Serial.println("Rollback: emergency opening");
      drive(PWM_EMERGENCY);
      delay(10);
    }
    drive(0);
    Serial.println("Rollback complete, state -> OPEN");
    state = OPEN;
    openStartTime = millis();
  } else if (state == OPENING) {
    Serial.println("Rollback: opening already in progress -> state OPEN");
    state = OPEN;
    openStartTime = millis();
  }
}

// Begin ramp sequence
void startRamp(bool opening) {
  Serial.print(opening ? "Starting OPENING ramp" : "Starting CLOSING ramp");
  Serial.println();
  phase = RAMP_UP;
  segIndex = 0;
  pwmValue = segments[0].startPWM;
  lastStepTime = millis();
  int delta = abs(segments[0].endPWM - segments[0].startPWM);
  int steps = max(1, delta / ACC_STEP);
  stepDelay = segments[0].duration / steps;
}

// Handle OPENING state non-blocking
void handleOpening() {
  unsigned long now = millis();
  switch (phase) {
    case RAMP_UP:
      if (now - lastStepTime >= stepDelay) {
        pwmValue += ACC_STEP;
        drive(pwmValue);
        lastStepTime += stepDelay;
        if (pwmValue >= segments[segIndex].endPWM) {
          segIndex++;
          if (segIndex < numSegments) {
            int start = segments[segIndex].startPWM;
            int end   = segments[segIndex].endPWM;
            int delta = abs(end - start);
            int steps = max(1, delta / ACC_STEP);
            stepDelay = segments[segIndex].duration / steps;
            pwmValue = start;
          } else {
            phase = RUN;
            Serial.println("Opening ramp complete, full speed now");
          }
        }
      }
      break;

    case RUN:
      if (dbMiddle.read() == LOW) {
        Serial.println("Obstruction detected during opening");
        safetyRollback();
        return;
      }
      drive(PWM_MAX);
      if (dbSlowOpen.read() == LOW) {
        Serial.println("Slow-open sensor triggered, braking");
        phase = BRAKE;
        pwmValue = PWM_MAX;
        lastStepTime = now;
      }
      break;

    case BRAKE:
      if (now - lastStepTime >= BRAKE_DELAY) {
        pwmValue -= ACC_STEP;
        drive(pwmValue);
        lastStepTime += BRAKE_DELAY;
        if (pwmValue <= PWM_SLOW) {
          phase = FINAL;
          Serial.println("Braking complete, final approach");
        }
      }
      break;

    case FINAL:
      if (dbOpen.read() == HIGH) {
        drive(PWM_SLOW);
      } else {
        drive(0);
        state = OPEN;
        openStartTime = now;
        Serial.println("State -> OPEN");
      }
      break;
  }
}

// Handle CLOSING state non-blocking (mirror of OPENING)
void handleClosing() {
  unsigned long now = millis();
  switch (phase) {
    case RAMP_UP:
      if (now - lastStepTime >= stepDelay) {
        pwmValue += ACC_STEP;
        drive(-pwmValue);
        lastStepTime += stepDelay;
        if (pwmValue >= segments[segIndex].endPWM) {
          segIndex++;
          if (segIndex < numSegments) {
            int start = segments[segIndex].startPWM;
            int end   = segments[segIndex].endPWM;
            int delta = abs(end - start);
            int steps = max(1, delta / ACC_STEP);
            stepDelay = segments[segIndex].duration / steps;
            pwmValue = start;
          } else {
            phase = RUN;
            Serial.println("Closing ramp complete, full speed now");
          }
        }
      }
      break;

    case RUN:
      if (dbMiddle.read() == LOW) {
        Serial.println("Obstruction detected during closing");
        safetyRollback();
        return;
      }
      drive(-PWM_MAX);
      if (dbSlowClose.read() == LOW) {
        Serial.println("Slow-close sensor triggered, braking");
        phase = BRAKE;
        pwmValue = PWM_MAX;
        lastStepTime = now;
      }
      break;

    case BRAKE:
      if (now - lastStepTime >= BRAKE_DELAY) {
        pwmValue -= ACC_STEP;
        drive(-pwmValue);
        lastStepTime += BRAKE_DELAY;
        if (pwmValue <= PWM_SLOW) {
          phase = FINAL;
          Serial.println("Braking complete, final approach");
        }
      }
      break;

    case FINAL:
      if (dbClose.read() == HIGH) {
        drive(-PWM_SLOW);
      } else {
        drive(0);
        state = CLOSED;
        Serial.println("State -> CLOSED");
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Sliding Door System starting...");

  // Motor and LED
  pinMode(pwmPin, OUTPUT);
  pinMode(motorIn1Pin, OUTPUT);
  pinMode(motorIn2Pin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // PIR motion sensors
  pinMode(motionInsidePin, INPUT);
  pinMode(motionOutsidePin, INPUT);

  // Other switches
  dbMiddle.begin(); dbOpen.begin(); dbClose.begin(); dbSlowOpen.begin(); dbSlowClose.begin();

  // Quick motor test
  Serial.println("Motor test: max speed open");
  drive(PWM_MAX);
  delay(500);
  drive(0);
  Serial.println("Motor test complete");

  initialSequence();
}

void loop() {
  unsigned long now = millis();

  // Read sensors
  bool insideMotion  = (digitalRead(motionInsidePin)  == HIGH);
  bool outsideMotion = (digitalRead(motionOutsidePin) == HIGH);
  bool middle        = (dbMiddle.read()  == LOW);
  bool openLim       = (dbOpen.read()    == LOW);
  bool closeLim      = (dbClose.read()   == LOW);
  bool slowO         = (dbSlowOpen.read()== LOW);
  bool slowC         = (dbSlowClose.read()==LOW);

    // Endstop overrides: only when moving towards the end
  if (state == OPENING && openLim) {
    Serial.println("Endstop: door fully OPEN -> stop");
    drive(0);
    state = OPEN;
    openStartTime = now;
  }
  if (state == CLOSING && closeLim) {
    Serial.println("Endstop: door fully CLOSED -> stop");
    drive(0);
    state = CLOSED;
  }

  // Trigger opening on motion
  if (state == CLOSED && (insideMotion || outsideMotion)) {
    state = OPENING;
    motionTime = now;
    Serial.print("Motion trigger: "); Serial.println(insideMotion ? "Inside" : "Outside");
    startRamp(true);
  }

  // Reset hold timer if motion during open
  if (state == OPEN && (insideMotion || outsideMotion)) {
    openStartTime = now;
  }

  // LED blink indicator
  if (now - motionTime < BLINK_DURATION) {
    digitalWrite(ledPin, ((now / BLINK_INTERVAL) % 2) ? HIGH : LOW);
  } else {
    digitalWrite(ledPin, LOW);
  }

  // Periodic debug report
  if (now - lastSerial >= SERIAL_INTERVAL) {
    lastSerial = now;
    Serial.print("State="); Serial.print(state);
    Serial.print(" InMotion="); Serial.print(insideMotion);
    Serial.print(" OutMotion="); Serial.print(outsideMotion);
    Serial.print(" Mid=" ); Serial.print(middle);
    Serial.print(" SO="  ); Serial.print(slowO);
    Serial.print(" SC="  ); Serial.print(slowC);
    Serial.print(" O="   ); Serial.print(openLim);
    Serial.print(" C="   ); Serial.println(closeLim);
  }

  // FSM dispatch
  switch (state) {
    case CLOSED:
      drive(0);
      break;
    case OPENING:
      handleOpening();
      break;
    case OPEN:
      drive(0);
      if (now - openStartTime >= OPEN_HOLD_TIME) {
        state = CLOSING;
        Serial.println("Hold time expired -> CLOSING");
        startRamp(false);
      }
      break;
    case CLOSING:
      handleClosing();
      break;
  }
}
