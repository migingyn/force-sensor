/*
 * Robotic Claw with Per-Jaw Force-Modulated Servos
 *
 * Mechanism:
 *   - Two jaws, each driven by its own servo.
 *   - Each jaw has a force sensor at its tip; that sensor modulates *its own*
 *     servo. The two jaws close independently, so an asymmetric object (one
 *     tip touches first) is gripped cleanly: the contacted side stops while
 *     the other keeps closing until it also hits the target force.
 *
 * Hardware (Arduino Nano + Interlink FSR-402, two channels):
 *   - FSR-402 per tip wired as a voltage divider:
 *       5V -> FSR -> ANALOG_PIN, and ANALOG_PIN -> 10k -> GND.
 *     Left tip -> A0, right tip -> A1.
 *   - Two hobby servos on PWM pins D9 (left jaw) and D10 (right jaw).
 *   - Power the servos from their own 5-6V supply with a common ground to
 *     the Nano. Do NOT power servos from the Nano's 5V rail.
 *
 * Behavior:
 *   - On startup, opens both jaws and tares the no-load FSR baseline.
 *   - On 'c' command, each jaw closes at its own pace; jaw stops when its
 *     own tip force reaches TARGET_FORCE_N (with a hysteresis deadband so
 *     it can re-tighten if the object slips and force drops).
 *   - Hard safety stop per jaw at MAX_FORCE_N (backs that jaw off).
 *   - Serial telemetry at 115200 baud.
 *
 * Serial commands:
 *   o          open both jaws
 *   c          close to target force (per-jaw)
 *   t<value>   set target force in Newtons, e.g. "t2.5"
 *   z          re-tare baselines (jaws must be open and untouched)
 */

#include <Servo.h>

// ---------- Pin / hardware config ----------
const uint8_t  PIN_FSR_LEFT     = A0;
const uint8_t  PIN_FSR_RIGHT    = A1;
const uint8_t  PIN_SERVO_LEFT   = 9;
const uint8_t  PIN_SERVO_RIGHT  = 10;

const float    VREF             = 5.0;
const int      ADC_MAX          = 1023;
const float    PULLDOWN_OHMS    = 10000.0;

// Per-jaw travel. Jaws are typically mirrored, so "closed" angles differ.
// Tune these for your linkage; the controller doesn't care which is larger,
// it just steps each jaw from its OPEN value toward its CLOSED value.
const int      LEFT_OPEN_DEG    = 30;
const int      LEFT_CLOSED_DEG  = 150;
const int      RIGHT_OPEN_DEG   = 150;
const int      RIGHT_CLOSED_DEG = 30;

// ---------- Control parameters ----------
float TARGET_FORCE_N      = 2.0;   // desired grip force per tip
const float MAX_FORCE_N   = 8.0;   // hard per-jaw safety stop
const float FORCE_DEADBAND = 0.15; // N, hysteresis around target
const int   STEP_DEG       = 1;    // servo increment per control tick
const unsigned long CONTROL_PERIOD_MS = 20;   // ~50 Hz
const unsigned long TELEM_PERIOD_MS   = 200;  // 5 Hz

// ---------- Per-jaw state ----------
struct Jaw {
  Servo    servo;
  uint8_t  fsrPin;
  uint8_t  servoPin;
  int      openDeg;
  int      closedDeg;
  int      angle;       // current commanded angle
  float    baseline;    // ADC baseline at no load
  float    force;       // latest force in Newtons
  const __FlashStringHelper* name;
};

Jaw left  = { Servo(), PIN_FSR_LEFT,  PIN_SERVO_LEFT,  LEFT_OPEN_DEG,  LEFT_CLOSED_DEG,
              LEFT_OPEN_DEG,  0, 0, F("L") };
Jaw right_ = { Servo(), PIN_FSR_RIGHT, PIN_SERVO_RIGHT, RIGHT_OPEN_DEG, RIGHT_CLOSED_DEG,
              RIGHT_OPEN_DEG, 0, 0, F("R") };

enum Mode { MODE_IDLE, MODE_OPEN, MODE_CLOSE_TO_FORCE };
Mode mode = MODE_OPEN;

unsigned long lastControl = 0;
unsigned long lastTelem   = 0;

// ---------- FSR conversion (Interlink FSR-402, datasheet curve) ----------
// Voltage-divider math: V = Vcc * R / (R + R_FSR), so R_FSR = R * (Vcc-V)/V.
// Conductance G = 1/R_FSR. The FSR-402 datasheet relates conductance to force
// roughly linearly with a knee near 1 mS; the piecewise fit below matches the
// standard Adafruit/Interlink guide (output is force in Newtons).
// Tune against a known weight if precision matters.
float adcToForceN(int adc, float baseline) {
  float corrected = adc - baseline;
  if (corrected < 1) return 0.0;
  float v_mV = (corrected / (float)ADC_MAX) * VREF * 1000.0;
  if (v_mV < 1.0) return 0.0;                       // no contact
  if (v_mV > VREF * 1000.0 - 1.0) v_mV = VREF * 1000.0 - 1.0;
  float rFsr = (VREF * 1000.0 - v_mV) * PULLDOWN_OHMS / v_mV;  // ohms
  if (rFsr <= 0) return 0.0;
  float conductance_uS = 1.0e6 / rFsr;
  // Piecewise force-in-Newtons fit. +12.5 in the upper branch makes the two
  // segments continuous at the knee (the Adafruit reference omits it).
  if (conductance_uS <= 1000.0) return conductance_uS / 80.0;
  return (conductance_uS - 1000.0) / 30.0 + 12.5;
}

int readAveraged(uint8_t pin, uint8_t samples = 8) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) sum += analogRead(pin);
  return sum / samples;
}

// Move a jaw one STEP_DEG toward `targetDeg`, regardless of direction.
void stepJawToward(Jaw& j, int targetDeg) {
  int next = j.angle;
  if (j.angle < targetDeg) next = min(j.angle + STEP_DEG, targetDeg);
  else if (j.angle > targetDeg) next = max(j.angle - STEP_DEG, targetDeg);
  // Clamp to the jaw's own travel window (handles either direction).
  int lo = min(j.openDeg, j.closedDeg);
  int hi = max(j.openDeg, j.closedDeg);
  next = constrain(next, lo, hi);
  if (next != j.angle) {
    j.angle = next;
    j.servo.write(j.angle);
  }
}

// Step a jaw one STEP_DEG in the "closing" direction (toward closedDeg).
void stepJawClose(Jaw& j) {
  stepJawToward(j, j.closedDeg);
}

// Step a jaw one STEP_DEG in the "opening" direction (toward openDeg).
void stepJawOpen(Jaw& j) {
  stepJawToward(j, j.openDeg);
}

void tareJaw(Jaw& j) {
  long sum = 0;
  const int N = 32;
  for (int i = 0; i < N; i++) { sum += analogRead(j.fsrPin); delay(5); }
  j.baseline = sum / (float)N;
}

void tareBaselines() {
  tareJaw(left);
  tareJaw(right_);
  Serial.print(F("Tared. baselines L="));
  Serial.print(left.baseline, 1);
  Serial.print(F(" R="));
  Serial.println(right_.baseline, 1);
}

// One control tick for a single jaw.
void updateJaw(Jaw& j) {
  int adc = readAveraged(j.fsrPin);
  j.force = adcToForceN(adc, j.baseline);

  // Safety: back off no matter what mode we're in.
  if (j.force > MAX_FORCE_N) {
    // Back off two steps in the opening direction.
    stepJawOpen(j);
    stepJawOpen(j);
    Serial.print(F("!! OVERFORCE "));
    Serial.print((const __FlashStringHelper*)j.name);
    Serial.print(F("="));
    Serial.print(j.force, 2);
    Serial.println(F(" N"));
    return;
  }

  switch (mode) {
    case MODE_OPEN:
      stepJawOpen(j);
      break;
    case MODE_CLOSE_TO_FORCE:
      if (j.force < TARGET_FORCE_N - FORCE_DEADBAND) {
        stepJawClose(j);
      } else if (j.force > TARGET_FORCE_N + FORCE_DEADBAND) {
        stepJawOpen(j);
      }
      // else: within deadband, hold position
      break;
    case MODE_IDLE:
    default:
      break;
  }
}

// ---------- Serial command handling ----------
void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'o': case 'O':
      mode = MODE_OPEN;
      Serial.println(F("CMD: open"));
      break;
    case 'c': case 'C':
      mode = MODE_CLOSE_TO_FORCE;
      Serial.println(F("CMD: close to target"));
      break;
    case 'z': case 'Z':
      mode = MODE_OPEN;
      left.servo.write(left.openDeg);
      right_.servo.write(right_.openDeg);
      left.angle = left.openDeg;
      right_.angle = right_.openDeg;
      delay(400);
      tareBaselines();
      break;
    case 't': case 'T': {
      float v = Serial.parseFloat();
      if (v > 0 && v <= MAX_FORCE_N) {
        TARGET_FORCE_N = v;
        Serial.print(F("CMD: target = "));
        Serial.print(TARGET_FORCE_N, 2);
        Serial.println(F(" N"));
      } else {
        Serial.println(F("CMD: target out of range"));
      }
      break;
    }
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(left.fsrPin,  INPUT);
  pinMode(right_.fsrPin, INPUT);
  left.servo.attach(left.servoPin);
  right_.servo.attach(right_.servoPin);
  left.servo.write(left.openDeg);
  right_.servo.write(right_.openDeg);
  left.angle = left.openDeg;
  right_.angle = right_.openDeg;
  delay(500);
  tareBaselines();
  Serial.println(F("Per-jaw force claw ready. Commands: o, c, t<N>, z"));
}

void loop() {
  handleSerial();

  unsigned long now = millis();
  if (now - lastControl < CONTROL_PERIOD_MS) return;
  lastControl = now;

  updateJaw(left);
  updateJaw(right_);

  if (now - lastTelem >= TELEM_PERIOD_MS) {
    lastTelem = now;
    Serial.print(F("L: "));
    Serial.print(left.force, 2);
    Serial.print(F(" N @ "));
    Serial.print(left.angle);
    Serial.print(F("deg   R: "));
    Serial.print(right_.force, 2);
    Serial.print(F(" N @ "));
    Serial.print(right_.angle);
    Serial.print(F("deg   mode="));
    Serial.println(
      mode == MODE_OPEN ? F("open") :
      mode == MODE_CLOSE_TO_FORCE ? F("close") : F("idle"));
  }
}
