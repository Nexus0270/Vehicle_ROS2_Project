// =============================================================
//  ENCODER CALIBRATION UTILITY  --  Arduino Mega 2560
//
//  Measures TICKS_PER_REV for each of the four wheels, then derives
//  TICKS_PER_METER. Run this ONCE, copy the results into
//  mecanum_direction_control.ino and mecanum_closed_loop.ino.
//
//  THIS SKETCH NEVER DRIVES THE MOTORS. It holds both drivers at neutral
//  and you turn the wheels by hand. Safe to run with the 2S connected,
//  though running with motor power OFF is simpler still.
//
//  ENCODER WIRING -- same as the drive sketches, unchanged since Rev.C:
//   LEFT block                     RIGHT block
//    FL: A=18 (INT5)  B=16          FR: A=2  (INT0)  B=4
//    RL: A=19 (INT4)  B=17          RR: A=3  (INT1)  B=5
//    Blue (Enc VCC) -> 5V rail ; Black (Enc GND) -> GND rail
//
//  RC neutral hold (so the drivers stay calm and their ERR LED stays off)
//  -- PIN MAP REV.F, current:
//    pin 12 -> D1 RC1 (FL) | pin 13 -> D1 RC2 (RL)
//    pin 11 -> D2 RC1 (FR) | pin 10 -> D2 RC2 (RR)
//  Both drivers are wired the same way: front on Ch.1, rear on Ch.2.
//
//  ---- PROCEDURE ----
//    1. Lift the robot so all four wheels spin freely.
//    2. Mark one spoke of each wheel with tape as a reference.
//    3. Press 'z' to zero the counters.
//    4. Rotate ONE wheel slowly and steadily through exactly
//       REVOLUTIONS full turns, finishing on the tape mark.
//    5. Press 'c'. Read that wheel's ticks/rev.
//    6. Repeat 3-5 for the other three wheels.
//
//  Serial Monitor @ 115200.
//    z  zero counters        c  compute + report
//    l  toggle live counts   h  help
// =============================================================

#include <Servo.h>

// ---------- How many turns you will rotate per measurement ----------------
// More turns = more accuracy, because the 1-tick error at the start and end
// gets divided across more revolutions. 10 is a good balance.
const float REVOLUTIONS = 10.0f;

// ---------- Wheel diameter, measured ACROSS THE ROLLERS, in metres --------
const float WHEEL_DIAMETER_M = 0.095f;   // measured 2026-07, across the rollers

// ---------- Encoder pins (unchanged since Rev.C) ---------------------------
const uint8_t ENCA_FL = 18, ENCB_FL = 16;   // INT5
const uint8_t ENCA_RL = 19, ENCB_RL = 17;   // INT4
const uint8_t ENCA_FR = 2,  ENCB_FR = 4;    // INT0
const uint8_t ENCA_RR = 3,  ENCB_RR = 5;    // INT1

// ---------- RC pins (neutral hold only -- never driven) -------------------
const int PIN_FL = 12, PIN_RL = 13, PIN_FR = 11, PIN_RR = 10;   // REV.F
const int NEUTRAL = 1500;

Servo mFL, mFR, mRL, mRR;

volatile long tickFL = 0, tickFR = 0, tickRL = 0, tickRR = 0;

bool live = true;
unsigned long lastPrint = 0;

// ---------- Encoder ISRs ---------------------------------------------------
void isrFL() { if (digitalRead(ENCB_FL)) tickFL--; else tickFL++; }
void isrFR() { if (digitalRead(ENCB_FR)) tickFR--; else tickFR++; }
void isrRL() { if (digitalRead(ENCB_RL)) tickRL--; else tickRL++; }
void isrRR() { if (digitalRead(ENCB_RR)) tickRR--; else tickRR++; }

long readTicks(volatile long &t) {
  noInterrupts();
  long v = t;
  interrupts();
  return v;
}

void zeroAll() {
  noInterrupts();
  tickFL = tickFR = tickRL = tickRR = 0;
  interrupts();
  Serial.println(F("zeroed -- now rotate one wheel"));
}

void help() {
  Serial.println(F("z=zero  c=compute  l=live counts  h=help"));
  Serial.print(F("Rotate each wheel exactly "));
  Serial.print(REVOLUTIONS, 0);
  Serial.println(F(" full turns between z and c."));
}

void report() {
  live = false;              // otherwise the live stream scrolls this away
  long cFL = readTicks(tickFL), cFR = readTicks(tickFR);
  long cRL = readTicks(tickRL), cRR = readTicks(tickRR);

  Serial.println(F("--------------------------------------------"));
  Serial.print(F("raw ticks     FL=")); Serial.print(cFL);
  Serial.print(F("  FR="));             Serial.print(cFR);
  Serial.print(F("  RL="));             Serial.print(cRL);
  Serial.print(F("  RR="));             Serial.println(cRR);

  float pFL = fabs(cFL) / REVOLUTIONS;
  float pFR = fabs(cFR) / REVOLUTIONS;
  float pRL = fabs(cRL) / REVOLUTIONS;
  float pRR = fabs(cRR) / REVOLUTIONS;

  Serial.print(F("ticks/rev     FL=")); Serial.print(pFL, 1);
  Serial.print(F("  FR="));             Serial.print(pFR, 1);
  Serial.print(F("  RL="));             Serial.print(pRL, 1);
  Serial.print(F("  RR="));             Serial.println(pRR, 1);

  // Average only the wheels that actually moved, so you can measure one
  // wheel at a time without the three idle wheels dragging the mean to zero.
  float sum = 0; int n = 0;
  if (pFL > 1) { sum += pFL; n++; }
  if (pFR > 1) { sum += pFR; n++; }
  if (pRL > 1) { sum += pRL; n++; }
  if (pRR > 1) { sum += pRR; n++; }

  if (n == 0) {
    Serial.println(F("no movement detected -- check wiring or rotate more"));
    Serial.println(F("--------------------------------------------"));
    return;
  }

  float avg = sum / n;
  float tpm = avg / (PI * WHEEL_DIAMETER_M);

  Serial.print(F("averaged over ")); Serial.print(n); Serial.println(F(" moved wheel(s)"));
  Serial.println(F("--- paste into the drive sketches ---"));
  Serial.print(F("const float TICKS_PER_REV    = "));
  Serial.print(avg, 1); Serial.println(F("f;"));
  Serial.print(F("const float WHEEL_DIAMETER_M = "));
  Serial.print(WHEEL_DIAMETER_M, 4); Serial.println(F("f;"));
  Serial.print(F("// implies TICKS_PER_METER = "));
  Serial.println(tpm, 1);
  Serial.println(F("--------------------------------------------"));

  // Direction sanity note: rolling a wheel FORWARD should give a POSITIVE
  // raw count. Negative means that wheel's ENC_DIR_* needs to be -1.
  Serial.println(F("NOTE: forward rotation should give POSITIVE ticks."));
  Serial.println(F("Any negative wheel above needs ENC_DIR_* = -1."));
  Serial.println(F("(left and right sides counting opposite is NORMAL --"));
  Serial.println(F(" the motors are mirror-mounted)"));
  Serial.println(F("live display paused -- press l to resume, z to restart"));
}

void setup() {
  Serial.begin(115200);

  // Hold both drivers at neutral so nothing can move and ERR stays off
  mFL.attach(PIN_FL); mFR.attach(PIN_FR);
  mRL.attach(PIN_RL); mRR.attach(PIN_RR);
  mFL.writeMicroseconds(NEUTRAL); mFR.writeMicroseconds(NEUTRAL);
  mRL.writeMicroseconds(NEUTRAL); mRR.writeMicroseconds(NEUTRAL);

  pinMode(ENCA_FL, INPUT_PULLUP); pinMode(ENCB_FL, INPUT_PULLUP);
  pinMode(ENCA_FR, INPUT_PULLUP); pinMode(ENCB_FR, INPUT_PULLUP);
  pinMode(ENCA_RL, INPUT_PULLUP); pinMode(ENCB_RL, INPUT_PULLUP);
  pinMode(ENCA_RR, INPUT_PULLUP); pinMode(ENCB_RR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_FL), isrFL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_FR), isrFR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_RL), isrRL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_RR), isrRR, RISING);

  delay(2000);
  Serial.println(F("=== ENCODER CALIBRATION -- motors will NOT move ==="));
  help();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if      (c == 'z') zeroAll();
    else if (c == 'c') report();
    else if (c == 'l') { live = !live; Serial.println(F("live toggled")); }
    else if (c == 'h' || c == '?') help();
  }

  if (live && millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.print(F("FL ")); Serial.print(readTicks(tickFL));
    Serial.print(F("  FR ")); Serial.print(readTicks(tickFR));
    Serial.print(F("  RL ")); Serial.print(readTicks(tickRL));
    Serial.print(F("  RR ")); Serial.println(readTicks(tickRR));
  }
}
