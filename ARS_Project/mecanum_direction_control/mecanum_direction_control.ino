// =============================================================
//  MECANUM OPEN-LOOP DIRECTION CONTROL + ENCODER VERIFICATION
//  Arduino Mega 2560 + 2x Cytron MDDRC10  --  BOTH DRIVERS IN IND MODE
//
//  DRIVER SPLIT: LEFT / RIGHT -- both drivers wired the SAME way:
//  front wheel on Ch.1, rear wheel on Ch.2.
//    Driver #1 = LEFT  side -> FL (Ch.1 MLA/MLB) + RL (Ch.2 MRA/MRB)
//    Driver #2 = RIGHT side -> FR (Ch.1 MLA/MLB) + RR (Ch.2 MRA/MRB)
//
//  ---------------------------------------------------------------------
//  PIN MAP REV.F (2026-07) -- CURRENT. History in Pinout_Reference.md.
//  ---------------------------------------------------------------------
//  RC SIGNAL WIRING (direct drive, no PCA9685):
//    pin 12 -> Driver#1 RC1  (Ch.1 = MLA/MLB) -> FL
//    pin 13 -> Driver#1 RC2  (Ch.2 = MRA/MRB) -> RL
//    pin 11 -> Driver#2 RC1  (Ch.1 = MLA/MLB) -> FR
//    pin 10 -> Driver#2 RC2  (Ch.2 = MRA/MRB) -> RR
//  Rev.F moved all four RC pins onto 10-13 (builder's own physical
//  re-wiring). Wire colours are not tracked for these four pins as of
//  Rev.F -- earlier colour tables (Rev.D/E) no longer apply here; verify
//  colour-to-pin by continuity if you need it, don't assume from history.
//  NOTE: "ML"/"MR" are legacy differential-drive labels. In IND mode they are
//  just Channel 1 / Channel 2 -- the words carry no left/right meaning.
//    Arduino GND -> BOTH drivers' SIGNAL GND (pin beside the RC inputs,
//                   NOT VB-). Two wires total.
//
//  MOTOR 6-WIRE COLOUR CODE (JGB37-520) -- VERIFIED:
//    Red    Motor +            -> driver channel terminal
//    White  Motor -            -> driver channel terminal
//    Blue   Encoder VCC        -> Arduino 5V
//    Black  Encoder GND        -> star ground      (NOT a motor lead!)
//    Yellow Encoder A (Hall)   -> Arduino interrupt pin
//    Green  Encoder B (Hall)   -> Arduino digital pin
//  WARNING: Black is encoder GND, NOT motor negative. The motor pair is
//  RED + WHITE. Never land White on an Arduino pin, never land Black on a
//  driver terminal.
//
//  ENCODER WIRING (A-channels MUST be on external-interrupt pins, unchanged
//  since Rev.C -- Rev.F only touched the four RC pins above):
//    Only 2, 3, 18, 19, 20, 21 are interrupt-capable on the Mega, and 20/21
//    are reserved for I2C. That leaves exactly {2,3} at one end of the board
//    and {18,19} at the other -- so the interrupt pairs themselves dictate
//    the left/right split.
//   LEFT block                     RIGHT block
//    FL: A=18 (INT5)  B=16           FR: A=2  (INT0)  B=4
//    RL: A=19 (INT4)  B=17           RR: A=3  (INT1)  B=5
//    All encoder VCC (blue) -> Arduino 5V ; all encoder GND (black) -> star ground
//
//  UNUSED / FREE as of Rev.F: 6, 7, 14, 15 (Serial3), 22-25, and the entire
//  rear 22-53 header. 20/21 stay clear for I2C. Pin 13 is now IN USE
//  (PIN_RL) -- the on-board LED will flicker with the RL servo pulse, this
//  is cosmetic only.
//
//  Serial Monitor @ 115200. Commands:
//    w forward   b backward   a strafe-left   d strafe-right
//    q diag F-L  e diag F-R   z diag B-L      c diag B-R
//    j turn-left l turn-right s stop
//    + / -  increase / decrease speed
//    t  toggle encoder telemetry    r  reset tick counters
//    k  ENCODER CHECK (motors forced off, ticks streamed)
//    h  help
//
//  ---------------------------------------------------------------------
//  ROS2 DIRECT-THROTTLE COMMAND (added for the serial bridge)
//  ---------------------------------------------------------------------
//    M <fl> <fr> <rl> <rr>\n     four floats, each -1.0 .. +1.0
//
//  e.g.  "M 0.42 -0.18 -0.18 0.42\n"  -> strafe right at part throttle.
//  Values are ABSOLUTE throttles: unlike the letter keys they are NOT
//  scaled by 'spd', so the host owns the speed profile entirely. DIR_* is
//  still applied (that is physical wiring correction, not policy), as is
//  the deadband push and MAX_THROTTLE clamp in writeWheel().
//
//  Any motion letter returns the board to key control, so a human can
//  always take over mid-session without a reset.
//
//  SAFETY: while in M mode the hold-to-move watchdog is ALWAYS armed,
//  regardless of the y toggle -- if the host stops sending M lines for
//  MOTION_TIMEOUT_MS the robot stops on its own. Stream M at >= 10 Hz.
// =============================================================

#include <Servo.h>

// ---------- RC output pins (front=Ch.1, rear=Ch.2 on BOTH drivers) --------
const int PIN_FL = 12;   // Driver#1 (LEFT)  RC1 -> Ch.1 MLA/MLB   [REV.F]
const int PIN_RL = 13;   // Driver#1 (LEFT)  RC2 -> Ch.2 MRA/MRB   [REV.F]
const int PIN_FR = 11;   // Driver#2 (RIGHT) RC1 -> Ch.1 MLA/MLB   [REV.F]
const int PIN_RR = 10;   // Driver#2 (RIGHT) RC2 -> Ch.2 MRA/MRB   [REV.F]
// REV.F: all four RC pins moved onto 10-13 (builder's own re-wiring).
// Encoder pins are unaffected -- still 2,3,4,5 (RIGHT) and 16,17,18,19
// (LEFT). No conflicts: all twelve pin values are unique.

// ---------- Encoder pins ---------------------------------------------------
// A must be interrupt-capable. LEFT gets {18,19}, RIGHT gets {2,3}.
// LEFT block REV.C: moved off the rear 22-53 header onto pins 14-17, which
// sit on the SAME primary header as 18/19 -- shortest possible run to
// Driver#1, no detour to the rear double header at all.
const uint8_t ENCA_FL = 18, ENCB_FL = 16;   // INT5
const uint8_t ENCA_RL = 19, ENCB_RL = 17;   // INT4
const uint8_t ENCA_FR = 2,  ENCB_FR = 4;    // INT0
const uint8_t ENCA_RR = 3,  ENCB_RR = 5;    // INT1

// ---------- RC signal shaping ----------------------------------------------
const int   NEUTRAL      = 1500;   // us -> stop
const int   DEFLECT      = 435;    // us -> full scale (MDDRC10 spec)
// Full scale. 1.0 = the driver's full +/-435us deflection, which is this
// hardware's equivalent of "PWM 255" -- there is no 0-255 PWM anywhere in
// this chain, the MDDRC10 is commanded by RC servo pulse width.
//
// Raised from 0.85 on 2026-07-22 because the robot could not rotate under its
// own weight: the chassis is heavy and 0.85 left too little torque.
//
// TRADE-OFF, know this: the JGB37-520 motors are 6V nominal and a 2S pack is
// 7.4V, so sustained full throttle overvolts them by ~23%. In practice the
// supply sags hard under four motors, which limits it anyway -- but do not
// hold full throttle for long stretches. Drop this back toward 0.85 if the
// motors get hot.
const float MAX_THROTTLE = 1.00;

// The MDDRC10 ignores +/-35us either side of neutral. Anything below this
// fraction of full scale produces NO motion -- which looks exactly like a
// dead wheel. Small commands get pushed past it instead of vanishing.
const float DEADBAND_THROTTLE = 35.0f / 435.0f;   // ~0.080

float spd = 0.4;                   // base speed (0.0 .. 0.85)

// ===========================================================================
//  CALIBRATION -- turns raw ticks into RPM and m/s
//
//  Both values come from the separate encoder_calibration sketch. Run that
//  once, paste its two output lines here, re-upload. While TICKS_PER_REV is
//  0 the telemetry falls back to raw ticks/second, so this still runs fine
//  uncalibrated.
// ===========================================================================
// Measured 2026-07: FL 69.5, FR 70.2, RL 71.3, RR 71.2 over 10 turns each.
const float TICKS_PER_REV    = 70.6f;    // averaged
const float WHEEL_DIAMETER_M = 0.095f;   // measured 2026-07, across the rollers

// Derived. Ticks the wheel produces over one metre of travel.
float ticksPerMeter() {
  return TICKS_PER_REV / (PI * WHEEL_DIAMETER_M);
}
bool calibrated() { return TICKS_PER_REV > 0.5f; }

// If a wheel SPINS the wrong way on 'w', flip its sign here
// (or swap that motor's two power leads at the driver).
// LEFT flipped to -1: on 'w' the left side was turning opposite to the
// right side. Both sides get the same +1 command from move(), so a visible
// mismatch here means the two sides' motors are mirror-mounted relative to
// each other -- flipping LEFT's sign brings both sides in sync. This
// corrects 'b' (backward) the same way, since 'b' just negates all four
// direction multipliers uniformly -- the left/right relationship is
// unchanged, so one fix covers both directions.
const int DIR_FL = -1, DIR_FR = +1, DIR_RL = -1, DIR_RR = +1;

// If a wheel COUNTS DOWN while rolling forward, flip its sign here.
// LEFT side is -1 because those motors are mirror-mounted relative to the
// right side, so they turn the opposite absolute direction when the robot
// moves forward. Confirmed during calibration: FL and RL both read negative.
const int ENC_DIR_FL = -1, ENC_DIR_FR = +1, ENC_DIR_RL = -1, ENC_DIR_RR = +1;

// ---------- State ----------------------------------------------------------
Servo mFL, mFR, mRL, mRR;

volatile long tickFL = 0, tickFR = 0, tickRL = 0, tickRR = 0;

bool telemetry  = false;
bool checkMode  = false;           // true -> motors forced off

// true while the ROS2 bridge is driving via "M" lines. In this mode dFL..dRR
// hold absolute throttles rather than -1/0/+1 direction multipliers, so
// applyMotion() must not scale them by spd. Cleared by any motion letter.
bool directMode = false;

// ---------- Hold-to-move ---------------------------------------------------
// Serial has no "key released" event, so we infer release from silence: the
// host repeats the motion character while the key is held, and if nothing
// arrives for MOTION_TIMEOUT_MS we assume it was let go and stop.
// Use teleop_keyboard.py on the PC -- the Arduino Serial Monitor is
// line-buffered and cannot drive this.
bool holdMode = true;              // 'y' toggles hold vs latched
const unsigned long MOTION_TIMEOUT_MS = 200;
unsigned long lastMotionMs = 0;
float dFL = 0, dFR = 0, dRL = 0, dRR = 0;   // held direction multipliers
unsigned long lastReport = 0;
const unsigned long REPORT_MS = 100;        // 10 Hz while bench testing

// previous tick snapshot, used to derive ticks/second
long prevFL = 0, prevFR = 0, prevRL = 0, prevRR = 0;

// ---------- Encoder ISRs ---------------------------------------------------
// 1x decoding: count on the rising edge of A, read B to infer direction.
void isrFL() { if (digitalRead(ENCB_FL)) tickFL -= ENC_DIR_FL; else tickFL += ENC_DIR_FL; }
void isrFR() { if (digitalRead(ENCB_FR)) tickFR -= ENC_DIR_FR; else tickFR += ENC_DIR_FR; }
void isrRL() { if (digitalRead(ENCB_RL)) tickRL -= ENC_DIR_RL; else tickRL += ENC_DIR_RL; }
void isrRR() { if (digitalRead(ENCB_RR)) tickRR -= ENC_DIR_RR; else tickRR += ENC_DIR_RR; }

long readTicks(volatile long &t) {
  noInterrupts();
  long v = t;
  interrupts();
  return v;
}

// ---------- Output ---------------------------------------------------------
void writeWheel(Servo &m, float t) {
  t = constrain(t, -MAX_THROTTLE, MAX_THROTTLE);
  if (t != 0.0f && fabs(t) < DEADBAND_THROTTLE) {
    t = (t > 0) ? DEADBAND_THROTTLE : -DEADBAND_THROTTLE;
  }
  m.writeMicroseconds(NEUTRAL + (int)(t * DEFLECT));
}

// fl, fr, rl, rr are direction multipliers (-1, 0, +1); scaled by spd
void applyMotion() {
  if (checkMode) {                 // encoder check: everything held stopped
    writeWheel(mFL, 0); writeWheel(mFR, 0);
    writeWheel(mRL, 0); writeWheel(mRR, 0);
    return;
  }
  if (directMode) {                // "M" lines carry absolute throttles
    writeWheel(mFL, dFL * DIR_FL);
    writeWheel(mFR, dFR * DIR_FR);
    writeWheel(mRL, dRL * DIR_RL);
    writeWheel(mRR, dRR * DIR_RR);
    return;
  }
  writeWheel(mFL, dFL * spd * DIR_FL);
  writeWheel(mFR, dFR * spd * DIR_FR);
  writeWheel(mRL, dRL * spd * DIR_RL);
  writeWheel(mRR, dRR * spd * DIR_RR);
}

void move(float fl, float fr, float rl, float rr) {
  dFL = fl; dFR = fr; dRL = rl; dRR = rr;
  lastMotionMs = millis();     // refreshes the hold-to-move watchdog
  applyMotion();
}

bool isMoving() { return dFL != 0 || dFR != 0 || dRL != 0 || dRR != 0; }

void help() {
  Serial.println(F("w=fwd b=back a=strafeL d=strafeR"));
  Serial.println(F("q=diagFL e=diagFR z=diagBL c=diagBR"));
  Serial.println(F("j=turnL l=turnR s=stop  +/- speed"));
  Serial.println(F("t=telemetry r=reset-ticks k=encoder-check h=help"));
  Serial.println(F("y=toggle HOLD/LATCH  (HOLD needs teleop_keyboard.py)"));
  Serial.println(F("M fl fr rl rr = direct throttles (ROS2 bridge)"));
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);     // bounds the "M" line read; default 1000 ms is
                             // far too long to sit inside loop()

  mFL.attach(PIN_FL); mFR.attach(PIN_FR);
  mRL.attach(PIN_RL); mRR.attach(PIN_RR);
  move(0, 0, 0, 0);          // hold stop

  pinMode(ENCA_FL, INPUT_PULLUP); pinMode(ENCB_FL, INPUT_PULLUP);
  pinMode(ENCA_FR, INPUT_PULLUP); pinMode(ENCB_FR, INPUT_PULLUP);
  pinMode(ENCA_RL, INPUT_PULLUP); pinMode(ENCB_RL, INPUT_PULLUP);
  pinMode(ENCA_RR, INPUT_PULLUP); pinMode(ENCB_RR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_FL), isrFL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_FR), isrFR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_RL), isrRL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_RR), isrRR, RISING);

  delay(2000);               // let BOTH drivers auto-calibrate neutral
  Serial.println(F("Mecanum ready. LEFT/RIGHT driver split."));
  // TEMP DEBUG: prints the actual compiled DIR_* values so you can confirm
  // the board is running THIS file and not a stale/duplicate copy. Remove
  // once the direction issue is confirmed fixed.
  Serial.print(F("DIR_FL=")); Serial.print(DIR_FL);
  Serial.print(F(" DIR_FR=")); Serial.print(DIR_FR);
  Serial.print(F(" DIR_RL=")); Serial.print(DIR_RL);
  Serial.print(F(" DIR_RR=")); Serial.println(DIR_RR);
  help();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    // Any motion letter hands control back to the keys, so a human can take
    // over from the ROS2 bridge mid-session without power-cycling the board.
    if (strchr("wbadqezcjls", c)) directMode = false;

    switch (c) {
      case 'w': move(+1, +1, +1, +1); Serial.println(F("forward"));        break;
      case 'b': move(-1, -1, -1, -1); Serial.println(F("backward"));       break;
      case 'd': move(+1, -1, -1, +1); Serial.println(F("strafe right"));   break;
      case 'a': move(-1, +1, +1, -1); Serial.println(F("strafe left"));    break;
      case 'e': move(+1,  0,  0, +1); Serial.println(F("diag fwd-right")); break;
      case 'q': move( 0, +1, +1,  0); Serial.println(F("diag fwd-left"));  break;
      case 'c': move( 0, -1, -1,  0); Serial.println(F("diag back-right"));break;
      case 'z': move(-1,  0,  0, -1); Serial.println(F("diag back-left")); break;
      case 'l': move(+1, -1, +1, -1); Serial.println(F("turn right"));     break;
      case 'j': move(-1, +1, -1, +1); Serial.println(F("turn left"));      break;
      case 's': move( 0,  0,  0,  0); Serial.println(F("stop"));           break;

      case '+': spd = min(spd + 0.05f, MAX_THROTTLE);
                applyMotion(); Serial.print(F("speed=")); Serial.println(spd); break;
      case '-': spd = max(spd - 0.05f, 0.0f);
                applyMotion(); Serial.print(F("speed=")); Serial.println(spd); break;

      // NOTE: F() must not appear twice in one expression (avr-gcc section
      // type conflict), so these use if/else rather than a ternary.
      case 'k': checkMode = !checkMode;
                move(0, 0, 0, 0);
                if (checkMode) {
                  telemetry = true;
                  Serial.println(F("ENCODER CHECK: motors OFF, roll wheels by hand"));
                } else {
                  Serial.println(F("drive mode"));
                }
                break;
      case 't': telemetry = !telemetry;
                if (telemetry) Serial.println(F("telemetry ON"));
                else           Serial.println(F("telemetry OFF"));
                break;
      case 'r': noInterrupts(); tickFL = tickFR = tickRL = tickRR = 0; interrupts();
                prevFL = prevFR = prevRL = prevRR = 0;
                Serial.println(F("ticks reset"));
                break;
      case 'y': holdMode = !holdMode;
                move(0, 0, 0, 0);
                if (holdMode) Serial.println(F("HOLD mode: stops when key released"));
                else          Serial.println(F("LATCH mode: runs until s"));
                break;

      case 'h': case '?': help(); break;

      // ---- ROS2 bridge: "M <fl> <fr> <rl> <rr>" absolute throttles --------
      // Bounded read: Serial.setTimeout(20) caps the wait, and a full line is
      // ~24 bytes = ~2 ms at 115200, so this cannot stall the loop long
      // enough to starve the watchdog.
      case 'M': {
        char buf[48];
        size_t n = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
        buf[n] = '\0';

        float v[4];
        char *p = buf;
        bool ok = true;
        for (uint8_t i = 0; i < 4; i++) {
          char *end;
          v[i] = strtod(p, &end);
          if (end == p) { ok = false; break; }   // no number parsed -> junk
          p = end;
        }
        // A malformed line is ignored rather than treated as zeros: the
        // watchdog below already stops us if the host has genuinely gone
        // away, and reacting to line noise with a hard stop would make the
        // robot stutter on every corrupted byte.
        if (ok) {
          directMode = true;
          move(v[0], v[1], v[2], v[3]);
        }
        break;
      }
    }
  }

  // ---- hold-to-move watchdog: silence means the key was released ----------
  // Armed in directMode regardless of holdMode -- if the ROS2 bridge or the
  // USB link dies, the robot must not keep driving.
  if ((holdMode || directMode) && !checkMode && isMoving() &&
      millis() - lastMotionMs > MOTION_TIMEOUT_MS) {
    dFL = dFR = dRL = dRR = 0;
    applyMotion();
  }

  unsigned long nowMs = millis();
  if (telemetry && nowMs - lastReport >= REPORT_MS) {
    float dt = (nowMs - lastReport) / 1000.0f;
    lastReport = nowMs;

    long cFL = readTicks(tickFL), cFR = readTicks(tickFR);
    long cRL = readTicks(tickRL), cRR = readTicks(tickRR);

    // ticks/second -- no calibration needed, so this works from day one.
    // Use it to compare wheels against EACH OTHER: on 'w' all four rates
    // should be similar. An outlier means that wheel is dragging.
    long rFL = (long)((cFL - prevFL) / dt);
    long rFR = (long)((cFR - prevFR) / dt);
    long rRL = (long)((cRL - prevRL) / dt);
    long rRR = (long)((cRR - prevRR) / dt);
    prevFL = cFL; prevFR = cFR; prevRL = cRL; prevRR = cRR;

    // "E" = cumulative ticks (the ROS2 contract line -- do not reformat)
    Serial.print(F("E "));
    Serial.print(cFL); Serial.print(' ');
    Serial.print(cFR); Serial.print(' ');
    Serial.print(cRL); Serial.print(' ');
    Serial.println(cRR);

    if (!calibrated()) {
      // Uncalibrated fallback: raw rate. Still useful for comparing wheels.
      Serial.print(F("R "));
      Serial.print(rFL); Serial.print(' ');
      Serial.print(rFR); Serial.print(' ');
      Serial.print(rRL); Serial.print(' ');
      Serial.print(rRR); Serial.println(F("  ticks/s (set TICKS_PER_REV for rpm/m/s)"));
    } else {
      float tpm = ticksPerMeter();

      Serial.print(F("RPM "));
      Serial.print(rFL * 60.0f / TICKS_PER_REV, 0); Serial.print(' ');
      Serial.print(rFR * 60.0f / TICKS_PER_REV, 0); Serial.print(' ');
      Serial.print(rRL * 60.0f / TICKS_PER_REV, 0); Serial.print(' ');
      Serial.print(rRR * 60.0f / TICKS_PER_REV, 0);

      Serial.print(F(" | m/s "));
      Serial.print(rFL / tpm, 2); Serial.print(' ');
      Serial.print(rFR / tpm, 2); Serial.print(' ');
      Serial.print(rRL / tpm, 2); Serial.print(' ');
      Serial.print(rRR / tpm, 2);

      // distance travelled by each wheel since the last 'r'
      Serial.print(F(" | dist "));
      Serial.print(cFL / tpm, 2); Serial.print(' ');
      Serial.print(cFR / tpm, 2); Serial.print(' ');
      Serial.print(cRL / tpm, 2); Serial.print(' ');
      Serial.print(cRR / tpm, 2); Serial.println(F(" m"));
    }
  }
}
