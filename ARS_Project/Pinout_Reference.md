# Pinout Reference — Mecanum Drive + Encoders

**PIN MAP REV.F (2026-07) — current.** Matches `mecanum_direction_control.ino`,
`mecanum_closed_loop.ino`, and `encoder_calibration.ino`.

Both drivers are wired the **same** way: front wheel on Ch.1, rear wheel on
Ch.2. Driver #1 is the LEFT edge, Driver #2 is the RIGHT edge.

```
PIN_FL = 12    ENCA_FL = 18    ENCB_FL = 16
PIN_RL = 13    ENCA_RL = 19    ENCB_RL = 17
PIN_FR = 11    ENCA_FR =  2    ENCB_FR =  4
PIN_RR = 10    ENCA_RR =  3    ENCB_RR =  5
```

---

## Revision history (condensed)

| Rev | What changed | Why |
| :--- | :--- | :--- |
| A | FL=9, RL=10, FR=11, RR=12; encoder A split front/rear (2,3,18,19) | Original layout, grouped by function |
| B | Pins regrouped by chassis side instead of function | Rev.A forced FL/RL signal wires to cross the whole chassis to reach Driver #1 |
| C | LEFT RC + encoder-B moved off the rear 22–53 header onto 14–17 | Same primary header as 18/19 — shorter physical run, no second connector |
| D | LEFT RC pins moved again, to 10/11 | Builder's own physical routing to Driver #1, solved directly on the chassis |
| E | RIGHT RC pins 8↔9 swapped | Confirmed on the physical board that pin 8 was wired to the RC2 header and pin 9 to RC1 — opposite of what was assumed |
| **F** | **All four RC pins re-wired onto 10–13** (FL=12, RL=13, FR=11, RR=10) | Builder's own physical re-wiring while chasing an ERR-LED fault that moved between drivers as pins changed |

Encoder pins (2, 3, 4, 5, 16, 17, 18, 19) have been stable since Rev.C and are
untouched by D, E, or F — only RC signal pins have moved.

**Wire colour is not tracked for the four RC pins as of Rev.F.** Earlier
colour tables (Rev.D/E) assumed specific jumpers; since Rev.F reused pins
that previously belonged to other signals, don't trust those colour
associations any more. Verify colour-to-pin by continuity if you need it.

---

## 0. Why the pins are grouped this way

### The interrupt constraint drives the encoder layout

Encoder A-channels must sit on external-interrupt pins. On the Mega only
2, 3, 18, 19, 20, 21 qualify, and 20/21 are reserved for I²C — leaving
exactly four usable pins in two fixed pairs: **{2, 3}** and **{18, 19}**.
Those two pairs sit at opposite ends of the header, so the hardware itself
decided the left/right encoder split; nothing here is arbitrary.

Only the **A** channel needs an interrupt pin — it fires the ISR that counts
ticks. **B** is only read *inside* that ISR to determine direction, so it can
be any ordinary digital pin. This is why each wheel's A and B are NOT on
consecutive pins (e.g. FR is not simply "2 and 3") — giving one wheel both
of a side's interrupt pins would strand the other wheel on that side with no
interrupt at all. The grouping is by **function within each side**:

- **2, 3** → both A channels, RIGHT side
- **4, 5** → both B channels, RIGHT side
- **18, 19** → both A channels, LEFT side
- **16, 17** → both B channels, LEFT side

### Why the RC pins don't follow the same tidy pattern any more

Rev.B through D tried to keep RC signal pins physically adjacent to their
side's encoder cluster, for cable-management reasons (short runs, one
connector per side). Rev.E and Rev.F prioritized something else instead:
matching the *actual physical wiring* the builder ended up with while
debugging a real hardware fault (an ERR indicator that moved between drivers
as pins were reassigned). **Functionally this is fine** — the code doesn't
care whether a pin is "tidy," only that `PIN_FR` is the wire that truly
reaches FR's motor. Physically it means the RC pins (10–13) and the encoder
pins (2–5, 16–19) are no longer one contiguous cluster per side. That's a
cosmetic/cable-routing cost, not a functional one.

### Physical pin order along the Mega header

Reading the 0–21 header from the USB-B end toward the far end:

```
[ 21  20  AREF  GND  13 12 11 10  9  8 ] [ 7  6  5  4  3  2  1  0 ] [ 14 15 16 17 18 19 ]
   ^ USB end                                                              far end ^
```

Pins 10–13 (now the RC pins) sit near the USB end. Under the orientation
below, that's the robot's **RIGHT** — meaning as of Rev.F, LEFT's RC pins
(12, 13) are *not* physically near LEFT's encoder pins (16–19, at the far
end). This was a deliberate trade made while chasing the ERR fault; if it
ever becomes a cable-routing problem, the "tidy" alternative is still 14/15
(free, on the far/LEFT end) for FL/RL and 8/9 (free, near end) for FR/RR —
but don't change pins without re-confirming against the physical board first,
the way Rev.E and F both had to.

### ⚠️ THE ORIENTATION ASSUMPTION — confirm this before you cut anything

From the build photos the Mega sits in the centre recess with its **USB-B /
barrel-jack end facing the robot's RIGHT**. Under that orientation, pins near
the USB end (2–13) face RIGHT, and pins near the far end (14–19, and the
rear 22–53 header) face LEFT.

**If the board is actually rotated 180°, swap the LEFT and RIGHT constant
assignments in all three sketches.** Check this by eye: which side does the
USB cable exit from?

---

## 1. Motor power leads → MDDRC10 terminals

Each motor's **Red + White** pair only. Never White or Red near the Arduino.

| Driver | Terminal | Motor | Wires |
| :--- | :--- | :--- | :--- |
| **#1 (LEFT)** | MLA / MLB (Ch.1) | **FL** | Red / White |
| **#1 (LEFT)** | MRA / MRB (Ch.2) | **RL** | Red / White |
| **#2 (RIGHT)** | MLA / MLB (Ch.1) | **FR** | Red / White |
| **#2 (RIGHT)** | MRA / MRB (Ch.2) | **RR** | Red / White |

Confirmed: front wheels on MLA/MLB, rear wheels on MRA/MRB, Driver #1 left,
Driver #2 right. `ML`/`MR` are legacy differential-drive labels on the driver
silkscreen — in IND mode they are just Channel 1 / Channel 2, the naming
carries no left/right meaning.

⚠ **RC1 always drives Channel 1 (MLA/MLB). RC2 always drives Channel 2
(MRA/MRB). This is fixed by the board, not by wiring choice.** The Rev.E fault
happened because a jumper was plugged into the wrong physical RC header, not
because the board's RC1↔Channel1 relationship changed. If a wheel spins from
the wrong command again, check which physical header (labelled on the driver
board silkscreen) each Arduino pin's jumper is actually seated in — don't
assume from a pin number alone.

---

## 2. RC signal wires: Arduino → driver RC inputs

| Arduino pin | Driver | RC input | Channel | Terminal | Wheel |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **12** | #1 (LEFT) | RC1 | Ch.1 | MLA/MLB | **FL** |
| **13** | #1 (LEFT) | RC2 | Ch.2 | MRA/MRB | **RL** |
| **11** | #2 (RIGHT) | RC1 | Ch.1 | MLA/MLB | **FR** |
| **10** | #2 (RIGHT) | RC2 | Ch.2 | MRA/MRB | **RR** |

No wire colours are documented for these four as of Rev.F — see the revision
history note above. Label both ends physically (`FL-sig`, `RR-sig`, etc.)
since colour can't be relied on here.

Servo library note: none of pins 10–13 are hardware PWM-capable in the sense
`analogWrite` would need, and that's fine — `Servo` on the Mega bit-bangs
pulses from a single timer (Timer5, with four servos attached) and works on
any digital pin regardless of its native PWM capability.

---

## 3. Motor 6-wire colour code (JGB37-520)

Unchanged since the original build.

| Wire | Function | Goes to |
| :--- | :--- | :--- |
| **Red** | Motor + | Driver channel terminal |
| **White** | Motor − | Driver channel terminal |
| **Blue** | Encoder VCC | Breadboard `+` rail → Arduino 5V |
| **Black** | Encoder GND | Breadboard `−` rail → Arduino GND |
| **Yellow** | Encoder A (Hall) | Arduino interrupt pin |
| **Green** | Encoder B (Hall) | Arduino digital pin |

⚠️ Black is encoder ground, **not** motor negative. The motor pair is
Red + White. Never land White on an Arduino pin; never land Black on a
driver terminal.

---

## 4. Encoders → Arduino

| Side | Motor | Yellow (A) → pin | Green (B) → pin |
| :--- | :--- | :--- | :--- |
| LEFT | **FL** | 18 (INT5) | 16 |
| LEFT | **RL** | 19 (INT4) | 17 |
| RIGHT | **FR** | 2 (INT0) | 4 |
| RIGHT | **RR** | 3 (INT1) | 5 |

Unchanged since Rev.C — none of the RC pin changes (D, E, F) touched these.
Pins 18/19 are Serial1's TX/RX and 16/17 are Serial2's TX/RX; none used as
serial in this project, all safe as plain digital/interrupt pins.

**Free / unused pins as of Rev.F:** 6, 7, 14, 15 (Serial3), 22–25, and the
entire rear 22–53 header. 20/21 stay clear for I²C. Pin 13 is now **in use**
(`PIN_RL`) — the on-board LED will flicker with the RL servo pulse; this is
cosmetic only and not a fault.

---

## 5. Full pin map at a glance

### By Arduino pin

| Pin | Function | Wheel | Side | Other end |
| :--- | :--- | :--- | :--- | :--- |
| **2** | Encoder A (INT0) | FR | RIGHT | FR motor |
| **3** | Encoder A (INT1) | RR | RIGHT | RR motor |
| **4** | Encoder B | FR | RIGHT | FR motor |
| **5** | Encoder B | RR | RIGHT | RR motor |
| **10** | RC signal | RR | RIGHT | Driver#2 **RC2** |
| **11** | RC signal | FR | RIGHT | Driver#2 **RC1** |
| **12** | RC signal | FL | LEFT | Driver#1 **RC1** |
| **13** | RC signal | RL | LEFT | Driver#1 **RC2** |
| **16** | Encoder B | FL | LEFT | FL motor |
| **17** | Encoder B | RL | LEFT | RL motor |
| **18** | Encoder A (INT5) | FL | LEFT | FL motor |
| **19** | Encoder A (INT4) | RL | LEFT | RL motor |
| **5V** | rail feed | — | centre | RIGHT strip `+` |
| **GND** | rail feed | — | centre | RIGHT strip `−` |

### By wheel

| Wheel | Side | Driver + terminal | RC pin → input | Enc A | Enc B | VCC / GND |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **FL** | LEFT | #1 MLA + MLB | **12** → RC1 | **18** | **16** | LEFT strip |
| **RL** | LEFT | #1 MRA + MRB | **13** → RC2 | **19** | **17** | LEFT strip |
| **FR** | RIGHT | #2 MLA + MLB | **11** → RC1 | **2** | **4** | RIGHT strip |
| **RR** | RIGHT | #2 MRA + MRB | **10** → RC2 | **3** | **5** | RIGHT strip |

### Everything not on the Arduino

| Connection | From | To |
| :--- | :--- | :--- |
| Motor power ×4 | motor Red + White | driver channel terminal (see §1) |
| Encoder VCC ×4 | motor Blue | that side's `+` rail |
| Encoder GND ×4 | motor Black | that side's `−` rail |
| Driver signal GND ×2 | GND pin beside RC inputs (**not** VB−) | nearest strip's `−` rail |
| Rail bridge ×2 | LEFT strip `+` / `−` | RIGHT strip `+` / `−` |
| Driver power ×2 | 2S battery | driver VB+ / VB− |
| Converter | 3S battery (see §7 warning) | 19 V load |

---

## 6. Power distribution (breadboard rails)

Only **two** wires leave the Arduino's power header. Everything else
branches off the rails.

| Rail | Fed by | Distributes to |
| :--- | :--- | :--- |
| `+` | 1 jumper from Arduino **5V** | 4× encoder VCC (blue) |
| `−` | 1 jumper from Arduino **GND** | 2× driver signal GND + 4× encoder GND (black) |

- Driver **signal GND** = the GND pin beside the RC inputs, **not VB−**.
  VB− stays in the 2S battery loop only.
- Continuity-test each rail end-to-end before wiring — some rail strips are
  split into two disconnected halves at the midpoint, often unmarked.
- Breadboard stripe colour is the **opposite** of the motor loom convention
  (red stripe = positive, blue stripe = negative). Go by which jumper feeds
  the rail, never by colour-matching.
- This build uses two independent rail strips (one per side of the centre
  duct), bridged together with two jumpers (`+`↔`+`, `−`↔`−`) rather than one
  single breadboard. Feed point is the RIGHT strip (closest to the Arduino).

---

## 7. Firmware constants (measured)

| Constant | Value | Source |
| :--- | :--- | :--- |
| `TICKS_PER_REV` | 70.6 | `encoder_calibration.ino`, 10-turn average (FL 69.5, FR 70.2, RL 71.3, RR 71.2) |
| `WHEEL_DIAMETER_M` | 0.095 (measured 2026-07) | across the rollers, with calipers |
| `TICKS_PER_METER` | 236.6 | derived: `70.6 / (π × 0.095)` — recompute if diameter changes |
| `ENC_DIR_FL` | −1 | left side mirror-mounted |
| `ENC_DIR_FR` | +1 | |
| `ENC_DIR_RL` | −1 | left side mirror-mounted |
| `ENC_DIR_RR` | +1 | |
| `DIR_FL/FR/RL/RR` | all +1 | adjust if a wheel spins backward on `w` |
| `MAX_THROTTLE` | 0.85 | protects 6V motors on 7.4V (2S) pack |
| `NEUTRAL` / `DEFLECT` | 1500 / 435 µs | MDDRC10 spec |

The `ENC_DIR_*` signs describe motor mounting, not pin numbers — none of the
Rev.C–F pin changes affect them.

⚠️ **3S/converter note:** the DC-DC converter (152 W, in 10–36 V) sags close
to its floor if run from a 3S pack under load — a 3S pack can dip below 10 V
near the bottom of its discharge curve, before load is even considered.
Measure the pack under realistic load before relying on this combination; 4S
gives more headroom.

---

## 8. Checks before power-on

### Continuity / config

- [ ] Red–White on each motor reads a few ohms (winding) — confirms you
      have the right pair before connecting to a driver terminal.
- [ ] `+` and `−` rails each read ~5V once Arduino is powered (probe at the
      end furthest from the feed jumper, not just next to it).
- [ ] Both drivers' MIX/IND switch → **IND**.
- [ ] Arduino GND → both drivers' signal GND (not VB−).
- [ ] Both drivers have their own 2S power (VB+/VB−) connected and seated —
      a driver with **no LED lit at all** almost always means no power
      reaching that board, not a signal fault.

### Confirming the Ch.1/Ch.2 mapping against real hardware

The paperwork agrees; this proves the robot does too. Do it once per driver.

- [ ] Upload `mecanum_direction_control.ino`, drop speed to minimum with `-`.
- [ ] Press `e` (diagonal forward-right) — this drives **FL and RR only**.
      If a rear-left or front-right wheel moves instead, that driver's two
      channels are transposed: swap the Red/White pairs between MLA/MLB and
      MRA/MRB on that board.
- [ ] Press `q` (diagonal forward-left) — this drives **FR and RL only**.
      Same test, other diagonal. Between the two, all four wheels are
      identified unambiguously.
- [ ] Encoder check (`k`): all four wheels count **up** when rolled forward
      by hand, motors held off.

The diagonals are the right test here precisely because they activate two
wheels on **opposite corners** — a channel transposition can't hide behind a
symmetric result the way it can with `w` or `d`.

### If an ERR light appears on a driver that was previously fine

This happened during this build's Rev.E→F debugging, and it's worth a
specific note. Cytron MDDRC10 ERR LEDs blink in patterns that mean different
things (check the datasheet's blink-code table for the exact meaning) — but
in general, suspect, in order:

1. A signal wire jostled loose while working on the *other* driver, if the
   two share a ground rail or nearby connector.
2. A stall/overcurrent lockout from a motor that was stalled during testing
   (e.g. a swap test) — usually needs a power cycle to clear.
3. Genuinely invalid signal — check the RC jumper is seated in the header you
   think it is, on that specific board, not assumed from documentation.
