#!/usr/bin/env python3
"""
Hold-to-move keyboard teleop for mecanum_direction_control.ino

WHY THIS EXISTS
  Serial has no "key released" event, and the Arduino Serial Monitor is
  line-buffered (nothing is sent until you press Enter). So the firmware
  infers "key released" from silence: this script repeats the motion
  character at REPEAT_HZ while the key is physically held, and the moment
  you let go the stream stops and the Arduino's watchdog halts the robot
  after MOTION_TIMEOUT_MS (200 ms).

SETUP
  pip install pyserial pynput
  Set PORT below to your Arduino's port, then run:
      python teleop_keyboard.py

CONTROLS (hold to move)
  w forward    b backward    a strafe-left   d strafe-right
  q diag F-L   e diag F-R    z diag B-L      c diag B-R
  j turn-left  l turn-right
TAPPED (not held)
  + / -  speed      t telemetry     r reset ticks
  k encoder-check   y hold/latch    h help
  ESC or Ctrl-C to quit
"""

import sys
import time
import threading

try:
    import serial
except ImportError:
    sys.exit("Missing pyserial.  Run:  pip install pyserial")

try:
    from pynput import keyboard
except ImportError:
    sys.exit("Missing pynput.  Run:  pip install pynput")

# ---------------------------------------------------------------- settings
PORT      = "COM3"      # <-- change to your Arduino port (e.g. /dev/ttyACM0)
BAUD      = 115200
REPEAT_HZ = 20          # must be faster than the firmware's 200 ms timeout

# Keys that mean "keep moving while held"
HOLD_KEYS = set("wbadqezcjl")
# Keys that are one-shot commands
TAP_KEYS  = set("+-trkyh?s")

held_key = None
lock     = threading.Lock()
running  = True


def on_press(key):
    """Latch a motion key, or fire a one-shot command."""
    global held_key
    try:
        ch = key.char.lower()
    except AttributeError:
        return

    if ch in HOLD_KEYS:
        with lock:
            held_key = ch
    elif ch in TAP_KEYS:
        try:
            ser.write(ch.encode())
        except serial.SerialException:
            pass


def on_release(key):
    """Releasing a motion key stops the repeat stream."""
    global held_key, running

    if key == keyboard.Key.esc:
        running = False
        return False

    try:
        ch = key.char.lower()
    except AttributeError:
        return

    if ch in HOLD_KEYS:
        with lock:
            if held_key == ch:
                held_key = None
        # Send an explicit stop too, so we do not wait out the watchdog
        try:
            ser.write(b"s")
        except serial.SerialException:
            pass


def repeater():
    """Stream the held key so the firmware watchdog stays fed."""
    period = 1.0 / REPEAT_HZ
    while running:
        with lock:
            ch = held_key
        if ch:
            try:
                ser.write(ch.encode())
            except serial.SerialException:
                break
        time.sleep(period)


def reader():
    """Echo whatever the Arduino sends back."""
    while running:
        try:
            line = ser.readline().decode(errors="replace").strip()
        except serial.SerialException:
            break
        if line:
            print(line)


if __name__ == "__main__":
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"Could not open {PORT}: {e}\n"
                 f"Close the Arduino Serial Monitor first -- only one program "
                 f"can hold the port.")

    time.sleep(2.0)          # Arduino resets when the port opens
    ser.reset_input_buffer()

    print(__doc__)
    print(f"Connected on {PORT}. Hold a key to move, release to stop. ESC quits.\n")

    threading.Thread(target=repeater, daemon=True).start()
    threading.Thread(target=reader,   daemon=True).start()

    try:
        with keyboard.Listener(on_press=on_press, on_release=on_release) as l:
            l.join()
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        time.sleep(0.2)
        try:
            ser.write(b"s")          # make sure it stops on exit
            ser.close()
        except Exception:
            pass
        print("\nstopped, port closed")
