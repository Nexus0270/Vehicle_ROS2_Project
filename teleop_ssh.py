#!/usr/bin/env python3
"""
SSH-friendly keyboard teleop for mecanum_direction_control.ino

RUN THIS ON THE SBC, over SSH from your laptop.

WHY A SECOND SCRIPT
  teleop_keyboard.py uses pynput, which hooks the graphical display server.
  Over SSH there is no display, so it cannot see your keystrokes. This
  version reads the terminal directly (termios raw mode), which works fine
  through an SSH session.

HOW IT BEHAVES
  Terminals cannot report key RELEASE, so this uses latched control:
  tap a direction and it keeps going, tap SPACE or 's' to stop.
  The script re-sends the active command at 20 Hz, which keeps the
  firmware's 200 ms watchdog fed. If this script dies or the SSH link
  drops, the stream stops and the robot halts within 200 ms.

SETUP ON THE SBC
    sudo apt install python3-serial
    sudo usermod -aG dialout $USER      # then log out and back in
    python3 teleop_ssh.py

CONTROLS
  w forward    b backward    a strafe-left   d strafe-right
  q diag F-L   e diag F-R    z diag B-L      c diag B-R
  j turn-left  l turn-right
  SPACE or s   stop
  + / -        speed        t telemetry     r reset ticks
  k encoder-check           y hold/latch    h help
  Ctrl-C       quit (sends stop first)
"""

import sys
import time
import select
import termios
import tty
import threading

try:
    import serial
except ImportError:
    sys.exit("Missing pyserial.  Run:  sudo apt install python3-serial")

# ---------------------------------------------------------------- settings
# /dev/ttyACM0 is typical for a Mega on Linux. If you set up the udev rule
# from the project plan, use "/dev/arduino" instead so it never shifts.
PORT      = "/dev/ttyACM0"
BAUD      = 115200
REPEAT_HZ = 20            # must beat the firmware's 200 ms timeout

MOTION_KEYS = set("wbadqezcjl")
TAP_KEYS    = set("+-trkyh?")
STOP_KEYS   = {" ", "s"}

active  = None            # currently latched motion key
running = True


def serial_reader(ser):
    """Echo the Arduino's output. \r\n because the terminal is in raw mode."""
    while running:
        try:
            line = ser.readline().decode(errors="replace").strip()
        except serial.SerialException:
            break
        if line:
            sys.stdout.write(line + "\r\n")
            sys.stdout.flush()


def repeater(ser):
    """Re-send the latched command so the firmware watchdog stays fed."""
    period = 1.0 / REPEAT_HZ
    while running:
        if active:
            try:
                ser.write(active.encode())
            except serial.SerialException:
                break
        time.sleep(period)


def main():
    global active, running

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"Could not open {PORT}: {e}\n"
                 f"Check the port exists (ls /dev/ttyACM*) and that you are "
                 f"in the dialout group.")

    time.sleep(2.0)               # the Arduino resets when the port opens
    ser.reset_input_buffer()

    print(__doc__)
    print(f"Connected on {PORT}. Tap a direction, SPACE stops, Ctrl-C quits.\n")

    threading.Thread(target=serial_reader, args=(ser,), daemon=True).start()
    threading.Thread(target=repeater,      args=(ser,), daemon=True).start()

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        while True:
            # poll stdin without blocking, so the repeater keeps running
            if select.select([sys.stdin], [], [], 0.05)[0]:
                ch = sys.stdin.read(1)

                if ch == "\x03":                     # Ctrl-C
                    break
                elif ch in STOP_KEYS:
                    active = None
                    ser.write(b"s")
                elif ch in MOTION_KEYS:
                    active = ch
                    ser.write(ch.encode())
                elif ch in TAP_KEYS:
                    ser.write(ch.encode())

    except Exception as e:
        sys.stdout.write(f"error: {e}\r\n")
    finally:
        running = False
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        time.sleep(0.2)
        try:
            ser.write(b"s")       # always leave it stopped
            ser.close()
        except Exception:
            pass
        print("\nstopped, port closed")


if __name__ == "__main__":
    main()
