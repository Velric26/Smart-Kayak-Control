#!/usr/bin/env python3
"""
bt_console.py - live telemetry + tuning console for the Smart Kayak over the
Bluetooth SPP serial port (Windows pairs SmartKayak-* as a COM port; the rover
reads on COM5 here). A Tera-Term replacement with two helper workflows built in.

Usage:
  python tools/bt_console.py [COMx]            # interactive console (default COM5)
  python tools/bt_console.py COM5 --send "kp 0.01"   # fire one command, stream 6 s
  python tools/bt_console.py COM5 --secs 10 --send "cal max"

Interactive: type any rover command + Enter (kp/kd/db, mxl/mxr, runl/runr,
hoff, cal max, cal compass, ...). Telemetry streams in the background.
Local meta-commands (start with '/'):
  /max          shortcut for 'cal max' (motor straight-line trim run)
  /align        toggle hdg-vs-cog capture; prints the mean offset + a
                suggested 'hoff <deg>' (drive STRAIGHT while it samples)
  /clear        reset the /align samples
  /q            quit

------------------------------------------------------------------------------
MOTOR STRAIGHT-LINE TRIM  (keep the rover tracking straight at full)
  1. Clear space; rover on the ground, on battery.
  2. /max  -> both motors ramp to full and hold 2 s. Watch which way it veers.
  3. If it pulls LEFT, the right side is stronger: lower 'mxr' a touch
     (e.g. mxr 0.58) or raise 'mxl'. Pulls RIGHT -> opposite.
  4. /max again; repeat until it runs straight. Then the values are live;
     tell Claude and they get baked into config.h (MOTOR_MAX_L/R).

HEADING-vs-COURSE ALIGNMENT  (set hoff so ANCHOR aims true)
  1. Outdoors, GPS fix (acc a few m). /align to start capturing.
  2. Drive STRAIGHT forward in MANUAL several metres (cog only valid moving).
  3. Read the suggested 'hoff' it prints; do it at 2-3 headings to confirm
     it's a constant offset. Then type e.g. 'hoff 180' (saved to NVS).
------------------------------------------------------------------------------
"""
import sys, time, threading, re, math
import serial

PORT = "COM5"
SECS = 6
SEND = None
args = sys.argv[1:]
i = 0
while i < len(args):
    a = args[i]
    if a == "--send": SEND = args[i + 1]; i += 2
    elif a == "--secs": SECS = float(args[i + 1]); i += 2
    elif not a.startswith("--"): PORT = a; i += 1
    else: i += 1

HDG = re.compile(r"hdg=\s*([-\d.]+)")
COG = re.compile(r"cog=\s*([-\d.]+)")

_align = {"on": False, "samples": []}
_io_lock = threading.Lock()


def wrap180(d):
    while d > 180: d -= 360
    while d <= -180: d += 360
    return d


def circ_mean(diffs):
    s = sum(math.sin(math.radians(x)) for x in diffs)
    c = sum(math.cos(math.radians(x)) for x in diffs)
    return math.degrees(math.atan2(s, c))


def on_line(line):
    print(line)
    if _align["on"]:
        h, c = HDG.search(line), COG.search(line)
        if h and c:                      # cog only present while moving
            off = wrap180(float(c.group(1)) - float(h.group(1)))
            _align["samples"].append(off)
            if len(_align["samples"]) % 3 == 0:
                m = circ_mean(_align["samples"])
                print(f"   [align] n={len(_align['samples'])} "
                      f"mean(cog-hdg)={m:+.1f}  -> try 'hoff {round(m)}'")


def reader(ser, stop):
    buf = b""
    while not stop.is_set():
        try:
            buf += ser.read(ser.in_waiting or 1)
        except Exception:
            break
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            txt = ln.decode(errors="replace").strip()
            if txt:
                with _io_lock:
                    on_line(txt)


def send(ser, cmd):
    with _io_lock:
        print(f">>> {cmd}")
    ser.write((cmd + "\r\n").encode())
    ser.flush()


def main():
    try:
        ser = serial.Serial(PORT, 115200, timeout=0.2)
    except Exception as e:
        print(f"!! cannot open {PORT}: {e}\n   (close Tera Term / pio monitor first; "
              f"check the rover is powered and paired)")
        sys.exit(1)
    print(f"Connected {PORT}. Ctrl-C or /q to quit.")
    stop = threading.Event()
    t = threading.Thread(target=reader, args=(ser, stop), daemon=True)
    t.start()

    if SEND is not None:
        time.sleep(0.3); send(ser, SEND); time.sleep(SECS)
        stop.set(); ser.close(); return

    try:
        for raw in sys.stdin:
            cmd = raw.strip()
            if not cmd: continue
            if cmd in ("/q", "/quit"): break
            elif cmd == "/max": send(ser, "cal max")
            elif cmd == "/align":
                _align["on"] = not _align["on"]
                print(f"   [align] {'ON - drive straight' if _align['on'] else 'OFF'}")
            elif cmd == "/clear":
                _align["samples"].clear(); print("   [align] samples cleared")
            else: send(ser, cmd)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set(); ser.close()


if __name__ == "__main__":
    main()
