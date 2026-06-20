#!/usr/bin/env python3
"""
gps_config.py - one-shot NEO-8M (u-blox 8) configurator for the Smart Kayak
position-hold loop. Pushes a UBX profile over the USB-UART and verifies each
ACK, then saves to flash/BBR so it persists.

Profile (tuned for slow position-hold, Guadalajara ~1500 m):
  - 5 Hz nav rate (CFG-RATE)
  - Pedestrian dynamic model (CFG-NAV5)        # switch to Sea(5) for the kayak
  - Enable GST (per-axis position sigma, m)    # live accuracy estimate
  - SBAS on, ranging + diff-corrections (WAAS) # CFG-SBAS
  - NMEA trim: GGA/RMC/GST @5Hz, GSA/GSV @1Hz, GLL/VTG off
  - Save config to BBR + Flash (CFG-CFG)

Usage:  python tools/gps_config.py [COMx] [baud]
        defaults: COM7, tries 38400 then 9600.
"""
import sys, time, struct
import serial
import serial.tools.list_ports

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM7"
BAUDS = [int(sys.argv[2])] if len(sys.argv) > 2 else [38400, 9600]


def ubx(cls, mid, payload=b""):
    body = bytes([cls, mid]) + struct.pack("<H", len(payload)) + payload
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b"\xB5\x62" + body + bytes([ck_a, ck_b])


def wait_ack(ser, cls, mid, timeout=2.0):
    """Scan the incoming byte stream (NMEA + UBX) for ACK-ACK/NAK of cls,mid."""
    end = time.time() + timeout
    buf = bytearray()
    while time.time() < end:
        buf += ser.read(ser.in_waiting or 1)
        i = 0
        while i < len(buf) - 9:
            if buf[i] == 0xB5 and buf[i + 1] == 0x62 and buf[i + 2] == 0x05:
                mtype = buf[i + 3]              # 0x01 ACK, 0x00 NAK
                acls, amid = buf[i + 6], buf[i + 7]
                if acls == cls and amid == mid:
                    return mtype == 0x01
            i += 1
        buf = buf[-9:]
    return None  # no ack seen


def send(ser, name, cls, mid, payload):
    ser.reset_input_buffer()
    ser.write(ubx(cls, mid, payload))
    ser.flush()
    ack = wait_ack(ser, cls, mid)
    tag = "ACK" if ack else ("NAK" if ack is False else "no-reply")
    print(f"  {name:<20} -> {tag}")
    return ack


def detect(port):
    for baud in BAUDS:
        try:
            ser = serial.Serial()
            ser.port = port; ser.baudrate = baud; ser.timeout = 0.3
            # Keep DTR/RTS low so opening doesn't auto-reset an ESP32 bridge.
            ser.dtr = False; ser.rts = False
            ser.open()
        except Exception as e:
            print(f"!! cannot open {port}: {e}")
            sys.exit(1)
        time.sleep(1.5)            # allow an ESP32 bridge to boot if it reset
        data = ser.read(1200)
        if b"$" in data and (b"G" in data):
            print(f"Connected {port} @ {baud} (seeing NMEA).")
            return ser, baud
        ser.close()
    print(f"!! No NMEA on {port} at {BAUDS}. Check wiring/baud.")
    sys.exit(1)


def main():
    ser, baud = detect(PORT)

    # CFG-RATE: 5 Hz (measRate=200 ms, navRate=1, timeRef=1=GPS)
    send(ser, "rate 5Hz", 0x06, 0x08, struct.pack("<HHH", 200, 1, 1))

    # CFG-NAV5: apply dynModel only (mask=0x0001), dynModel=3 (pedestrian)
    nav5 = bytearray(36)
    struct.pack_into("<H", nav5, 0, 0x0001)   # mask: dyn only
    nav5[2] = 3                               # dynModel = pedestrian
    nav5[3] = 3                               # fixMode auto 2D/3D (not applied)
    send(ser, "nav5 pedestrian", 0x06, 0x24, bytes(nav5))

    # CFG-MSG (3-byte form -> applies to the port the cmd arrives on).
    # rate = output every N nav solutions (0 = off).
    for name, c, i, rate in [
        ("GGA @5Hz", 0xF0, 0x00, 1),
        ("RMC @5Hz", 0xF0, 0x04, 1),
        ("GST @5Hz", 0xF0, 0x07, 1),
        ("GSA @1Hz", 0xF0, 0x02, 5),
        ("GSV @1Hz", 0xF0, 0x03, 5),
        ("GLL off",  0xF0, 0x01, 0),
        ("VTG off",  0xF0, 0x05, 0),
    ]:
        send(ser, name, 0x06, 0x01, bytes([c, i, rate]))

    # CFG-SBAS: enabled, ranging + diff-corr, auto-scan all PRNs (WAAS)
    send(ser, "sbas WAAS", 0x06, 0x16,
         struct.pack("<BBBBI", 0x01, 0x03, 3, 0, 0x00000000))

    # CFG-PRT: switch UART1 to 38400 (5 Hz needs more than 9600 can carry).
    # 8N1 mode=0x08C0, in/out proto = UBX+NMEA. ACK comes back at the *new*
    # baud, so we don't wait for it here -- we reopen and confirm instead.
    TARGET_BAUD = 38400
    if baud != TARGET_BAUD:
        print(f"  switching UART1 -> {TARGET_BAUD} baud ...")
        prt = struct.pack("<BBHIIHHHH", 1, 0, 0, 0x000008C0, TARGET_BAUD,
                          0x0003, 0x0003, 0, 0)
        ser.write(ubx(0x06, 0x00, prt)); ser.flush()
        time.sleep(0.3); ser.close(); time.sleep(0.3)
        ser = serial.Serial(PORT, TARGET_BAUD, timeout=0.5)
        time.sleep(0.5)
        chk = ser.read(800)
        if b"$" in chk:
            print(f"  reconnected at {TARGET_BAUD} (NMEA confirmed).")
            baud = TARGET_BAUD
        else:
            print(f"  !! no NMEA at {TARGET_BAUD} - baud change may have failed; "
                  f"settings NOT saved. Re-run; module is still on {baud}.")
            ser.close(); sys.exit(1)

    # CFG-CFG: save current config to BBR + Flash + EEPROM (persist everything)
    send(ser, "save config", 0x06, 0x09,
         struct.pack("<III B", 0, 0x0000FFFF, 0, 0x07))

    print("\nSample stream after config (look for $G*GST and 5/s GGA):")
    ser.reset_input_buffer()
    t = time.time()
    seen = set()
    while time.time() - t < 3:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("$"):
            seen.add(line[1:6])
            if "GST" in line:
                print("  ", line)
    print("  sentence types seen:", sorted(seen))
    ser.close()


if __name__ == "__main__":
    main()
