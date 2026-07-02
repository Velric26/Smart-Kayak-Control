#!/usr/bin/env python3
"""
telemetry_bridge.py - serves the SmartKayak web dashboard and feeds it
telemetry over WebSocket. Two sources, same wire format (the firmware's
telemetry lines + console commands), so the frontend never changes:

  SIM mode (no hardware needed - frontend development / demo):
      python tools/telemetry_bridge.py --sim
  SERIAL mode (bridge the Bluetooth SPP COM port <-> WebSocket):
      python tools/telemetry_bridge.py COM5

Then open http://localhost:8000  (dashboard + ws://.../ws on the same port).

Later, the ESP32 itself can serve the same line stream over WiFi/WebSocket
and this bridge simply goes away - the dashboard is transport-agnostic.

Sim-only extra command: "mode manual|hh|anchor|disarm" switches the fake
rover's mode (on real hardware modes come from the RC transmitter).
"""
import argparse, asyncio, functools, random, sys, threading, time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import websockets

DASH_DIR = Path(__file__).parent / "dashboard"

clients: set = set()


async def broadcast(line: str):
    dead = []
    for ws in clients:
        try:
            await ws.send(line)
        except Exception:
            dead.append(ws)
    for ws in dead:
        clients.discard(ws)


# ---------------------------------------------------------------- simulator
class Sim:
    """Fake rover emitting firmware-format telemetry; answers console cmds."""

    def __init__(self):
        self.mode = "DISARMED"
        self.hdg = 137.0
        self.sp = 137.0
        self.l = self.r = 0.0
        self.rc_l = self.rc_r = 0.0
        self.sats, self.hdop, self.acc = 9, 1.1, 1.6
        self.drop = 0
        self.log_hz = 5.0
        self.anc_d, self.anc_b = 0.0, 0.0
        # full tunable set, defaults mirroring config.h (mule gain set)
        self.vals = dict(kp=0.0006, kd=0.0007, db=4.0, amp=0.45,
                         kickl=0.40, kickr=0.42, runl=0.20, runr=0.22, kickms=50,
                         mxl=0.60, mxr=0.62, slew=0.5, slewdn=20.0,
                         hoff=0.0, fuse=0.98, ancdb=1.0, ancacc=3.0,
                         pkp=0.30, pkd=0.0)

    def echo_all(self) -> str:
        """Firmware-identical value echo (same keys/format as echoAllValues())."""
        v = self.vals
        return (f">> kp={v['kp']:.4f} kd={v['kd']:.4f} db={v['db']:.1f} amp={v['amp']:.2f}"
                f"  kickL={v['kickl']:.2f} kickR={v['kickr']:.2f}"
                f" runL={v['runl']:.2f} runR={v['runr']:.2f} kickMs={int(v['kickms'])}"
                f"  maxL={v['mxl']:.2f} maxR={v['mxr']:.2f}"
                f" slew={v['slew']:.1f} slewdn={v['slewdn']:.1f}"
                f"  hoff={v['hoff']:.0f} fuse={v['fuse']:.3f}"
                f" ancdb={v['ancdb']:.1f} ancacc={v['ancacc']:.1f}"
                f" pkp={v['pkp']:.3f} pkd={v['pkd']:.3f}  log={self.log_hz:.0f}Hz")

    def tick(self, dt: float):
        # slow GPS quality wander
        self.acc = max(0.8, min(6.0, self.acc + random.uniform(-.05, .05)))
        self.hdop = max(0.7, min(2.5, self.hdop + random.uniform(-.02, .02)))
        # raw sticks: drive the rover in MANUAL, near-neutral chatter otherwise
        if self.mode == "MANUAL":
            self.rc_l = max(-1, min(1, self.rc_l + random.uniform(-.08, .1)))
            self.rc_r = max(-1, min(1, self.rc_r + random.uniform(-.08, .1)))
        else:
            self.rc_l = random.uniform(-.03, .03)
            self.rc_r = random.uniform(-.03, .03)
        if self.mode == "MANUAL":
            self.l, self.r = self.rc_l, self.rc_r
            self.hdg = (self.hdg + (self.r - self.l) * 40 * dt) % 360
        elif self.mode == "HEADING_HOLD":
            err = (self.sp - self.hdg + 540) % 360 - 180
            w = max(-.6, min(.6, .04 * err))
            self.l, self.r = -w * .8, w * .8
            self.hdg = (self.hdg + w * 60 * dt + random.uniform(-.6, .6)) % 360
        elif self.mode.startswith("ANCHOR"):
            self.anc_d = max(0.0, self.anc_d + random.uniform(-.25, .2))
            self.anc_b = (self.anc_b + random.uniform(-2, 2)) % 360
            chase = self.anc_d > 1.0
            self.l = self.r = 0.45 if chase else 0.0
            self.hdg = (self.hdg + random.uniform(-1, 1)) % 360
        else:
            self.l = self.r = 0.0
            self.hdg = (self.hdg + random.uniform(-.3, .3)) % 360

    def shaped(self, v: float, side: str) -> float:
        """Model the firmware drive shaping: 0 stays 0, else the command is
        remapped into [RUN floor, MAX cap] using the live-tuned values."""
        if abs(v) < 0.02:
            return 0.0
        lo = self.vals["runl" if side == "l" else "runr"]
        hi = self.vals["mxl" if side == "l" else "mxr"]
        out = lo + (hi - lo) * abs(v)
        return out if v > 0 else -out

    def line(self) -> str:
        moving = abs(self.l) + abs(self.r) > .1
        cog = f" cog={self.hdg + random.uniform(-4, 4):.0f}" if moving else ""
        anc = f"  anc={self.anc_d:.1f}m@{self.anc_b:.0f}" if self.mode.startswith("ANCHOR") else ""
        return (f"[{self.mode:<14}] "
                f"rc={self.rc_l:+.2f}/{self.rc_r:+.2f} "
                f"L={self.l:+.2f}>{self.shaped(self.l, 'l'):+.2f} "
                f"R={self.r:+.2f}>{self.shaped(self.r, 'r'):+.2f}  "
                f"hdg={self.hdg:5.1f}{cog} sp={self.sp:5.1f}  "
                f"gps={self.sats}s/{self.hdop:.1f} FIX acc={self.acc:.1f}{anc}  "
                f"link=OK  drop={self.drop}")

    def command(self, cmd: str) -> str:
        p = cmd.strip().split()
        if not p:
            return ""
        name = p[0].lower()
        if name == "mode" and len(p) > 1:          # sim-only helper
            m = {"manual": "MANUAL", "hh": "HEADING_HOLD",
                 "anchor": "ANCHOR", "disarm": "DISARMED"}.get(p[1].lower())
            if m:
                self.mode = m
                if m == "HEADING_HOLD":
                    self.sp = self.hdg
                if m == "ANCHOR":
                    self.anc_d, self.anc_b = 6.0, random.uniform(0, 360)
                return f">> [sim] mode -> {m}"
            return ">> [sim] usage: mode manual|hh|anchor|disarm"
        if name == "show":
            return self.echo_all()
        if name == "log" and len(p) > 1:
            self.log_hz = float(p[1])
            return self.echo_all()
        if name == "stop":
            self.l = self.r = 0
            return ">> [sim] stopped"
        if name in ("cal", "tune", "clrgains"):
            return f">> [sim] '{cmd}' acknowledged (routine not simulated)"
        if name in ("mnl", "mnr"):                 # firmware aliases
            name = "runl" if name == "mnl" else "runr"
        if name in self.vals and len(p) > 1:
            self.vals[name] = float(p[1])
            return self.echo_all()
        return f">> [sim] unknown cmd: {cmd}"


async def run_sim():
    sim = Sim()

    async def handler(ws):
        clients.add(ws)
        try:
            async for msg in ws:
                reply = sim.command(str(msg))
                if reply:
                    await broadcast(reply)
        finally:
            clients.discard(ws)

    async def pump():
        last = time.monotonic()
        while True:
            dt = time.monotonic() - last
            last = time.monotonic()
            sim.tick(dt)
            if sim.log_hz > 0:
                await broadcast(sim.line())
            await asyncio.sleep(1 / sim.log_hz if sim.log_hz > 0 else 0.2)

    return handler, pump()


# ---------------------------------------------------------------- serial bridge
async def run_serial(port: str, baud: int):
    import serial  # pyserial
    ser = serial.Serial(port, baud, timeout=0)
    print(f"serial bridge on {port} @ {baud}")

    async def handler(ws):
        clients.add(ws)
        try:
            async for msg in ws:
                ser.write((str(msg).strip() + "\r\n").encode())
        finally:
            clients.discard(ws)

    async def pump():
        buf = b""
        while True:
            buf += ser.read(4096)
            while b"\n" in buf:
                ln, buf = buf.split(b"\n", 1)
                txt = ln.decode(errors="replace").strip()
                if txt:
                    await broadcast(txt)
            await asyncio.sleep(0.02)

    return handler, pump()


# ---------------------------------------------------------------- http + main
def serve_dashboard(port: int):
    """Static HTTP for the dashboard (stdlib; handles HEAD probes etc.).
    The WebSocket lives on port+1 - same convention the ESP32 will use
    later (HTTP :80 / WS :81), so the frontend logic carries over."""
    handler = functools.partial(SimpleHTTPRequestHandler, directory=str(DASH_DIR))
    handler.log_message = lambda *a, **k: None          # quiet
    httpd = ThreadingHTTPServer(("0.0.0.0", port), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port_or_sim", nargs="?", default="--sim",
                    help="COMx for serial bridge, or --sim")
    ap.add_argument("--sim", action="store_true", help="simulated rover (no hardware)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--http", type=int, default=8000,
                    help="HTTP port (WebSocket = this + 1)")
    a = ap.parse_args()

    if a.sim or a.port_or_sim == "--sim":
        handler, pump = await run_sim()
        print("SIM mode (fake rover). Sim-only cmd: mode manual|hh|anchor|disarm")
    else:
        handler, pump = await run_serial(a.port_or_sim, a.baud)

    serve_dashboard(a.http)
    async with websockets.serve(handler, "0.0.0.0", a.http + 1):
        print(f"dashboard: http://localhost:{a.http}   (ws on :{a.http + 1})")
        await pump


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.exit(0)
