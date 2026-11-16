#!/usr/bin/env python3
"""Real gripper over Wi-Fi: ESP32 bridge (GripperBridge, 192.168.88.169) ->
BusLinker -> Hiwonder bus servo ID 1. Same HTTP API as hungros2's
gripper-wifi-ota/control/gripper_test.py, without its interactive shell.

Calibration (2026-11-01): pos 1000 = open (88 mm between the jaws), pos 0 =
servo end of travel with the jaws still ~30 mm apart. (The first calib saved
these the other way round: "open" closed the jaws onto the object and the
user hit the emergency stop.)
Only objects wider than ~30 mm can be held; GraspNet grasps narrower than
40 mm are dropped by grasp_once.py.

Password: ~/.ros/fr3_gripper_password (copy of hungros2 ota-password.txt) or GRIPPER_PASSWORD.

Usage: gripper.py status | open | grip [squeeze_mm] | close | free
"""
import base64
import json
import os
import struct
import sys
import time
import urllib.parse
import urllib.request

HOST = os.environ.get("GRIPPER_HOST", "192.168.88.169")
OPEN_POS, CLOSE_POS = 1000, 0
OPEN_MM, CLOSED_MM = 88.0, 30.0
MAX_TEMP_C = 60          # firmware and gripper_test.py refuse to move at or above this
HOLD_PAST_CONTACT = 15   # servo units past the stall point (grip_to)
SQUEEZE_MM = 8.0         # grip(): target this far closed past the contact point (~140 units): the grip force


def width_mm(pos):
    """Approximate gap between the jaws (linear between the two calibrated ends)."""
    return OPEN_MM + (CLOSED_MM - OPEN_MM) * (pos - OPEN_POS) / (CLOSE_POS - OPEN_POS)


def pos_for_width(mm):
    """Inverse of width_mm (same linear assumption), clamped to the servo range."""
    p = OPEN_POS + (mm - OPEN_MM) * (CLOSE_POS - OPEN_POS) / (CLOSED_MM - OPEN_MM)
    return int(round(max(0, min(1000, p))))


class Gripper:
    def __init__(self, host=HOST):
        self.host = host
        pw = os.environ.get("GRIPPER_PASSWORD") or open(os.path.expanduser("~/.ros/fr3_gripper_password")).read().strip()
        self.auth = "Basic " + base64.b64encode(f"gripper:{pw}".encode()).decode()
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def _packet(self, cmd, params=b"", read=False):
        data = urllib.parse.urlencode(dict(id=1, cmd=cmd, params=bytes(params).hex(), read=int(read))).encode()
        req = urllib.request.Request(f"http://{self.host}/servo/packet", data=data, headers={"Authorization": self.auth})
        # 2026-11-01: Wi-Fi to the ESP32 spikes to seconds (ping 10..760 ms, some requests > 4 s);
        # writes are retried too: torque on/off and move-to-position give the same result if repeated
        for attempt in range(3):
            try:
                with self.opener.open(req, timeout=6) as r:
                    out = json.load(r)
                return bytes.fromhex(out["params"]) if read and out.get("ok") else out
            except OSError:
                if attempt == 2:
                    raise
                time.sleep(0.3)

    def status(self):
        for attempt in range(3):
            try:
                with self.opener.open(f"http://{self.host}/servo/status", timeout=6) as r:
                    return json.load(r)
            except OSError:
                if attempt == 2:
                    raise
                time.sleep(0.3)

    def pos(self):
        r = self._packet(28, read=True)
        return struct.unpack("<h", r[:2])[0] if r and len(r) >= 2 else None

    def temp(self):
        r = self._packet(26, read=True)
        return r[0] if r else None

    def _ready(self):
        """Servo mode, torque on, cool enough; raises otherwise (no motion sent)."""
        t = self.temp()
        if t is None or t >= MAX_TEMP_C:
            raise RuntimeError(f"gripper: nhiet do servo {t} C (>= {MAX_TEMP_C} C hoac khong doc duoc), khong chay")
        mode = self._packet(30, read=True)
        if not mode or mode[0] != 0:
            self._packet(29, struct.pack("<BBh", 0, 0, 0))
            time.sleep(0.05)
        self._packet(31, b"\x01")
        time.sleep(0.05)

    def move(self, pos, ms=800, timeout=None):
        """Move to pos (0..1000) and wait until it arrives or stops moving; returns the final position."""
        pos = int(max(0, min(1000, pos)))
        self._ready()
        self._packet(1, struct.pack("<HH", pos, ms))
        return self._wait(pos, (timeout or ms / 1000 + 1.5))

    def _wait(self, target, timeout):
        t_end = time.monotonic() + timeout
        last, still, p = None, 0, None
        while time.monotonic() < t_end:
            time.sleep(0.05)
            p = self.pos()
            if p is None:
                continue
            if abs(p - target) <= 3:
                break
            still = still + 1 if last is not None and abs(p - last) <= 1 else 0
            if still >= 6:          # stalled on something
                break
            last = p
        return p

    def open(self, ms=800):
        return self.move(OPEN_POS, ms)

    def grip(self, ms=2000, squeeze_mm=None):
        """Close slowly towards fully closed; stop where the jaws stall on the object, then
        command squeeze_mm further closed. The servo is position controlled: its force grows
        with how far the target lies past where the jaws are held, so the squeeze IS the grip
        force (2026-11-01: closing to the measured width - 5 mm never even touched the stand
        or the tray -- the width or the mm/pos mapping is a few mm off -- and both slipped).
        Returns (contact_pos, held): held is False when the jaws closed all the way."""
        squeeze_mm = SQUEEZE_MM if squeeze_mm is None else squeeze_mm
        contact = self.move(CLOSE_POS, ms)
        if contact is None or abs(contact - CLOSE_POS) <= 3:
            return contact, False
        units = squeeze_mm * 1000 / (OPEN_MM - CLOSED_MM)
        step = units if CLOSE_POS > OPEN_POS else -units
        self._packet(1, struct.pack("<HH", int(max(0, min(1000, contact + step))), 200))
        return contact, True

    def grip_to(self, object_mm, squeeze_mm=5.0, ms=1500):
        """Close slowly towards object_mm - squeeze_mm (measured object width), not all the way.
        Returns (pos, held): held when the jaws stalled on the object before that target;
        pos past the target means the object is narrower than measured or was missed."""
        target = pos_for_width(max(CLOSED_MM, object_mm - squeeze_mm))
        p = self.move(target, ms)
        if p is None or abs(p - target) <= 3:
            return p, False
        step = HOLD_PAST_CONTACT if CLOSE_POS > OPEN_POS else -HOLD_PAST_CONTACT
        hold = p + step
        hold = min(hold, target) if CLOSE_POS > OPEN_POS else max(hold, target)   # never past the target
        self._packet(1, struct.pack("<HH", int(max(0, min(1000, hold))), 100))
        return p, True

    def grip_measured(self, object_mm, extra_deg=5.0, ms=1500):
        """User's rule (2026-11-01): from the camera width, compute the servo angle where the
        jaws touch the object, then go extra_deg further closed (e.g. touch at 35 deg -> 30 deg).
        1 deg = 1000/240 units. If the jaws get there without touching (the camera width or the
        linear mm<->pos mapping is off), keep closing until they stall and then add extra_deg.
        Returns (pos, held, touch_pos): touch_pos = predicted touching position."""
        units = extra_deg * 1000 / 240
        sign = 1 if CLOSE_POS > OPEN_POS else -1
        touch = pos_for_width(object_mm)
        target = int(max(0, min(1000, touch + sign * units)))
        p = self.move(target, ms)
        if p is not None and abs(p - target) > 3:          # stalled before the target: pressing on the object
            return p, True, touch
        p = self.move(CLOSE_POS, ms)                       # not touching yet: close until it stalls
        if p is None or abs(p - CLOSE_POS) <= 3:
            return p, False, touch
        if abs(p - CLOSE_POS) > 15:   # pressing near the end stop only heats the servo (79 C, 2026-11-02)
            self._packet(1, struct.pack("<HH", int(max(0, min(1000, p + sign * units))), 200))
        return p, True, touch

    def free(self):
        self._packet(31, b"\x00")


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    g = Gripper()
    if cmd == "status":
        s = g.status()
        print(f"pos {s['position']} (~{width_mm(s['position']):.0f} mm)  {s['voltage_mv'] / 1000:.2f} V  "
              f"{s['temperature_c']} C  torque {'ON' if s['torque'] == 1 else 'OFF'}")
    elif cmd == "open":
        p = g.open()
        print(f"mo: pos {p} (~{width_mm(p):.0f} mm)")
    elif cmd == "close":
        p = g.move(CLOSE_POS)
        print(f"dong: pos {p} (~{width_mm(p):.0f} mm)")
    elif cmd == "grip":
        p, held = g.grip(squeeze_mm=float(sys.argv[2]) if len(sys.argv) > 2 else None)
        print(f"kep vat tai pos {p} (~{width_mm(p):.0f} mm)" if held else
              f"dong het (pos {p}): khong co vat hoac vat < {CLOSED_MM:.0f} mm")
    elif cmd == "free":
        g.free()
        print("nha torque")
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
