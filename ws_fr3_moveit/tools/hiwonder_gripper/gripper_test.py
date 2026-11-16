#!/usr/bin/env python3
"""Interactive test tool for the FR3 custom gripper driven by one Hiwonder
35 kg bus servo through a BusLinker v3.0 (PC test now, ESP32 later).

  python3 gripper_test.py                 # interactive shell
  python3 gripper_test.py --port /dev/ttyACM0 --id 3

Gripper mapping matches the MoveIt model: joint_clamp_1 goes from -0.001 m
(group_state "open") to -0.03 m ("closed"). open_pos / close_pos are the servo
units (0..1000) at those two jaw positions; set them with `calib`.
"""
import argparse
import json
import os
import time

import serial

from hiwonder_bus_servo import BROADCAST_ID, BusServo

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gripper_config.json")
DEFAULTS = {
    "port": "auto",     # tu tim cong USB co servo tra loi
    "id": 3,
    "open_pos": None,   # servo units, set by `calib`
    "close_pos": None,
    "move_ms": 800,
    "joint_open": -0.001,   # m, SRDF group_state "open"
    "joint_closed": -0.03,  # m, SRDF group_state "closed"
}

HELP = """
Lenh (motion/status):
  scan                  tim servo tren bus (ID 0..253)
  s | status            vi tri, dien ap, nhiet do, torque
  m <pos> [ms]          quay toi vi tri 0..1000 (0..240 deg)
  a <deg> [ms]          quay toi goc 0..240 deg
  o | open              mo gripper
  c | close             dong gripper (toi close_pos)
  g | grip [ms]         dong tu tu, dung khi bi ket (dang kep vat)
  j <met>               dat joint_clamp_1 (-0.001 mo .. -0.03 dong)
  stop                  dung ngay
  free / hold           tha torque (xoay tay duoc) / giu torque
  w | watch             in vi tri lien tuc (Ctrl+C de dung)
  motor <speed>         quay lien tuc -1000..1000, 'motor off' ve servo mode
Cau hinh:
  calib                 tha torque, vang tay ghi vi tri mo/dong
  setid <new>           doi ID servo (chi cam 1 servo!)
  limit <lo> <hi>       gioi han goc trong servo (0..1000)
  q | quit
"""


def load_config():
    cfg = dict(DEFAULTS)
    if os.path.exists(CONFIG):
        with open(CONFIG) as f:
            cfg.update(json.load(f))
    return cfg


def save_config(cfg):
    saved = {k: v for k, v in cfg.items() if k != "port"}  # port changes per plug-in
    with open(CONFIG, "w") as f:
        json.dump(saved, f, indent=2)
    print(f"Da luu {CONFIG}")


def joint_to_pos(cfg, joint):
    lo, hi = cfg["joint_closed"], cfg["joint_open"]
    joint = max(lo, min(hi, joint))
    t = (joint - cfg["joint_open"]) / (cfg["joint_closed"] - cfg["joint_open"])
    return round(cfg["open_pos"] + t * (cfg["close_pos"] - cfg["open_pos"]))


def pos_to_joint(cfg, pos):
    span = cfg["close_pos"] - cfg["open_pos"]
    t = (pos - cfg["open_pos"]) / span if span else 0.0
    return cfg["joint_open"] + t * (cfg["joint_closed"] - cfg["joint_open"])


def status(bus, cfg):
    sid = cfg["id"]
    pos = bus.read_pos(sid)
    if pos is None:
        print(f"Servo ID {sid} khong tra loi (kiem tra day, nguon, ID).")
        return
    vin = bus.read_vin(sid)
    joint = f"{pos_to_joint(cfg, pos):+.4f} m" if cfg["close_pos"] is not None else "(chua calib)"
    print(f"ID {sid}: pos {pos} ({pos * 0.24:.1f} deg)  joint {joint}  "
          f"Vin {vin:.2f} V  temp {bus.read_temp(sid)} C  torque {'ON' if bus.read_load(sid) else 'OFF'}")
    if vin is not None and vin < 9.0:
        print("  ! Vin < 9 V: servo 35 kg can nguon 9-12.6 V vao cong DC cua BusLinker, yeu va de ket.")


def wait_move(bus, sid, target, timeout_s):
    """Print position until the servo reaches target or stops moving."""
    t_end = time.monotonic() + timeout_s
    last, still = None, 0
    while time.monotonic() < t_end:
        time.sleep(0.05)
        pos = bus.read_pos(sid)
        if pos is None:
            continue
        print(f"\r  pos {pos:4d}", end="", flush=True)
        if abs(pos - target) <= 3:
            break
        still = still + 1 if last is not None and abs(pos - last) <= 1 else 0
        if still >= 6:
            print("  (dung lai truoc dich - bi can / thieu luc)", end="")
            break
        last = pos
    print()
    return bus.read_pos(sid)


def move(bus, cfg, pos, ms=None):
    ms = cfg["move_ms"] if ms is None else ms
    bus.move(cfg["id"], pos, ms)
    wait_move(bus, cfg["id"], pos, ms / 1000 + 1.5)


def grip(bus, cfg, ms=1500):
    """Close slowly; when the jaws stall on an object, hold slightly past the
    contact point so the servo keeps squeezing without full stall current."""
    sid = cfg["id"]
    direction = 1 if cfg["close_pos"] > cfg["open_pos"] else -1
    bus.move(sid, cfg["close_pos"], ms)
    contact = wait_move(bus, sid, cfg["close_pos"], ms / 1000 + 1.5)
    if contact is not None and abs(contact - cfg["close_pos"]) > 3:
        hold = contact + direction * 15
        bus.move(sid, hold, 100)
        print(f"Kep vat tai pos {contact} (joint {pos_to_joint(cfg, contact):+.4f} m), giu o {hold}")
    else:
        print("Dong het hanh trinh - khong co vat.")


def calib(bus, cfg):
    sid = cfg["id"]
    bus.set_torque(sid, False)
    print("Da tha torque. Xoay gripper bang tay.")
    input("  -> Dua ve vi tri MO HOAN TOAN roi Enter ")
    o = bus.read_pos(sid)
    input("  -> Dua ve vi tri DONG (2 ma cach ~10 mm, nhu model) roi Enter ")
    c = bus.read_pos(sid)
    if o is None or c is None:
        print("Doc vi tri that bai, khong luu.")
        return
    print(f"open_pos={o}  close_pos={c}")
    if input("Luu? [y/N] ").strip().lower() == "y":
        cfg["open_pos"], cfg["close_pos"] = o, c
        save_config(cfg)
    bus.move(sid, o, 1000)  # also re-enables torque


def scan(bus):
    found = []
    old = bus.timeout
    bus.timeout = 0.015
    for sid in range(0, 254):
        if bus.request(sid, 14, retries=0) is not None:
            found.append(sid)
            print(f"  thay ID {sid}")
    bus.timeout = old
    print(f"Xong: {found or 'khong thay servo nao'}")


def watch(bus, cfg):
    try:
        while True:
            pos = bus.read_pos(cfg["id"])
            if pos is not None:
                joint = f"{pos_to_joint(cfg, pos):+.4f}" if cfg["close_pos"] is not None else "?"
                print(f"\r  pos {pos:4d}  {pos * 0.24:6.1f} deg  joint {joint} m ",
                      end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()


def main():
    cfg = load_config()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=cfg["port"], help="auto (mac dinh) hoac /dev/ttyACMx")
    ap.add_argument("--id", type=int, default=cfg["id"])
    args = ap.parse_args()
    cfg["port"], cfg["id"] = args.port, args.id

    try:
        bus = BusServo(cfg["port"])
    except (serial.SerialException, OSError) as e:
        print(f"Loi mo cong: {e}\nKiem tra: cap USB, nguon servo, quyen dialout.")
        return
    if bus.read_pos(cfg["id"]) is None:
        found = bus.read_id(BROADCAST_ID)  # works when only one servo is on the bus
        if found is not None:
            print(f"ID {cfg['id']} khong tra loi, dung servo ID {found}.")
            cfg["id"] = found
    print(f"{bus.port}  servo ID {cfg['id']}  open={cfg['open_pos']} close={cfg['close_pos']}  ('h' de xem lenh)")
    status(bus, cfg)

    while True:
        try:
            parts = input("gripper> ").split()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not parts:
            continue
        cmd, a = parts[0].lower(), parts[1:]
        sid = cfg["id"]
        if cmd in ("o", "open", "c", "close", "g", "grip", "j") and cfg["close_pos"] is None:
            print("Chua calib vi tri mo/dong: chay 'calib' truoc.")
            continue
        try:
            if cmd in ("q", "quit", "exit"):
                break
            elif cmd in ("h", "help", "?"):
                print(HELP)
            elif cmd in ("s", "status"):
                status(bus, cfg)
            elif cmd == "scan":
                scan(bus)
            elif cmd == "m":
                move(bus, cfg, int(a[0]), int(a[1]) if len(a) > 1 else None)
            elif cmd == "a":
                move(bus, cfg, round(float(a[0]) / 0.24), int(a[1]) if len(a) > 1 else None)
            elif cmd in ("o", "open"):
                move(bus, cfg, cfg["open_pos"])
            elif cmd in ("c", "close"):
                move(bus, cfg, cfg["close_pos"])
            elif cmd in ("g", "grip"):
                grip(bus, cfg, int(a[0]) if a else 1500)
            elif cmd == "j":
                move(bus, cfg, joint_to_pos(cfg, float(a[0])))
            elif cmd == "stop":
                bus.stop(sid)
            elif cmd == "free":
                bus.set_torque(sid, False)
            elif cmd == "hold":
                bus.set_torque(sid, True)
            elif cmd in ("w", "watch"):
                watch(bus, cfg)
            elif cmd == "motor":
                bus.set_motor_mode(sid, None if a[0] == "off" else int(a[0]))
            elif cmd == "calib":
                calib(bus, cfg)
            elif cmd == "setid":
                new = int(a[0])
                if input(f"Doi ID {sid} -> {new}? Chi cam 1 servo. [y/N] ").strip().lower() == "y":
                    bus.set_id(sid, new)
                    time.sleep(0.1)
                    cfg["id"] = new
                    save_config(cfg)
            elif cmd == "limit":
                bus.set_angle_limit(sid, int(a[0]), int(a[1]))
                time.sleep(0.05)
                print("Gioi han:", bus.read_angle_limit(sid))
            else:
                print("Lenh khong hop le, go 'h'.")
        except (IndexError, ValueError):
            print("Thieu/sai tham so, go 'h'.")
        except (serial.SerialException, OSError) as e:
            print(f"Loi bus: {e}")
    bus.close()


if __name__ == "__main__":
    main()
