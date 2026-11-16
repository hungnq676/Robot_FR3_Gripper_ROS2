"""Hiwonder bus servo protocol (HTD-35H / HTS / LX series) over a BusLinker.

Frame: 0x55 0x55 ID LEN CMD PARAM... CHECKSUM
  LEN      = number of PARAM bytes + 3
  CHECKSUM = ~(ID + LEN + CMD + sum(PARAM)) & 0xFF
Position units: 0..1000 = 0..240 deg (0.24 deg / unit), time in ms.

Pure Python + pyserial so the same framing ports 1:1 to the ESP32 later.
"""
import struct
import time

import serial
from serial.tools import list_ports

BROADCAST_ID = 0xFE

MOVE_TIME_WRITE = 1
MOVE_TIME_READ = 2
MOVE_STOP = 12
ID_WRITE = 13
ID_READ = 14
ANGLE_OFFSET_ADJUST = 17
ANGLE_OFFSET_WRITE = 18
ANGLE_OFFSET_READ = 19
ANGLE_LIMIT_WRITE = 20
ANGLE_LIMIT_READ = 21
VIN_LIMIT_READ = 23
TEMP_MAX_LIMIT_READ = 25
TEMP_READ = 26
VIN_READ = 27
POS_READ = 28
MODE_WRITE = 29
LOAD_WRITE = 31
LOAD_READ = 32
LED_ERROR_READ = 36


def checksum(body):
    return (~sum(body)) & 0xFF


def build_frame(servo_id, cmd, params=b""):
    body = bytes([servo_id, len(params) + 3, cmd]) + bytes(params)
    return b"\x55\x55" + body + bytes([checksum(body)])


def parse_frames(buf):
    """Yield (id, cmd, params) for every valid frame in buf."""
    i = 0
    while i + 6 <= len(buf):
        if buf[i] != 0x55 or buf[i + 1] != 0x55:
            i += 1
            continue
        length = buf[i + 3]
        end = i + 2 + length  # index of checksum byte
        if length < 3 or end >= len(buf):
            i += 1
            continue
        body = buf[i + 2:end]
        if checksum(body) == buf[end]:
            yield body[0], body[2], bytes(body[3:])
            i = end + 1
        else:
            i += 1


# USB-serial chips the BusLinker (and later an ESP32 bridge) may use:
# WCH CH340/CH9102, Silicon Labs CP210x, FTDI.
USB_SERIAL_VIDS = (0x1A86, 0x10C4, 0x0403)


def candidate_ports():
    """USB serial ports, likely BusLinker chips first (ttyS* are skipped)."""
    ports = [p for p in list_ports.comports() if p.vid is not None]
    ports.sort(key=lambda p: p.vid not in USB_SERIAL_VIDS)
    return [p.device for p in ports]


def probe(port, baud=115200, servo_id=BROADCAST_ID):
    """Return the ID of a servo answering on port, else None."""
    try:
        bus = BusServo(port, baud, timeout=0.03, auto_reconnect=False)
    except (serial.SerialException, OSError):
        return None
    try:
        r = bus.request(servo_id, ID_READ, retries=1)
        return r[0] if r else None
    except (serial.SerialException, OSError):
        return None
    finally:
        bus.close()


def find_port(baud=115200, servo_id=BROADCAST_ID):
    """Find the port whose bus has a servo answering. Returns (port, id) or
    (None, None). servo_id=BROADCAST_ID only works with one servo on the bus."""
    for port in candidate_ports():
        sid = probe(port, baud, servo_id)
        if sid is not None:
            return port, sid
    return None, None


class BusServo:
    """port=None or "auto": scan USB serial ports for one with a servo.
    With auto_reconnect, an unplug/replug (even onto a new ttyACM/ttyUSB name)
    is recovered on the next command."""

    def __init__(self, port=None, baud=115200, timeout=0.05, servo_id=BROADCAST_ID,
                 auto_reconnect=True):
        self.baud = baud
        self.timeout = timeout
        self.servo_id = servo_id
        self.auto_reconnect = auto_reconnect
        self.ser = None
        if port in (None, "", "auto"):
            port, _ = find_port(baud, servo_id)
            if port is None:
                raise serial.SerialException(
                    f"Khong tim thay servo tren cong USB nao: {candidate_ports() or 'khong co cong USB'}")
        self.port = port
        self.ser = serial.Serial(port, baud, timeout=timeout)

    def close(self):
        if self.ser:
            self.ser.close()

    def reconnect(self, wait_s=10.0):
        """Re-find the bus after an unplug; waits up to wait_s for the replug."""
        self.close()
        deadline = time.monotonic() + wait_s
        while True:
            port, _ = find_port(self.baud, self.servo_id)
            if port is not None:
                self.port = port
                self.ser = serial.Serial(port, self.baud, timeout=self.timeout)
                print(f"[bus] ket noi lai o {port}")
                return
            if time.monotonic() > deadline:
                raise serial.SerialException("Mat ket noi BusLinker, khong tim lai duoc.")
            time.sleep(0.5)

    # ---- low level -------------------------------------------------------
    def _send(self, frame):
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()

    def write(self, servo_id, cmd, params=b""):
        frame = build_frame(servo_id, cmd, params)
        try:
            self._send(frame)
        except (serial.SerialException, OSError):
            if not self.auto_reconnect:
                raise
            print("[bus] mat ket noi, dang tim lai cong...")
            self.reconnect()
            self._send(frame)
        return frame

    def _read(self, n):
        try:
            return self.ser.read(n)
        except (serial.SerialException, OSError):
            if not self.auto_reconnect:
                raise
            self.reconnect()
            return b""

    def request(self, servo_id, cmd, params=b"", retries=2):
        """Send a read command and return the reply params (None on timeout).

        Some USB adapters echo the TX frame on the half-duplex bus, so the echo
        is skipped and only a frame with a different payload is accepted.
        """
        for _ in range(retries + 1):
            sent = self.write(servo_id, cmd, params)
            buf = b""
            deadline = time.monotonic() + self.timeout * 2
            while time.monotonic() < deadline:
                buf += self._read(64)
                for rid, rcmd, rparams in parse_frames(buf):
                    if rcmd != cmd:
                        continue
                    if build_frame(rid, rcmd, rparams) == sent:
                        continue  # our own echo
                    if servo_id != BROADCAST_ID and rid != servo_id:
                        continue
                    return rparams
        return None

    # ---- motion ----------------------------------------------------------
    def move(self, servo_id, pos, time_ms=500):
        pos = max(0, min(1000, int(pos)))
        time_ms = max(0, min(30000, int(time_ms)))
        self.write(servo_id, MOVE_TIME_WRITE, struct.pack("<HH", pos, time_ms))

    def stop(self, servo_id):
        self.write(servo_id, MOVE_STOP)

    def set_torque(self, servo_id, on):
        """on=False lets the horn turn freely by hand (unload)."""
        self.write(servo_id, LOAD_WRITE, bytes([1 if on else 0]))

    def set_motor_mode(self, servo_id, speed):
        """Continuous rotation, speed -1000..1000. speed=None -> servo mode."""
        if speed is None:
            self.write(servo_id, MODE_WRITE, struct.pack("<BBh", 0, 0, 0))
        else:
            speed = max(-1000, min(1000, int(speed)))
            self.write(servo_id, MODE_WRITE, struct.pack("<BBh", 1, 0, speed))

    # ---- reads -----------------------------------------------------------
    def read_id(self, servo_id=BROADCAST_ID):
        r = self.request(servo_id, ID_READ)
        return r[0] if r else None

    def read_pos(self, servo_id):
        r = self.request(servo_id, POS_READ)
        return struct.unpack("<h", r[:2])[0] if r and len(r) >= 2 else None

    def read_vin(self, servo_id):
        r = self.request(servo_id, VIN_READ)
        return struct.unpack("<H", r[:2])[0] / 1000.0 if r and len(r) >= 2 else None

    def read_temp(self, servo_id):
        r = self.request(servo_id, TEMP_READ)
        return r[0] if r else None

    def read_angle_limit(self, servo_id):
        r = self.request(servo_id, ANGLE_LIMIT_READ)
        return struct.unpack("<HH", r[:4]) if r and len(r) >= 4 else None

    def read_offset(self, servo_id):
        r = self.request(servo_id, ANGLE_OFFSET_READ)
        return struct.unpack("<b", r[:1])[0] if r else None

    def read_load(self, servo_id):
        r = self.request(servo_id, LOAD_READ)
        return bool(r[0]) if r else None

    def read_alarm_config(self, servo_id):
        """Which faults blink the LED: bit0 over-temp, bit1 over-volt, bit2 stall."""
        r = self.request(servo_id, LED_ERROR_READ)
        return r[0] if r else None

    # ---- config (written to servo EEPROM) --------------------------------
    def set_id(self, old_id, new_id):
        self.write(old_id, ID_WRITE, bytes([new_id]))

    def set_angle_limit(self, servo_id, lo, hi):
        self.write(servo_id, ANGLE_LIMIT_WRITE, struct.pack("<HH", lo, hi))
