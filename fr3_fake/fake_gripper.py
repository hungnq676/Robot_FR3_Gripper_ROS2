import serial
import struct
import time

def crc16_modbus(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)

def verify_crc(data: bytes) -> bool:
    if len(data) < 3:
        return False
    payload = data[:-2]
    expected = crc16_modbus(payload)
    actual = data[-2:]
    return expected == actual

def parse_and_reply(data: bytes, ser: serial.Serial):
    # Tìm packet Modbus hợp lệ trong data (vì đôi khi bị lẫn noise)
    i = 0
    while i < len(data) - 3:
        # Thử parse từ vị trí i
        chunk = data[i:]
        
        if len(chunk) < 4:
            break
            
        slave_id = chunk[0]
        func_code = chunk[1]
        
        # Slave ID phải là 1-247
        if slave_id == 0 or slave_id > 247:
            i += 1
            continue

        # ── FC 0x03: Read Holding Registers ──────────────────
        # Request: [slave][03][RegH][RegL][nRegH][nRegL][CRC×2] = 8 bytes
        if func_code == 0x03 and len(chunk) >= 8:
            packet = chunk[:8]
            if not verify_crc(packet):
                i += 1
                continue
            
            reg = (packet[2] << 8) | packet[3]
            n_reg = (packet[4] << 8) | packet[5]
            
            print(f"  [READ ] slave={slave_id:#04x} reg={reg:#06x} n={n_reg}")
            
            # Reply: [slave][03][byteCount][data...][CRC×2]
            # Trả về toàn 0x00 — gripper "idle, no error"
            byte_count = n_reg * 2
            resp_body = bytes([slave_id, 0x03, byte_count]) + bytes(byte_count)
            resp = resp_body + crc16_modbus(resp_body)
            
            time.sleep(0.005)  # 5ms delay như gripper thật
            ser.write(resp)
            print(f"  [REPLY] {resp.hex(' ').upper()}")
            i += 8
            continue

        # ── FC 0x06: Write Single Register ───────────────────
        # Request: [slave][06][RegH][RegL][DataH][DataL][CRC×2] = 8 bytes
        if func_code == 0x06 and len(chunk) >= 8:
            packet = chunk[:8]
            if not verify_crc(packet):
                i += 1
                continue
            
            reg  = (packet[2] << 8) | packet[3]
            data_val = (packet[4] << 8) | packet[5]
            
            print(f"  [WRITE1] slave={slave_id:#04x} reg={reg:#06x} val={data_val:#06x} ({data_val})")
            decode_register(reg, packet[4], packet[5])
            
            # ACK = echo nguyên packet
            resp = packet  # FC06 ACK = mirror
            time.sleep(0.005)
            ser.write(resp)
            print(f"  [REPLY] {resp.hex(' ').upper()}")
            i += 8
            continue

        # ── FC 0x10: Write Multiple Registers ────────────────
        # Request: [slave][10][RegH][RegL][nRegH][nRegL][byteCount][data...][CRC×2]
        if func_code == 0x10 and len(chunk) >= 7:
            byte_count = chunk[6]
            total_len = 7 + byte_count + 2  # header + data + CRC
            
            if len(chunk) < total_len:
                i += 1
                continue
                
            packet = chunk[:total_len]
            if not verify_crc(packet):
                i += 1
                continue
            
            reg    = (packet[2] << 8) | packet[3]
            n_reg  = (packet[4] << 8) | packet[5]
            payload = packet[7:7+byte_count]
            
            print(f"  [WRITE+] slave={slave_id:#04x} reg={reg:#06x} n={n_reg} data={payload.hex(' ').upper()}")
            decode_register(reg, payload[0] if len(payload)>0 else 0,
                                 payload[1] if len(payload)>1 else 0)
            
            # ACK: [slave][10][RegH][RegL][nRegH][nRegL][CRC×2] = 8 bytes
            ack_body = bytes([slave_id, 0x10, packet[2], packet[3], packet[4], packet[5]])
            ack = ack_body + crc16_modbus(ack_body)
            time.sleep(0.005)
            ser.write(ack)
            print(f"  [REPLY] {ack.hex(' ').upper()}")
            i += total_len
            continue

        # ── FC khác (0x01, 0x02, 0x04...) ───────────────────
        # Trả lỗi exception để robot biết mình không hỗ trợ
        if func_code in (0x01, 0x02, 0x04, 0x05, 0x0F) and len(chunk) >= 8:
            packet = chunk[:8]
            if verify_crc(packet):
                print(f"  [UNKNOWN FC] slave={slave_id:#04x} fc={func_code:#04x} → reply exception")
                # Modbus exception: FC | 0x80, exception code 0x01 (illegal function)
                exc_body = bytes([slave_id, func_code | 0x80, 0x01])
                exc = exc_body + crc16_modbus(exc_body)
                ser.write(exc)
                i += 8
                continue

        i += 1

def decode_register(reg: int, byte_hi: int, byte_lo: int):
    """In ra ý nghĩa của lệnh dựa trên register address"""
    label = {
        0x03E8: "POSITION  ",
        0x03E9: "SPEED     ",
        0x03EA: "TORQUE    ",
        0x07D0: "READ_STAT ",
        0x07D1: "READ_INIT ",
        0x07D2: "READ_ERR  ",
        0x07D3: "READ_POS  ",
    }.get(reg, f"UNKNOWN({reg:#06x})")
    
    print(f"         → {label} hi={byte_hi:#04x}({byte_hi}) lo={byte_lo:#04x}({byte_lo})")

# ─── MAIN ────────────────────────────────────────────────────
ser = serial.Serial(
    port='/dev/ttyUSB0',   # đổi nếu cần
    baudrate=115200,
    bytesize=8,
    parity='N',
    stopbits=1,
    timeout=0.05           # 50ms timeout để gom đủ packet
)

print("=" * 50)
print("FAKE GRIPPER v2 — Auto-reply all Modbus FC")
print("=" * 50)

while True:
    raw = ser.read(256)
    if not raw:
        continue
    
    # Lọc bỏ packet toàn 0x00 (noise)
    if all(b == 0 for b in raw):
        continue
    
    print(f"\nRX ({len(raw)} bytes): {raw.hex(' ').upper()}")
    parse_and_reply(raw, ser)