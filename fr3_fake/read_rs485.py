import serial

ser = serial.Serial(
    '/dev/ttyUSB0',
    115200,
    timeout=1
)

print("Waiting RS485 data...")

while True:

    data = ser.read()

    if data:

        print("RX :", data.hex())
