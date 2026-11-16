import serial

ser = serial.Serial(
    '/dev/ttyUSB0',
    230400,
    timeout=0
)

print("START")

while True:

    if ser.in_waiting:

        b = ser.read(1)

        print(
            hex(b[0]),
            end=' ',
            flush=True
        )
