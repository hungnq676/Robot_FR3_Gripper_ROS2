import serial
import time

ser = serial.Serial(
    '/dev/ttyUSB0',
    115200,
    timeout=0.1
)

print("FAKE GRIPPER READY")

while True:

    data = ser.read(64)

    if data:

        print(
            "RX:",
            data.hex(' ')
        )

        time.sleep(0.01)

        # echo back
        ser.write(data)

        print(
            "TX:",
            data.hex(' ')
        )
        
