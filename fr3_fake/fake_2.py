#!/usr/bin/env python3
"""
PC -> USB-RS485 -> MAX485 RO -> ESP32 Test

This script sends a predefined message from PC via USB-RS485.
The MAX485 module should receive it and output via RO to ESP32.
ESP32 will then print received characters to its Serial Monitor.

Configuration:
  RS485 Port: Defaults to /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
  RS485 Baud: 9600
  Message: "Test message for MAX485 RO"
  Send Interval: 2 seconds
"""

import argparse
import os
import serial
import time

DEFAULT_RS485_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"


def open_serial(port: str, baud: int) -> serial.Serial:
    if not os.path.exists(port):
        raise SystemExit(f"ERROR: port not found: {port}\nCheck: ls -l /dev/serial/by-id /dev/ttyUSB*")

    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1,
        write_timeout=1,
    )

    # Ensure control lines are stable for CH340 in receive mode
    ser.rts = False
    ser.dtr = False
    time.sleep(0.1)
    return ser


def send_message(ser: serial.Serial, message: str):
    # Add a newline character to signify end of message for ESP32 parsing
    encoded_message = (message + "\n").encode('utf-8')
    n = ser.write(encoded_message)
    ser.flush()
    print(f"PC TX: \'{message}\n\' ({n} bytes)", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Send a string over USB-RS485 to test MAX485 RO output")
    parser.add_argument("-p", "--port", default=DEFAULT_RS485_PORT, help="USB-RS485 serial port")
    parser.add_argument("-b", "--baud", type=int, default=9600, help="baudrate")
    parser.add_argument("-d", "--delay", type=float, default=2.0, help="delay between messages, seconds")
    parser.add_argument("-m", "--message", default="xin chào MAX485", help="message to send")
    parser.add_argument("-n", "--count", type=int, default=0, help="number of times to send message, 0 = forever")

    args = parser.parse_args()

    print(f"Opening RS485 port {args.port} at {args.baud} baud", flush=True)
    try:
        with open_serial(args.port, args.baud) as ser:
            print(f"Opened RS485 port: {ser.name}", flush=True)

            i = 0
            while args.count == 0 or i < args.count:
                send_message(ser, args.message)
                time.sleep(args.delay)
                i += 1
    except serial.SerialException as e:
        print(f"Serial Error: {e}", flush=True)
        print(f"Please ensure the RS485 port '{args.port}' is correct and not in use.", flush=True)
    except Exception as e:
        print(f"An unexpected error occurred: {e}", flush=True)


if __name__ == "__main__":
    main()
