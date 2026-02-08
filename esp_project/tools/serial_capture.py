#!/usr/bin/env python3
"""Capture raw UART for N seconds. Usage: python serial_capture.py PORT [seconds]"""
import sys
import time

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem5ABA0603751"
    secs = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    try:
        import serial
    except ImportError:
        print("pip install pyserial", file=sys.stderr)
        sys.exit(1)
    ser = serial.Serial(port, 115200, timeout=0.1)
    print(f"Capturing {port} for {secs}s...", file=sys.stderr)
    start = time.monotonic()
    while time.monotonic() - start < secs:
        chunk = ser.read(4096)
        if chunk:
            sys.stdout.buffer.write(chunk)
            sys.stdout.buffer.flush()
    ser.close()

if __name__ == "__main__":
    main()
