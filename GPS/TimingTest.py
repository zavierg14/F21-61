# -*- coding: utf-8 -*-
"""
Created on Thu Jan 30 23:39:19 2025

@author: kaleb
"""

import serial
import time

# Adjust serial_port to your device (e.g., /dev/ttyAMA0, /dev/serial0, etc.)
serial_port = 'COM3'
baud_rateo = 115200
baud_raten = 9600  # Adafruit Ultimate GPS default baud rate
lasttime = time.monotonic()
samplingfreq = 4;
interval = 1/samplingfreq

try: 
    ser = serial.Serial(serial_port, baud_rateo, timeout = .1)
except serial.SerialException as e:
   print(f"Error opening serial port {serial_port}: {e}")
   exit(1)
ser.baudrate = baud_raten

while True:
    try:
        elapsed = time.monotonic()-lasttime
        if elapsed > interval:
            line = ser.readline().decode("ascii", errors='replace').strip()
            print(elapsed)
            print(line)
            lasttime = time.monotonic()
    except KeyboardInterrupt:
        print("\nExiting...")
        break
    
ser.close()
