#!/bin/python3
import can

bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)
try:
	while True:
		msg = bus.recv()
		print(msg)
except KeyboardInterrupt():
	print("done")
