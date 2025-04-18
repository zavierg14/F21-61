#!/bin/python3
import can
import dependencies.util_funcv2 as util_func

bus = can.interface.Bus(channel='can0', interface='socketcan', bitrate=500000)
print(0x123)
print(0xA11)
print(0xA12)
print(0xB11)
try:
	while True:
		msg = bus.recv()
		if msg.arbitration_id == 0x123 and msg.data[0] == 1:
			print(msg)
		elif msg.arbitration_id == 0xA11:
			timestamp, value = util_func.parse_can_data(msg.data)
			print(timestamp, value)
		elif msg.arbitration_id == 0xA12:
			timestamp, value = util_func.parse_can_data(msg.data)
			print(timestamp, value)
		elif msg.arbitration_id == 0xB11:
			timestamp, value = util_func.parse_can_data(msg.data)
			print(timestamp, value)
		else:
			print("\nbad")
			#print(msg.data)
			print(msg.arbitration_id)
except KeyboardInterrupt():
	print("done")
	msg = bust.recv(1.0)
	bus.close()
