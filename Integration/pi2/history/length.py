#!/bin/python3
import can
import dependencies.util_funcv2 as util_func

bus = can.interface.Bus(channel='can0', interface='socketcan', bitrate=500000)
A1 = []
try:
	while True:
		msg = bus.recv()
		if msg.arbitration_id == 0xE1 and msg.data[0] == 1:
			print(msg)
		elif msg.arbitration_id == 0xA1:
			timestamp, value = util_func.parse_can_data(msg.data)
			A1.append([timestamp, value])
		elif msg.arbitration_id == 0xA2:
			timestamp, value = util_func.parse_can_data(msg.data)
			print(timestamp, value)
		elif msg.arbitration_id == 0xB1:
			timestamp, value = util_func.parse_can_data(msg.data)
			print(timestamp, value)
		elif msg.arbitration_id == 0xE1 and msg.data[0] == 3:
			break
		else:
			print("\nbad")
			#print(msg.data)
			print(msg.arbitration_id)
	print(len(A1))

except KeyboardInterrupt():
	print("done")
	msg = bust.recv(1.0)
	bus.close()
