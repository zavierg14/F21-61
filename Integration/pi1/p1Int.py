import smbus2
import time
import sys
import os

# Sampling Frequency Stuff
sampling_freq = 10 # Hz
interval = 1 /sampling_freq # s

# Magnetic Encoder I2C addy
AS5600_ADDR = 0x36
RAW_ANGLE_REG = 0x0C
AS5600data = [[time.monotonic(), 0.0]]

def csvWrite(data, title, headers):
	with open(title+str(datetime.datetime.now().strftime('%Y%m%d%H%M%S'))+".csv", mode='w', newline='') as file:
		writer = csv.writer(file)
		writer.writerow(headers)
		writer.writerows(data)
		
# Function to Read AS5600 Angle
def read_angle():
	bus = smbus2.SMBus(1)
	data = bus.read_i2c_block_data(AS5600_ADDR, RAW_ANGLE_REG, 2)
	bus.close()
	raw_angle = (data[0] << 8 | date[1])
	angle = (raw_angle / 4096.0) * 360.0
	return angle

# Housekeeping before recording data
last_print = time.monotonic()

# Meat of recording and printing data
try:
	while True:
		current = time.monotonic()
		if current - last_print >= interval:
			last_print = current
			angle = read_angle()
			print(f"Angle: {angle:.2f} def")
			AS5600data.append([current, angle])

except KeyboardInterrupt:
	pass

# Write Data to CSV
csvWrite(AS5600data, "AS5600", ["Time", "Angle (deg)"])
