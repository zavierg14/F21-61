import serial
import time
import adafruit_gps

# Open serial connection at the current baud rate (9600 by default)
gps = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=1)

# PMTK command to change baud rate to 115200
pmtk_baud_115200 = "$PMTK251,115200*1F\r\n"

# Send command
gps.write(pmtk_baud_115200.encode())

# Give GPS time to process command
time.sleep(1)

# Close and reopen serial connection at new baud rate
gps.close()

ser = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=1)
ser.baudrate = 115200
print("Baud rate changed to 115200!")
gps = adafruit_gps.GPS(ser, debug = False)
# Test by reading GPS data
while True:
	data = ser.readline().decode('utf-8', errors='ignore').strip()
	if data:
		print(data)
	gps.update()
	print(gps.latitude)
ser.close()


