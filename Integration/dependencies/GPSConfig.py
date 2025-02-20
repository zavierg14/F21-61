import serial
import adafruit_gps
import time

ser = serial.Serial("/dev/ttyUSB0", "9600", timeout =1)
ser.baudrate = 9600
gps = adafruit_gps.GPS(ser, debug=False)
gps.send_command(b"PMTK314,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
gps.send_command(b'PMTK220,1000')


while True:
	print(gps.satellites)
	time.sleep(.5)
