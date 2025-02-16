import serial
import time
import adafruit_gps
import datetime
import sys
import platform
import struct
import dependencies.chs.lib.device_model as deviceModel
import dependencies.chs.JY901S as JY
from dependencies.chs.lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from dependencies.chs.lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver
import dependencies.util_func as util_func

# Declaring GPS Port and Baud
GPSserial_port = "/dev/ttyUSB0" # GPS serial port
GPSbaud_rate = 9600 		# Default GPS baud rate

# Declaring IMU Port and baud
IMUserial_port = "/dev/ttyAMA0" #IMU Serial port
IMUbaud_rate = 9600		#Default IMU baud rate

# Make GPS Serial object
GPSser = serial.Serial(GPSserial_port, GPSbaud_rate, timeout=.3) 		# Using our fancy serial import, create an object for the GPS with the port name, baud rate, and timeout
GPSser.baudrate = GPSbaud_rate					 		# Change baudrate to wakeup GPS
gps = adafruit_gps.GPS(GPSser, debug = False)			 		# Create gps object with adafruit parser
gps.send_command(b"PMTK314,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")	# See adafruit documentation, telling GPS what data we want
gps.send_command(b'PMTK220,200')						# See adafruit documentation, telling GPS how fast we want to update date

# Make IMU Serial Object
device = deviceModel.DeviceModel("IMU",WitProtocolResolver(),JY901SDataProcessor(),"51_0")
device.serialConfig.portName = IMUserial_port   #Set serial port
device.serialConfig.baud = IMUbaud_rate                 #Set baud rate
device.openDevice()                                 #Open serial port

print("Waiting for GPS fix...")
last_print = time.monotonic()
util_func.startRecord()
try:
	while True:
		gps.update()
		current = time.monotonic()
		if current - last_print >= .2:
			last_print = current
			if not gps.has_fix:
				# Try again if we don't have a fix yet.
				print("Waiting for fix...")
				continue
			print("=" * 40) # Print a separator line.
			util_func.deviceWrite(gps, device, False)
except KeyboardInterrupt:
	pass

util_func.endRecord()                                         #End record data
GPSser.close()
device.closeDevice()
