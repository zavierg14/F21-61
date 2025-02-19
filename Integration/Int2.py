# Kaleb Binger 2/2025
# F21-61 - Swap Hudson
# This is v2 of the file moving from printing to txt to puting each value in a numpy array.
# :)

import csv
import serial			# Serial to mess w sensors
import time			# Lets us check what time it is
import adafruit_gps		# Tells what the GPS is thinking
import numpy as np		# We need to use some arrays
import datetime			# Bro idek but the dependencies need it
import sys			# ^^^^^^
import platform			# ^^^^^^
import struct			# ^^^^^^
import dependencies.chs.lib.device_model as deviceModel	# Actual local file w the deviceModel class
import dependencies.chs.JY901S as JY			# WitMotion's actual file that I'm stealing functions from :)
from dependencies.chs.lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor	# WitMotion file to help w data parsing i think
from dependencies.chs.lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver	# ^^^^^
import dependencies.util_func as util_func	# My file :^) this is super useful

# Sampling Frequency Stuff
sampling_freq = 5 #Hz ---------- frequency we want
interval = 1/sampling_freq #s -- time in seconds between samples

# Declaring GPS Port and Baud
GPSserial_port = "/dev/ttyUSB0" # GPS serial port
GPSbaud_rate = 9600 		# Default GPS baud rate
GPSdata = [[time.monotonic(),0,0,0,0,0]]	# Time/lat/long/altitude/speed/sat count

# Declaring IMU Port and baud
IMUserial_port = "/dev/ttyAMA0" # IMU Serial port
IMUbaud_rate = 9600		# Default IMU baud rate
IMUdata = [[time.monotonic(),0,0,0,0,0,0]] # Time/accX/accY/accZ/angleX/AngleY/AngleZ

# Make GPS Serial object
GPSser = serial.Serial(GPSserial_port, GPSbaud_rate, timeout=.3) 		# Using our fancy serial import, create an object for the GPS with the port name, baud rate, and timeout
GPSser.baudrate = GPSbaud_rate					 		# Change baudrate to wakeup GPS
gps = adafruit_gps.GPS(GPSser, debug = False)			 		# Create gps object with adafruit parser
gps.send_command(b"PMTK314,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")	# See adafruit documentation, telling GPS what data we want
gps.send_command(b'PMTK220,200')						# See adafruit documentation, telling GPS how fast we want to update date

# Make IMU Serial Object
device = deviceModel.DeviceModel("IMU",WitProtocolResolver(),JY901SDataProcessor(),"51_0")	# WitMotion libraries, make device object
device.serialConfig.portName = IMUserial_port   						# Set serial port
device.serialConfig.baud = IMUbaud_rate                 					# Set baud rate
device.openDevice()                                 						# Open serial port

# Housekeeping before recording data
last_print = time.monotonic()	# Start time for sampling

# Meat of recording and printing data
try:							# Try & except to give a way of ending loop someday 
	while True:					# While loop to continue checking sensors
		gps.update()				# Command to make the parsing library check if theres anything new from GPS
		current = time.monotonic()		# Check current time
		if current - last_print >= interval:	# Compare elapsed time to sampling interval

			# If the elapsed time is past the interval it will check the sensors 
			last_print = current		# Set time since last sample to current time
			if not gps.has_fix:		# Check for GPS fix

				# Try again if we don't have a fix yet.
				print("Waiting for fix...")
				continue			# Continue loop until fix is obtained
			print("=" * 40) 			# Print a separator line.
			util_func.deviceWrite(gps, device, False)
			imutemp = [time.monotonic(), device.getDeviceData("accX"), device.getDeviceData("accY"), device.getDeviceData("accZ"), device.getDeviceData("angleX"), device.getDeviceData("angleY"), device.getDeviceData("angleZ")]
			gpstemp = [time.monotonic(), gps.latitude, gps.longitude, gps.altitude_m, gps.speed_kmh, gps.satellites]
			GPSdata.append(gpstemp)
			IMUdata.append(imutemp)

except KeyboardInterrupt:	# Ctrl+C sends keyboard interupt and stops loop
	pass			# Does literally nothing but stop python from whining

# Close serial devices
GPSser.close()			# Closes gps serial
device.closeDevice()		# Closes IMU serial and stops thread - can take a few seconds don't freak out :)

# Write to file
util_func.csvWrite(GPSdata, "GPS.csv", ["Time", "Lat", "Long", "Alt", "Speed", "Sats"])
util_func.csvWrite(IMUdata, "IMU.csv", ["Time", "AccX", "AccY", "AccZ", "AngleX", "AngleY", "AngleZ"])



##
#~Camel~
#   .--' |
#  /___^ |     .--.
#      ) |    /    \
#     /  |  /`      '.
#    |   '-'    /     \
#    \         |      |\
#     \    /   \      /\|
#      \  /'----`\   /
#      |||       \\ |
#      ((|        ((|
#      |||        |||
#     //_(       //_(



# Happy Driving
# F21-61 - Kaleb Binger
# soli deo gloria
# Ιησους Χριστος
