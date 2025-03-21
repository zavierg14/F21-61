# Kaleb Binger 2/2025
# F21-61 - Swap Hudson
# v4 of code introducing both potentiometers and 550Hz sampling
# :)

import serial			# Serial to mess w sensors
import time			# Lets us check what time it is
import adafruit_gps		# Tells what the GPS is thinking
import datetime			# Bro idek but the dependencies need it
import dependencies.chs.lib.device_model as deviceModel	# Actual local file w the deviceModel class
import dependencies.chs.JY901S as JY			# WitMotion's actual file that I'm stealing functions from :)
from dependencies.chs.lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor	# WitMotion file to help w data parsing i think
from dependencies.chs.lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver	# ^^^^^
import dependencies.util_func as util_func	# My file :^) this is super useful
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Sampling Frequency Stuff
slow_sampling_freq = 10 #Hz ---------- frequency we want
interval = 1/slow_sampling_freq #s -- time in seconds between samples
fast_sampling_freq = 3300 #Hz --------samples persecond for ADS
interval2 = 1/fast_sampling_freq #S----time in seconds between ads samples

# Declaring GPS Port and Baud
GPSserial_port = "/dev/ttyUSB0" # GPS serial port
GPSbaud_rate = 115200 		# Default GPS baud rate
GPSdata = [[time.perf_counter(),0,0,0,0,0]]	# Time/lat/long/altitude/speed/sat count

# Declaring IMU Port and baud
IMUserial_port = "/dev/ttyAMA0" # IMU Serial port
IMUbaud_rate = 9600		# Default IMU baud rate
IMUdata = [[time.perf_counter(),0,0,0,0,0,0]] # Time/accX/accY/accZ/angleX/AngleY/AngleZ

# Make GPS Serial object
GPSser = serial.Serial(GPSserial_port, GPSbaud_rate, timeout=.1) 		# Using our fancy serial import, create an object for the GPS with the port name, baud rate, and timeout
GPSser.baudrate = GPSbaud_rate					 		# Change baudrate to wakeup GPS
gps = adafruit_gps.GPS(GPSser, debug = False)			 		# Create gps object with adafruit parsery
gps.send_command(b"PMTK314,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")	# See adafruit documentation, telling GPS what data we want
gps.send_command(b'PMTK220,100')						# See adafruit documentation, telling GPS how fast we want to update date

# Make IMU Serial Object
device = deviceModel.DeviceModel("IMU",WitProtocolResolver(),JY901SDataProcessor(),"51_0")	# WitMotion libraries, make device object
device.serialConfig.portName = IMUserial_port   						# Set serial port
device.serialConfig.baud = IMUbaud_rate                 					# Set baud rate
device.openDevice()                                 						# Open serial port

# Linear Potentiometer 1 configuration
i2c = busio.I2C(board.SCL, board.SDA)	# Declaring I2C object
adc = ADS.ADS1015(i2c)			# ADS1015 object
CONFIG_REGISTER = 0x01 			# Honestly idk ask zavier
util_func.set_continuous_mode(adc, CONFIG_REGISTER)	# Enable continuous mode
config_value = adc._read_register(CONFIG_REGISTER, 2)
#print(f"Config Register after setting continuous mode: {bin(config_value)}")
adc.data_rate = fast_sampling_freq	# Set ADS1015 to an appropriate sample rate (predetermined by hardware-see datasheet)
pot_channel1 = AnalogIn(adc, ADS.P0)	# Initialize channel in Single-Ended Mode
Pot1data = [[time.perf_counter(), 0]]	# Time/rawvalue/voltage

# Linear Potentiometer 2 configuration
pot_channel2 = AnalogIn(adc, ADS.P1)
Pot2data = [[time.perf_counter(), 0]]

# Housekeeping before recording data
slast_print = time.perf_counter()	# Start time for sampling
flast_print = slast_print
start = slast_print

# Meat of recording and printing data
try:							# Try & except to give a way of ending loop someday 
	while True:					# While loop to continue checking sensors
		gps.update()				# Command to make the parsing library check if theres anything new from GPS
		current = time.perf_counter()		# Check current time
		if current - slast_print >= interval:	# Compare elapsed time to sampling interval

			# If the elapsed time is past the interval it will check the sensors 
			slast_print = current		# Set time since last sample to current time
			if not gps.has_fix:		# Check for GPS fix
				imutemp = [time.perf_counter(), device.getDeviceData("accx"), device.getDeviceData("accY"), device.getDeviceData("accZ"), device.getDeviceData("angleX"), device.getDeviceData("angleY"), device.getDeviceData("angleZ")]
				# Try again if we don't have a fix yet.
#				print("Waiting for fix...")
				continue			# Continue loop until fix is obtained
			imutemp = [time.perf_counter(), device.getDeviceData("accX"), device.getDeviceData("accY"), device.getDeviceData("accZ"), device.getDeviceData("angleX"), device.getDeviceData("angleY"), device.getDeviceData("angleZ")]		# Current time step IMU Data
			gpstemp = [time.perf_counter(), gps.latitude, gps.longitude, gps.altitude_m, gps.speed_kmh, gps.satellites]		# Current time step GPS data
			GPSdata.append(gpstemp)			# Append GPS data to big list
			IMUdata.append(imutemp)			# Append IMU data to big list
		if current - flast_print >= interval2:
			raw_value1 = pot_channel1.value	# Read ADC Values
			raw_value2 = pot_channel2.value
			if raw_value1 < 0:
				raw_value1 = 0
			if raw_value2 < 0:
				raw_value2 = 0
#			print(f"Time: {time.perf_counter()-flast_print:.4f}s | Raw 1: {raw_value1:.2f} |, Raw 2: {raw_value2:.2f}")
			Pot1data.append([time.perf_counter(), raw_value1])
			Pot2data.append([time.perf_counter(), raw_value1])
			flast_print=current

except KeyboardInterrupt:	# Ctrl+C sends keyboard interupt and stops loop
	pass			# Does literally nothing but stop python from whining

# Close serial devices
GPSser.close()			# Closes gps serial
device.closeDevice()		# Closes IMU serial and stops thread - can take a few seconds don't freak out :)

# Write to file
util_func.csvWriteUSB(GPSdata, "GPS", ["Time", "Lat", "Long", "Alt", "Speed", "Sats"])				# GPS
util_func.csvWriteUSB(IMUdata, "IMU", ["Time", "AccX", "AccY", "AccZ", "AngleX", "AngleY", "AngleZ"])		# IMU
util_func.csvWriteUSB(Pot1data, "Pot1", ["Time", "Raw Value"])
util_func.csvWriteUSB(Pot2data, "Pot2", ["Time", "Raw Value"])

