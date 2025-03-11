# Kaleb Binger 2/2025
# F21-61 - Swap Hudson
# v5: moved IMU and GPS to a seperate process to keep potentiometer running fast enough

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
import multiprocessing

# Sampling Frequency Setup
slow_sampling_freq = 10 #Hz (GPS & IMU sampling rate)
interval = 1/slow_sampling_freq #s GPS & IMU interval
fast_sampling_freq = 3300 #Hz (ADS1015 sampling rate)
interval2 = 1/fast_sampling_freq #s ADS interval

# Data Storage
GPSdata = []
IMUdata = []
Pot1data = []
Pot2data = []

def imu_gps_process(gps_queue, imu_queue):
	'''Runs GPS and IMU processing in a seperate process to avoid slowing down
	other sampling
	'''
	# Initialize GPS
	GPSserial_port = '/dev/ttyUSB0'
	GPSbaud_rate = 115200
	GPSser = serial.Serial(GPSserial_port, GPSbaud_rate, timeout = .1)
	gps = adafruit_gps.GPS(GPSser, debug = False)

	# Configure GPS
	gps.send_command(b"PMTK314,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")	# See adafruit documentation, telling GPS what data we want
	gps.send_command(b'PMTK220,100')						# See adafruit documentation, telling GPS how fast we want to update date

	# Initialize IMU
	device = deviceModel.DeviceModel("IMU",WitProtocolResolver(),JY901SDataProcessor(),"51_0")	# WitMotion libraries, make device object
	device.serialConfig.portName = '/dev/ttyAMA0'   						# Set serial port
	device.serialConfig.baud = 9600                 					# Set baud rate
	device.openDevice()                                 						# Open serial port

	last_update = time.perf_counter()

	while True:
		current_time = time.perf_counter()
		if current_time - last_update >= interval:
			last_update = current_time

			# Read GPS data
			gps.update()
			gps_data = [current_time, gps.latitude, gps.longitude, gps.altitude_m, gps.speed_kmh, gps.satellites]

			# Read IMU data
			imu_data = [current_time,
				device.getDeviceData("accX"),
				device.getDeviceData("accY"),
				device.getDeviceData("accZ"),
				device.getDeviceData("angleX"),
				device.getDeviceData("angleY"),
				device.getDeviceData("angleZ")]

			# Put data into queues
			gps_queue.put(gps_data)
			imu_queue.put(imu_data)

# Initialize ADS1015
i2c = busio.I2C(board.SCL, board.SDA)	# Declaring I2C object
adc = ADS.ADS1015(i2c)			# ADS1015 object
CONFIG_REGISTER = 0x01 			# Honestly idk ask zavier
util_func.set_continuous_mode(adc, CONFIG_REGISTER)	# Enable continuous mode
adc.data_rate = fast_sampling_freq	# Set ADS1015 to an appropriate sample rate (predetermined by hardware-see datasheet)
pot_channel1 = AnalogIn(adc, ADS.P0)	# Initialize channel in Single-Ended Mode
pot_channel2 = AnalogIn(adc, ADS.P1)	# Initialize channel 2 in Single-Ended Mode

# Housekeeping before recording data
slast_print = time.perf_counter()	# Start time for sampling
flast_print = slast_print

# Start GPS & IMU processing
gps_queue = multiprocessing.Queue()
imu_queue = multiprocessing.Queue()
gps_imu_proc = multiprocessing.Process(target=imu_gps_process, args=(gps_queue, imu_queue), daemon=True)
gps_imu_proc.start()

# Meat of recording and printing data
try:							# Try & except to give a way of ending loop someday 
	while True:					# While loop to continue checking sensors
		current = time.perf_counter()		# Check current time
		if current - flast_print >= interval2:
			raw_value1 = max(0, pot_channel1.value)	# Read ADC Values
			raw_value2 = max(0, pot_channel2.value)
			Pot1data.append([current, raw_value1])
			Pot2data.append([current, raw_value2])
			flast_print=current
		while not gps_queue.empty():
			GPSdata.append(gps_queue.get())
		while not imu_queue.empty():
			IMUdata.append(imu_queue.get())
except KeyboardInterrupt:	# Ctrl+C sends keyboard interupt and stops loop
	pass			# Does literally nothing but stop python from whining

# Close serial devices
gps_imu_proc.terminate()
gps_imu_proc.join(timeout=1)
#GPSser.close()			# Closes gps serial
#device.closeDevice()		# Closes IMU serial and stops thread - can take a few seconds don't freak out :)

# Write to file
util_func.csvWriteUSB(GPSdata, "GPS", ["Time", "Lat", "Long", "Alt", "Speed", "Sats"])				# GPS
util_func.csvWriteUSB(IMUdata, "IMU", ["Time", "AccX", "AccY", "AccZ", "AngleX", "AngleY", "AngleZ"])		# IMU
util_func.csvWriteUSB(Pot1data, "Pot1", ["Time", "Raw Value"])
util_func.csvWriteUSB(Pot2data, "Pot2", ["Time", "Raw Value"])

