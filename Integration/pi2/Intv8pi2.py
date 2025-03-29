#! ~/env/bin/python3
# Kaleb Binger 2/2025
# F21-61 - Swap Hudson
# v8: Optimizing for switch

#############################
# --- Required Packages --- #
#############################

import serial			# Serial to mess w sensors
import time			# Lets us check what time it is
import adafruit_gps		# Tells what the GPS is thinking
import datetime			# Bro idek but the dependencies need it
import dependencies.chs.lib.device_model as deviceModel	# Actual local file w the deviceModel class
import dependencies.chs.JY901S as JY			# WitMotion's actual file that I'm stealing functions from :)
from dependencies.chs.lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor	# WitMotion file to help w data parsing i think
from dependencies.chs.lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver	# ^^^^^
import dependencies.util_funcv2 as util_func	# My file :^) this is super useful
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import multiprocessing
import lgpio
import gc
import can

#########################
# --- Configuration --- #
#########################

# Sampling Frequency Setup
slow_sampling_freq = 10 #Hz (GPS & IMU sampling rate)
interval = 1/slow_sampling_freq #s GPS & IMU interval
ADSsampling_freq = 3300 #Hz (ADS1015 sampling rate)
ADSinterval = 1/ADSsampling_freq #s ADS interval
pulse_count1 = 0
pulse_count2 = 0

# Data Storage
GPSdata = []
IMUdata = []
Potdata = []
Halldata = []

# Configure Hall Effect
CHIP = 0
PIN1 = 17
PIN2 = 27
h = lgpio.gpiochip_open(CHIP)

lgpio.gpio_claim_input(h, PIN1)
lgpio.gpio_claim_alert(h, PIN1, lgpio.RISING_EDGE)

lgpio.gpio_claim_input(h, PIN2)
lgpio.gpio_claim_alert(h, PIN2, lgpio.RISING_EDGE)

# Initialize ADS1015
i2c = busio.I2C(board.SCL, board.SDA)	# Declaring I2C object
adc = ADS.ADS1015(i2c)			# ADS1015 object
CONFIG_REGISTER = 0x01 			# Honestly idk ask zavier
util_func.set_continuous_mode(adc, CONFIG_REGISTER)	# Enable continuous mode
adc.data_rate = ADSsampling_freq	# Set ADS1015 to an appropriate sample rate (predetermined by hardware-see datasheet)
pot_channel1 = AnalogIn(adc, ADS.P0)	# Initialize channel in Single-Ended Mode
pot_channel2 = AnalogIn(adc, ADS.P1)	# Initialize channel 2 in Single-Ended Mode

# Housekeeping before recording data
last_print = time.perf_counter()	# Start time for sampling
can_update = last_print
gc.disable()
bus = can.Bus(channel='can0', interface='socketcan', bitrate=1000000)

#####################
# --- Functions --- #
#####################

def pulse_callback(chip, gpio, level, timestamp):
	'''
	callback function that gets called on the rising edge of the hall sensor output.
	increments pulse counter
	'''
	global pulse_count1,pulse_count2, Halldata
	if level == 1:
		if gpio == PIN1:
			pulse_count1 +=1
			Halldata.append([time.perf_counter(), pulse_count1, pulse_count2])
		elif gpio == PIN2:
			pulse_count2 += 1
			Halldata.append([time.perf_counter(), pulse_count1, pulse_count2])

def imu_gps_process(gps_queue, imu_queue):
	'''
 	Runs GPS and IMU processing in a separate process to avoid slowing down
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
	GPSdata_local = []
	IMUdata_local = []

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

def stop_callback():
	global hall1, hall2
	hall1.cancel()
	hall2.cancel()

###########################
# --- Data Aquisition --- #
###########################
while True:
	try:
		msg = bus.recv()
		if msg.arbitration_id == 0xE1 and msg.data[0] == 1:
			print("Logging Data")
			# Start GPS & IMU processing
			gps_queue = multiprocessing.Queue()
			imu_queue = multiprocessing.Queue()
			gps_imu_proc = multiprocessing.Process(target=imu_gps_process, args=(gps_queue, imu_queue), daemon=True)
			gps_imu_proc.start()

			# Start hall Effects
			hall1 = lgpio.callback(h, PIN1, lgpio.RISING_EDGE, pulse_callback)
			hall2 = lgpio.callback(h, PIN2, lgpio.RISING_EDGE, pulse_callback)

			GPSdata = []
			IMUdata = []
			Potdata = []
			Halldata = []
			FPot1data = []
			FPot2data = []
			MagEncodedata = []
			FHall1data = []
			FHall2data = []
			Rotarydata = []
			flags = []

			# Meat of recording and printing data
			while True:				# While loop to continue checking sensors
				msg = bus.recv(timeout=0)
				#print(msg, type(msg))
				if msg != None and msg.arbitration_id == 0xE1 and msg.data[0] == 2:
					break
				current = time.perf_counter()		# Check current time
				if current - last_print >= (1/600.0):
					raw_value1 = max(0, pot_channel1.value)	# Read ADC Values
					raw_value2 = max(0, pot_channel2.value)
					Potdata.append([current, raw_value1, raw_value2])
					last_print=current
				while not gps_queue.empty():
					GPSdata.append(gps_queue.get())
#					print(GPSdata)
				while not imu_queue.empty():
					IMUdata.append(imu_queue.get())
			# Can Recieve
			print("Begin Receive")
			while True:
				msg = bus.recv()
				timestamp, value = util_func.parse_can_data(msg.data)
#				print(timestamp, value)
				if msg.arbitration_id == 0xA1:	# Potentiometer
					FPot1data.append([timestamp, value])
				elif msg.arbitration_id == 0xA2:	# Pot 2
					FPot2data.append([timestamp, value])
				elif msg.arbitration_id == 0xB1:	# magnetic encoder
					MagEncodedata.append([timestamp, value])
				elif msg.arbitration_id == 0xC1:
					FHall1data.append([timestamp, value])
				elif msg.arbitration_id == 0xC2:
					FHall2data.append([timestamp, value])
				elif msg.arbitration_id == 0xD1:
					Rotarydata.append([timestamp, value])
				elif msg.arbitration_id == 0xE2:
					flags.append([timestamp])
				elif msg.arbitration_id == 0xE1 and msg.data[0] == 3:
					print("break")
					break
				else:
					print("Broked")
					print(msg.arbitration_id)
					print(msg.data[0])
			FPotdata = [[t1, d1, d2] for (t1, d1), (t2, d2) in zip(FPot1data, FPot2data) if t1 == t2]
			FHalldata = [[t1, d1, d2] for (t1, d1), (t2, d2) in zip(FHall1data, FHall2data) if t1 == t2]
			print("data finished")

			# Close Devices
			gps_imu_proc.terminate()
			gps_imu_proc.join(timeout=1)
			util_func.stop_callbacks(hall1, hall2)

			# Write to file
			util_func.csvWriteUSB(GPSdata, "GPS", ["Time", "Lat", "Long", "Alt", "Speed", "Sats"])				# GPS
			util_func.csvWriteUSB(IMUdata, "IMU", ["Time", "AccX", "AccY", "AccZ", "AngleX", "AngleY", "AngleZ"])		# IMU
			util_func.csvWriteUSB(Potdata, "RPot", ["Time", "RawValue1", "RawValue2"])
			util_func.csvWriteUSB(Halldata, "Hall", ["Time", "PulseCounts1", "PulseCounts2"])
			util_func.csvWriteUSB(FPotdata, "FPot", ["Time", "RawValue1", "RawValue2"])
			util_func.csvWriteUSB(MagEncodedata, "MagEncode", ["Time", "Data"])
			util_func.csvWriteUSB(FHalldata, "FHall", ["Time", "Data1", "Data2"])
			util_func.csvWriteUSB(Rotarydata, "Rotary", ["Time", "Data"])
			util_func.csvWriteUSB(flags, "Flags", ["Time"])
			gc.collect


	except KeyboardInterrupt:
		break
#################################
# --- End of Script Cleanup --- #
#################################

bus.shutdown()
lgpio.gpiochip_close(h)
gc.collect()
