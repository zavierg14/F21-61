# Kaleb Binger 3/2025
# F24-61 - Swap Hudson
# v9: Final Commenting

# -----------------------------------------
# Required Packages
# -----------------------------------------
import serial # Serial communication for GPS and IMU
import time												# Time tracking for sampling intervals
import adafruit_gps											# GPS module interface
import dependencies.chs.lib.device_model as deviceModel							# Actual local file w the deviceModel class
import dependencies.chs.JY901S as JY									# WitMotion IMU code
from dependencies.chs.lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor		# WitMotion IMU code
from dependencies.chs.lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver	# WitMotion IMU code
import dependencies.util_funcv2 as util_func								# Custom utility function
import board												# Board communication
import busio												# I2C communication for ADC 
import adafruit_ads1x15.ads1015 as ADS									# ADS1015 module interface
from adafruit_ads1x15.analog_in import AnalogIn								# ADS1015 module interface
import multiprocessing											# Multiprocessing for GPS and IMU
import lgpio												# lgpio for hall effect sensors
import gc												# Garbage collect for memory handling
import can												# CAN for communication with other pi
from RPLCD.i2c import CharLCD

# -----------------------------------------
# Configuration
# -----------------------------------------
slow_sampling_freq = 10 #Hz 						GPS & IMU sampling rate
interval = 1/slow_sampling_freq #s 					GPS & IMU interval
ADSinterval = 1/3300 #s 						ADS interval
last_print = time.perf_counter()					# Start time for sampling
can_update = last_print							# Start time for CAN
gc.disable()								# Disable python memory collect
bus = can.Bus(channel='can0', interface='socketcan', bitrate=500000)	# Turn on CAN
lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=20, rows=4)
lcd.clear()

# -----------------------------------------
# ADS1015 Setup
# -----------------------------------------
i2c = busio.I2C(board.SCL, board.SDA)			# Creating I2C object
adc = ADS.ADS1015(i2c)					# ADS1015 object
CONFIG_REGISTER = 0x01 					# Address of register to write to
util_func.set_continuous_mode(adc, CONFIG_REGISTER)	# Enable continuous mode - see util_funcv2
adc.data_rate = 3300					# Set ADS1015 to an appropriate sample rate (predetermined by hardware- see datasheet)

pot_channel1 = AnalogIn(adc, ADS.P0)			# Initialize channel 1 in Single-Ended Mode
pot_channel2 = AnalogIn(adc, ADS.P1)			# Initialize channel 2 in Single-Ended Mode


# -----------------------------------------
# Configure Hall Effect
# -----------------------------------------
CHIP = 0						# GPIO chip being used
PIN1 = 17						# GPIO PIN1
PIN2 = 27						# GPIO Pin2
h = lgpio.gpiochip_open(CHIP)				# Open the GPIO chip

lgpio.gpio_claim_input(h, PIN1)				# Configure PIN1 as input
lgpio.gpio_claim_alert(h, PIN1, lgpio.RISING_EDGE)	# Set up alert for rising edge detection

lgpio.gpio_claim_input(h, PIN2)				# Configure PIN2 as input
lgpio.gpio_claim_alert(h, PIN2, lgpio.RISING_EDGE)	# Set up alert for rising edge detection

pulse_count1 = 0					# Pulse counter 1 for hall effect
pulse_count2 = 0					# Pulse counter 2 for hall effect
hall_time = last_print
hall_interval = 1/20

def pulse_callback(chip, gpio, level, timestamp):
	'''
	callback function that gets called on the rising edge of the hall sensor output.
	increments pulse counter
	'''
	global pulse_count1, pulse_count2, Halldata						# Import external variables
	if level == 1:										# IF the signal is HIGH
		if gpio == PIN1:								# IF pin is 1
			pulse_count1 +=1							# Increment counter 1
		elif gpio == PIN2:								# IF pin is 2
			pulse_count2 += 1							# Increment counter 2

def stop_callback():
	'''
 	stop callbacks between switch flips
  	'''
	global hall1, hall2	# Import global hall variable
	hall1.cancel()		# Cancel hall1 callback
	hall2.cancel()		# Cancel hall2 callback

# -----------------------------------------
# GPS and IMU Collection
# -----------------------------------------

def imu_gps_process(gps_queue, imu_queue, lcd):
	'''
 	Runs GPS and IMU processing in a separate process to avoid slowing down
	other sampling
	'''
	# Initialize GPS
	global interval								# GPS and IMU interval time
	GPSserial_port = '/dev/ttyUSB0'						# GPS port USB
	GPSbaud_rate = 115200							# GPS baud rate
	GPSser = serial.Serial(GPSserial_port, GPSbaud_rate, timeout = .1)	# GPS serial object
	gps = adafruit_gps.GPS(GPSser, debug = False)				# GPS parsing object

	# Configure GPS
	gps.send_command(b"PMTK314,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")	# See adafruit documentation, telling GPS what data we want
	gps.send_command(b'PMTK220,100')						# See adafruit documentation, telling GPS how fast we want to update data

	# Initialize IMU
	device = deviceModel.DeviceModel("IMU",WitProtocolResolver(),JY901SDataProcessor(),"51_0")	# WitMotion libraries, make IMU device object
	device.serialConfig.portName = '/dev/ttyAMA0'   						# IMU serial port
	device.serialConfig.baud = 9600                 						# IMU baud rate
	device.openDevice()                                 						# Open serial port

	last_update = time.perf_counter()	# Start time for GPS
	lcd_update = last_update
	
	# Loop through data collection
	while True:	# Infinite loop!
		current_time = time.perf_counter()		# Variable for time now
		if current_time - last_update >= interval:	# IF time since last update is greater than sampling interval
			last_update = current_time		# Set last update to now

			# Read GPS data
			gps.update()							# Update GPS data
			gps_data = [current_time, gps.latitude, gps.longitude, 		# Retrieve GPS data
				    gps.altitude_m, gps.speed_kmh, gps.satellites]
			# Read IMU data
			imu_data = [current_time,					# Retrieve IMU data
				device.getDeviceData("accX"),
				device.getDeviceData("accY"),
				device.getDeviceData("accZ"),
				device.getDeviceData("angleX"),
				device.getDeviceData("angleY"),
				device.getDeviceData("angleZ"),
				device.getDeviceData("gyroX"),
				device.getDeviceData("gyroY"),
				device.getDeviceData("gyroZ"),
				device.getDeviceData("magX"),
				device.getDeviceData("magY"),
				device.getDeviceData("magZ")]
		if current_time - lcd_update >= 1:
			lcd.cursor_pos = (2,0)
			lcd.write_string("Speed (km/hr):")
			lcd.cursor_pos=(3,0)
			lcd.write_string(str(gps_data[4]))
			lcd_update = current_time

			# Put data into queues
			gps_queue.put(gps_data)		# GPS queue to get out of process
			imu_queue.put(imu_data)		# IMU queue to get out of process

printed = False
# -----------------------------------------
# MAIN
# -----------------------------------------
while True:	# Infinite loop for data acquisition
	try:	# Try to ensure an error doesn't break everything
		if printed != True:
			lcd.clear()
			lcd.cursor_pos = (0,0)
			lcd.write_string("System Initialized")
			lcd.cursor_pos = (1,0)
			lcd.write_string("Ready to Begin")
			printed = True

		msg = bus.recv()	# CAN recieve
		#print(msg)
		if (msg.arbitration_id == 0xE1 or msg.arbitration_id == 225) and msg.data[0] == 1:	# IF CAN ID is 0xE1 (power switch) and command is 1 (ON)
			lcd.clear()				# Print logging data
			lcd.cursor_pos = (0,0)
			lcd.write_string("System Logging")
			# Start GPS & IMU processing
			gps_queue = multiprocessing.Queue()										# Start GPS queue
			imu_queue = multiprocessing.Queue()										# Start IMU queue
			gps_imu_proc = multiprocessing.Process(target=imu_gps_process, args=(gps_queue, imu_queue, lcd), daemon=True)	# Create seperate process
			gps_imu_proc.start()												# Start process

			# Start hall Effects
			hall1 = lgpio.callback(h, PIN1, lgpio.RISING_EDGE, pulse_callback)	# Turn on hall 1 collection
			hall2 = lgpio.callback(h, PIN2, lgpio.RISING_EDGE, pulse_callback)	# Turn on hall 2 collection

			# Data Storage
			GPSdata = []		# GPS
			IMUdata = []		# IMU
			Potdata = []		# Rear Potentiometers
			Halldata = []		# Rear Hall Effects
			FPot1data = []		# Front Potentiometer 1
			FPot2data = []		# Front Potentiometer 2
			MagEncodedata = []	# Magnetic Encoder
			FHall1data = []		# Front Hall 1
			FHall2data = []		# Front Hall 2
			Rotarydata = []		# Rotary Encoder
			flags = []		# Flags

			#lcd.cursor_pos = (0,3)

			# Meat of recording and printing data
			while True:					# While loop to continually check sensors
				msg = bus.recv(timeout=0)		# Check CAN
				if msg != None and msg.arbitration_id == 0xE1 and msg.data[0] == 2:	# IF CAN ID is 0xE1 (power switch) and command is 2 (OFF)
					break								# Break loop, end data recording
				current = time.perf_counter()					# Check current time
				if current - last_print >= (1/600.0):				# Sample at 600Hz
					raw_value1 = max(0, pot_channel1.value)			# Read ADC1 Values
					raw_value2 = max(0, pot_channel2.value)			# Read ADC2 Values
					Potdata.append([current, raw_value1, raw_value2])	# Append potentiometer data
					last_print=current					# Set last print to now	
				if current - hall_time >= hall_interval:
					elapsed_time = current - hall_time
					pulses1 = pulse_count1
					pulses2 = pulse_count2
					rot1 = pulses1/16.0
					rot2 = pulses2/16.0
					frequency_hz1 = rot1 / elapsed_time
					frequency_hz2 = rot2 / elapsed_time
					Halldata.append([current, frequency_hz1, frequency_hz2])
					pulse_count1 = 0
					pulse_count2 = 0
					hall_time = current

				while not gps_queue.empty():			# IF GPS queue has data
					GPSdata.append(gps_queue.get())		# Append data to GPS data
				while not imu_queue.empty():			# IF IMU queue has data
					IMUdata.append(imu_queue.get())		# Append data to IMU data

			gps_imu_proc.terminate()
			gps_imu_proc.join(timeout=1)
			# Recieve Front Pi (Pi 1) data after recording
			lcd.clear()
			lcd.cursor_pot = (0,0)
			lcd.write_string("Begin Receive")						# Print Recieve begin
			while True:							# Infinite loop
				msg = bus.recv()					# Recieve CAN data
				timestamp, value = util_func.parse_can_data(msg.data)	# Put can data through parsing function - see util_funcv2

				# Front Potentiometer Left
				if msg.arbitration_id == 0xA1 or msg.arbitration_id == 161:			# IF CAN ID is 0xA1 (Potentiometer 1)
					FPot1data.append([timestamp, value])	# Append time and data to Front Potentiometer 1 data
				# Front Potentiometer Right
				elif msg.arbitration_id == 0xA2 or msg.arbitration_id == 162:		# IF CAN ID is 0xA2 (Potentiometer 2)
					FPot2data.append([timestamp, value])	# Append time and data to Front Potentiometer 2 data
				# Magnetic Encoder
				elif msg.arbitration_id == 0xB1 or msg.arbitration_id == 177:		# IF CAN ID is 0xB1 (Magnetic Encoder)
					MagEncodedata.append([timestamp, value/100.0])# Append time and data to Magnetic Encoder data
				# Front Hall Effect Left
				elif msg.arbitration_id == 0xC1 or msg.arbitration_id == 193:		# IF CAN ID is 0xC1 (Hall 1 )
					FHall1data.append([timestamp, value/100.0])	# Append time and data to Hall Effect 1 data
				# Front Hall Effect Right
				elif msg.arbitration_id == 0xC2 or msg.arbitration_id == 194:		# IF CAN ID is 0xC2 (Hall 2)
					FHall2data.append([timestamp, value/100.0])	# Append time and data to Hall Effect 2 data
				# Rotary Encoder
				elif msg.arbitration_id == 0xD1 or msg.arbitration_id == 209:		# IF CAN ID is 0xD1 (Rotary Encoder)
					if value == 0x7FFF or value == 32767:
						value = 0
					if value & 0x8000:
						value -=65536
						value = value * .1
					Rotarydata.append([timestamp, value])	# Append time and data to Rotary Encoder data
				# Flags
				elif msg.arbitration_id == 0xE2 or msg.arbitration_id == 226:		# IF CAN ID is 0xE2 (Flags)
					flags.append([timestamp])		# Append time to flags data
				# Switch
				elif (msg.arbitration_id == 0xE1 or msg.arbitration_id == 225) and msg.data[0] == 3:	# IF CAN ID is 0xE1 (Switch)
					print("break")					# Print break
					break						# Break loop
				elif (msg.arbitration_id == 0x2B0):
					pass
				else:					# Catch all if something breaks
					print("Broked")			# You should never see this print
					print(msg.arbitration_id)	# If you do it's all jacked up
					print(msg.data[0])		# Call/text me - 7206439097
					print(type(msg.arbitration_id))

			FPotdata = [[t1, d1, d2] for (t1, d1), (t2, d2) in zip(FPot1data, FPot2data) if t1 == t2]	# Combine Front Potentiometers into one list
			FHalldata = [[t1, d1, d2] for (t1, d1), (t2, d2) in zip(FHall1data, FHall2data) if t1 == t2]	# Combine Front Halls into one list
			lcd.clear()
			lcd.cursor_pos = (0,0)
			lcd.write_string("Data Finished")	# We done
			time.sleep(1.5)
			# Close Devices
			#gps_imu_proc.terminate()		# Kill seperate process
			#gps_imu_proc.join(timeout=1)		# Wait for it to join
			util_func.stop_callbacks(hall1, hall2)	# Stop hall effects

			# Write to file
			util_func.csvWriteUSB(GPSdata, "GPS", ["Time", "Lat", "Long", "Alt", "Speed", "Sats"])			# GPS
			util_func.csvWriteUSB(IMUdata, "IMU", ["Time", "AccX", "AccY", "AccZ", "AngleX", "AngleY", "AngleZ", "GyroX", "GyroY", "GyroZ", "MagX" ,"MagY", "MagZ"])	# IMU
			util_func.csvWriteUSB(Potdata, "RPot", ["Time", "RawValue1", "RawValue2"])				# Rear Pots
			util_func.csvWriteUSB(Halldata, "RHall", ["Time", "PulseCounts1", "PulseCounts2"])			# Rear Halls
			util_func.csvWriteUSB(FPotdata, "FPot", ["Time", "RawValue1", "RawValue2"])				# Front Pots
			util_func.csvWriteUSB(MagEncodedata, "MagEncode", ["Time", "Data"])					# Magnetic Encoder (Steering)
			util_func.csvWriteUSB(FHalldata, "FHall", ["Time", "Data1", "Data2"])					# Front Halls
			util_func.csvWriteUSB(Rotarydata, "Rotary", ["Time", "Data"])						# Rotary Encoder (5th wheel)
			util_func.csvWriteUSB(flags, "Flags", ["Time"])								# Flags
			gc.collect												# Take out the trash
			printed = False

	except KeyboardInterrupt:
		break

# -----------------------------------------
# End of Script Cleanup
# -----------------------------------------
#this shouldnt actually ever run but playing it safe ya know
bus.shutdown()
lgpio.gpiochip_close(h)
gc.collect()
