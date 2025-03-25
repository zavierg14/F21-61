import time
import csv
import datetime
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import dependencies.util_func as util_func
import smbus2
import can

#Global Variables

#Storage
pot1Data = []
pot2Data = []
steeringData = []
leftWheel = []
rightWheel = []
slipData = []

#  Setup for CAN bus
bustype = 'socketcan'
channel = 'can0'
bitrate = 500000
bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=bitrate)

#ADS
i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS.ADS1015(i2c)
CONFIG_REGISTER = 0x01  # Config Register Address from datasheet
util_func.set_continuous_mode(adc, CONFIG_REGISTER)
adc.data_rate = 3300

#Pot
channel1 = AnalogIn(adc, ADS.P0)
channel2 = AnalogIn(adc, ADS.P1)

# Steering
AS5600_ADDR = 0x36
RAW_ANGLE_REG = 0x0C
def read_angle():
    bus = smbus2.SMBus(1)
    data = bus.read_i2c_block_data(AS5600_ADDR, RAW_ANGLE_REG, 2)
    bus.close()
    raw_angle = (data[0] << 8) | data[1]
    angle = (raw_angle / 4096.0) * 360.0
    return angle

#CAN send
def send_can(can_id, timestamp, value):
	time_bytes = int(timestamp * 1e6).to_bytes(4, 'big')
	pot_bytes = value.to_bytes(4, 'big')
	can_msg = time_bytes + pot_bytes
	msg = can.Message(arbitration_id=can_id, data=can_data, is_extended_id=False)
	bus.send(msg)

#start time to start at 0
start_time = time.perf_counter()
#pot timing
pot_rate = 550 #Hz
pot_interval = 1.0/pot_rate
#steering timing
steering_rate = 60 #Hz
steering_interval = 1.0/steering_rate
#start timers
pot_time = time.perf_counter()
steering_time = pot_time

try:
	while True:
		current_time = time.perf_counter()
		#If sample interval is hit, record data
		if current_time - pot_time >= pot_interval:
			raw_value1 = max(0, channel1.value)	
			raw_value2 = max(0, channel2.value)
			pot1Data.append([round(current_time - start_time, 6), raw_value1])
			pot2Data.append([round(current_time - start_time, 6), raw_value2])
			pot_time = current_time
		if current_time - steering_time >= steering_interval:
			angle = read_angle()
			steeringData.append([round(current_time - start_time, 6), angle])
			steering_time = current_time	
except KeyboardInterrupt:
	print("\nSending CAN Msg...")
	
	for data in pot1Data:
		send_can(0x01, data[0], data[1])
	for data in pot2Data:
		send_can(0x02, data[0], data[1])
	for data in steeringData:
		send_can(0x03, data[0], data[1])
		 
		 

	









