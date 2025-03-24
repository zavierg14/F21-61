import time
import csv
import datetime
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import dependencies.util_func as util.func

#Global Variables

#Storage
Pot1data = []
Pot2data = []
steeringData = []
leftWheel = []
rightWheel = []

#ADS
i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS.ADS1015(i2c)
CONFIG_REGISTER = 0x01  # Config Register Address from datasheet
util_func.set_continuous_mode(adc, CONFIG_REGISTER)
adc.data_rate = 3300

#Pot
channel1 = AnalogIn(adc, ADS.P0)
channel2 = AnalogIn(adc, ADS.P1)

def pot_save(filename, datalog)
	with open(filename, mode="w", newline="") as file:
		writer = csv.writer(file)
        writer.writerow(["Timestamp (s)", "Raw Value",])  # Header
        writer.writerows(data_log)  # Write collected data
    print(f"\nData saved to {filename}")

def wheel_save(filename, datalog)
	with open(filename, mode="w", newline="") as file:
		writer = csv.writer(file)
        writer.writerow(["Timestamp (s)", "Freq",])  # Header
        writer.writerows(data_log)  # Write collected data
    print(f"\nData saved to {filename}")

def steering_save(filename, datalog)
	with open(filename, mode="w", newline="") as file:
		writer = csv.writer(file)
        writer.writerow(["Timestamp (s)", "Angle",])  # Header
        writer.writerows(data_log)  # Write collected data
    print(f"\nData saved to {filename}")

def main():
	pot_rate = 550
	pot_inerval = 1.0/pot_rate
	last_time = time.perf_counter()


	try:
		while True:
			current_time = time.perf_counter()
			if current_time - last_time >= pot_interval:
				raw_value1 = max(0, pot_channel1.value)	# Read ADC Values
				raw_value2 = max(0, pot_channel2.value)
				Pot1data.append([current, raw_value1])
				Pot2data.append([current, raw_value2])
				last_time = current_time
				
	except:






