import time		# idek if i use this lol
import datetime		# Used to name file
import adafruit_gps	# ~Parses the GPS~
import csv
import os
import lgpio

# EZPZ function to write list data to a csv file
def csvWrite(data, title, headers):
	with open(title+str(datetime.datetime.now().strftime('%Y%m%d%H%M%S'))+".csv", mode='w', newline='') as file:
		writer = csv.writer(file)
		writer.writerow(headers)
		writer.writerows(data)

def csvWriteUSB(data, title, headers):
	base_folder = get_usb_path() or os.getcwd()
	date_folder = os.path.join(base_folder, datetime.datetime.now().strftime('%Y-%m-%d-%H-%M'))
	os.makedirs(date_folder, exist_ok=True)

	filename = f"{title}.csv"
	filepath = os.path.join(date_folder, filename)

	with open(filepath,mode = 'w', newline ='') as file:
		writer = csv.writer(file)
		writer.writerow(headers)
		writer.writerows(data)
	print(f"File saved: {filepath}")

# Sets ADS1015 to continuous conversion mode by modifying config register
def set_continuous_mode(adc, CONFIG_REGISTER):
	config_value = adc._read_register(CONFIG_REGISTER,2)	# Read current 16-bit configuration register value, returns an int
	config_value &= ~(1 << 8)	# Clear Bit 8 (MODE Bit) to enable continuous conversion mode
	adc._write_register(CONFIG_REGISTER, config_value)	# Write back modified configuration as a single 16-bit integer

def get_usb_path():
	base_path = "/media/admin"
	return next((os.path.join(base_path, d) for d in os.listdir(base_path) if os.path.ismount(os.path.join(base_path, d))), None)

def read_ads1015_raw(i2c, CONVERSION_REGISTER):
	raw_value = adc._read_register(CONVERSION_REGISTER, 2)	# Read 2 bytes (16-bit value)
	return raw_value >> 4	# ADS1015 outputs 12-bit data, so shift right 4 bits

def stop_callbacks(hall1, hall2):
	hall1.cancel()
	hall2.cancel()

def parse_can_data(data):
	timestamp = int.from_bytes(data[0:4], 'big') / 1e6
	value = int.from_bytes(data[4:8], 'big')
	return timestamp, value



# Drive safe
# Soli deo gloria
#    (()__(()
#    /       \
#   ( /    \  \
#    \ o o    /
#    (_()_)__/ \
#    / _,==.____ \
#   (   |--|      )
#   /\_.|__|'-.__/\_
#  / (        /     \
#  \  \      (      /
#   )  '._____)    /
#(((____.--(((____/

