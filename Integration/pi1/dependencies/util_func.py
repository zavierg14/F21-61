import time
import datetime
import csv
import os

def csvWrite(data, title, headers):
	with open(title+str(datetime.datetime.now().strftime('%Y%m%d%H%M%S'))+".csv", mode='w', newline='') as file:
		writer = csv.writer(file)
		writer.writerow(headers)
		writer.writerows(data)

def csvWriteUSB(data, title, headers):
	base_folder = get_usb_path() or os.getcwd()
	date_folder = os.path.join(base_folder, datetime.datetime.now().strftime('%Y-%m-%d-%H-%M'))
	os.makedirs(date_folder, exist_ok=True)

	filename = f"{title}{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}.csv"
	filepath = os.path.join(date_folder, filename)

	with open(filepath,mode = 'w', newline ='') as file:
		writer = csv.writer(file)
		writer.writerow(headers)
		writer.writerows(data)
	print(f"File saved: {filepath}")

def get_usb_path():
	base_path = "/media/admin"
	return next((os.path.join(base_path, d) for d in os.listdir(base_path) if os.path.ismount(os.path.join(base_path, d))), None)
	
def set_continuous_mode(adc, CONFIG_REGISTER):
	config_value = adc._read_register(CONFIG_REGISTER,2)	# Read current 16-bit configuration register value, returns an int
	config_value &= ~(1 << 8)	# Clear Bit 8 (MODE Bit) to enable continuous conversion mode
	adc._write_register(CONFIG_REGISTER, config_value)	# Write back modified configuration as a single 16-bit integer
