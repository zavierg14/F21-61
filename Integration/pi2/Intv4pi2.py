import multiprocessing
import serial
import time
import adafruit_gps
import datetime
import dependencies.chs.lib.device_model as deviceModel
import dependencies.chs.JY901S as JY
from dependencies.chs.lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from dependencies.chs.lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver
import dependencies.util_func as util_func
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Sampling Frequency Stuff
slow_sampling_freq = 10  # Hz ---------- frequency we want
interval = 1 / slow_sampling_freq  # s -- time in seconds between samples
fast_sampling_freq = 2400  # Hz --------samples per second for ADS
interval2 = 1 / fast_sampling_freq  # S----time in seconds between ads samples

# Declaring GPS Port and Baud
GPSserial_port = "/dev/ttyUSB0"  # GPS serial port
GPSbaud_rate = 115200  # Default GPS baud rate
GPSdata = [[time.perf_counter(), 0, 0, 0, 0, 0]]  # Time/lat/long/altitude/speed/sat count

# Declaring IMU Port and baud
IMUserial_port = "/dev/ttyAMA0"  # IMU Serial port
IMUbaud_rate = 9600  # Default IMU baud rate
IMUdata = [[time.perf_counter(), 0, 0, 0, 0, 0, 0]]  # Time/accX/accY/accZ/angleX/AngleY/AngleZ

# Make GPS Serial object
GPSser = serial.Serial(GPSserial_port, GPSbaud_rate, timeout=.1)
GPSser.baudrate = GPSbaud_rate
gps = adafruit_gps.GPS(GPSser, debug=False)
gps.send_command(b"PMTK314,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
gps.send_command(b'PMTK220,100')

# Make IMU Serial Object
device = deviceModel.DeviceModel("IMU", WitProtocolResolver(), JY901SDataProcessor(), "51_0")
device.serialConfig.portName = IMUserial_port
device.serialConfig.baud = IMUbaud_rate
device.openDevice()

# Linear Potentiometer 1 configuration
i2c = busio.I2C(board.SCL, board.SDA)  # Declaring I2C object
adc = ADS.ADS1015(i2c)  # ADS1015 object
CONFIG_REGISTER = 0x01  # Honestly idk ask zavier
util_func.set_continuous_mode(adc, CONFIG_REGISTER)
adc.data_rate = fast_sampling_freq
pot_channel1 = AnalogIn(adc, ADS.P0)  # Initialize channel in Single-Ended Mode
Pot1data = [[time.perf_counter(), 0, 0]]  # Time/rawvalue/voltage

# Linear Potentiometer 2 configuration
pot_channel2 = AnalogIn(adc, ADS.P1)
Pot2data = [[time.perf_counter(), 0, 0]]

# Housekeeping before recording data
slast_print = time.perf_counter()  # Start time for sampling
flast_print = slast_print
start = slast_print


def adc_reader(queue):
    """Function to continuously read ADC values and push them into a queue"""
    while True:
        raw_value1 = pot_channel1.value
        raw_value2 = pot_channel2.value
        voltage1 = round(pot_channel1.voltage, 3)
        voltage2 = round(pot_channel2.voltage, 3)
        if raw_value1 < 0:
            raw_value1 = 0
            voltage1 = 0.00
        if raw_value2 < 0:
            raw_value2 = 0
            voltage2 = 0.00
        queue.put([time.perf_counter(), raw_value1, voltage1, raw_value2, voltage2])


# Start the ADC reader in a separate process
queue = multiprocessing.Queue()
adc_process = multiprocessing.Process(target=adc_reader, args=(queue,))
adc_process.start()

# Meat of recording and printing data
try:
    while True:
        gps.update()
        current = time.perf_counter()
        if current - slast_print >= interval:
            slast_print = current
            if not gps.has_fix:
                imutemp = [time.perf_counter(), device.getDeviceData("accx"), device.getDeviceData("accY"), device.getDeviceData("accZ"),
                           device.getDeviceData("angleX"), device.getDeviceData("angleY"), device.getDeviceData("angleZ")]
                continue
            util_func.deviceWrite(gps, device, False)
            imutemp = [time.perf_counter(), device.getDeviceData("accX"), device.getDeviceData("accY"), device.getDeviceData("accZ"),
                       device.getDeviceData("angleX"), device.getDeviceData("angleY"), device.getDeviceData("angleZ")]
            gpstemp = [time.perf_counter(), gps.latitude, gps.longitude, gps.altitude_m, gps.speed_kmh, gps.satellites]
            GPSdata.append(gpstemp)
            IMUdata.append(imutemp)

        if not queue.empty():
            pot_data = queue.get()
            Pot1data.append([pot_data[0], pot_data[1], pot_data[2]])
            Pot2data.append([pot_data[0], pot_data[3], pot_data[4]])

except KeyboardInterrupt:
    pass

# Close serial devices
GPSser.close()
device.closeDevice()

# Write to file
util_func.csvWriteUSB(GPSdata, "GPS", ["Time", "Lat", "Long", "Alt", "Speed", "Sats"])
util_func.csvWriteUSB(IMUdata, "IMU", ["Time", "AccX", "AccY", "AccZ", "AngleX", "AngleY", "AngleZ"])
util_func.csvWriteUSB(Pot1data, "Pot1", ["Time", "Raw Value", "Voltage"])
util_func.csvWriteUSB(Pot2data, "Pot2", ["Time", "Raw Value", "Voltage"])

# Terminate the ADC process
adc_process.terminate()
adc_process.join()
