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
import RPi.GPIO as GPIO
# -----------------------------------------
# ADS1015 Setup
# -----------------------------------------
i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS.ADS1015(i2c)
CONFIG_REGISTER = 0x01
util_func.set_continuous_mode(adc, CONFIG_REGISTER)
adc.data_rate = 3300

channel1 = AnalogIn(adc, ADS.P0)
channel2 = AnalogIn(adc, ADS.P1)

# -----------------------------------------
# AS5600 Steering Sensor Setup
# -----------------------------------------
AS5600_ADDR = 0x36
RAW_ANGLE_REG = 0x0C

def read_angle():
    bus_i2c = smbus2.SMBus(1)
    data = bus_i2c.read_i2c_block_data(AS5600_ADDR, RAW_ANGLE_REG, 2)
    bus_i2c.close()
    raw_angle = (data[0] << 8) | data[1]
    angle = (raw_angle / 4096.0) * 360.0
    return angle
    
# -----------------------------------------
# CAN Setup
# -----------------------------------------
bus = can.Bus(channel='can0', bustype='socketcan', bitrate=500000)

def send_can_message(can_id, data):
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)
    
# -----------------------------------------
# GPIO Setup 
# -----------------------------------------
GPIO.setmode(GPIO.BCM)
START_SWITCH_PIN = 27
GPIO.setup(START_SWITCH_PIN, GPIO.IN)

# -----------------------------------------
# Sampling Intervals
# -----------------------------------------
pot_rate = 550
pot_interval = 1.0 / pot_rate

steering_rate = 60
steering_interval = 1.0 / steering_rate

try:
    print("Waiting for switch to toggle ON...")
    while True:
        # Wait for switch to be turned ON
        while GPIO.input(START_SWITCH_PIN) == GPIO.LOW:
            time.sleep(0.05)
        send_can_message(0x123, [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])


        print("Switch ON: Starting data acquisition...")

        # Reset buffers for this session
        pot1Data = []
        pot2Data = []
        steeringData = []

        start_time = time.perf_counter()
        pot_time = start_time
        steering_time = start_time

        # Acquire data until switch is turned OFF
        while GPIO.input(START_SWITCH_PIN) == GPIO.HIGH:
            current_time = time.perf_counter()

            if (current_time - pot_time) >= pot_interval:
                raw_value1 = max(0, channel1.value)
                raw_value2 = max(0, channel2.value)
                pot1Data.append([round(current_time - start_time, 6), raw_value1])
                pot2Data.append([round(current_time - start_time, 6), raw_value2])
                pot_time = current_time

            if (current_time - steering_time) >= steering_interval:
                angle = read_angle()
                steeringData.append([round(current_time - start_time, 6), angle])
                steering_time = current_time

        print("Switch OFF: Stopping and writing CSV logs...")
        send_can_message(0x123, [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        
        util_func.csvWriteUSB(pot1Data, "P1", ["Time", "Raw Val"])
        util_func.csvWriteUSB(pot2Data, "P2", ["Time", "Raw Val"])
        util_func.csvWriteUSB(steeringData, "Steering", ["Time", "Angle"])
        ##### SEND CAN_DATA not CSV

except KeyboardInterrupt:
    print("\nKeyboard interrupt received. Cleaning up GPIO...")
    GPIO.cleanup()
    print("Done.")
