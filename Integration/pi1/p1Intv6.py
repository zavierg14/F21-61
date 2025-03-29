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
    raw_angle = raw_angle & 0x0FFF
    angle = (raw_angle / 4096.0) * 360.0
    return angle

# -----------------------------------------
# CAN Setup 
# -----------------------------------------
bus = can.Bus(channel='can0', interface='socketcan', bitrate=1000000)

def send_can_message(can_id, data):
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
    success = False
    while not success:
        try:
            bus.send(msg)
            success = True
        except can.CanError:
            print("CAN bus busy, retrying...")
            time.sleep(0.001)
    
def send_can_data(can_id, timestamp, value):
	time_bytes = int(timestamp * 1e6).to_bytes(4, 'big')
	val_bytes = value.to_bytes(4, 'big')
	can_data = time_bytes + val_bytes
	send_can_message(can_id, can_data)

def initialize_slip_sensor():
    print("Initializing slip sensor...")
    send_can_message(0x7C0, [0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    time.sleep(1)
    send_can_message(0x7C0, [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    print("Slip sensor calibration complete.")
    time.sleep(1)

def parse_slip_message(msg):
    raw = (msg.data[1] << 8) | msg.data[0]
    if raw == 0x7FFF:
        return None
    if raw & 0x8000:
        raw -= 65536
    return raw * 0.1

# -----------------------------------------
# GPIO Setup (Hall Effect Sensor & Switch)
# -----------------------------------------
pulse_count1 = 0
last_pulse_count1 = 0
pulse_count2 = 0
last_pulse_count2 = 0

def pulse_callback1(channel):
    global pulse_count1
    pulse_count1 += 1
def pulse_callback2(channel):
    global pulse_count2
    pulse_count2 += 1
def button_callback(channel):
    elapsed_time = round(time.perf_counter() - start_time, 6)
    press_times.append(elapsed_time)

GPIO.setmode(GPIO.BCM)
#Hall Effect 1
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
GPIO.add_event_detect(17, GPIO.RISING, callback=pulse_callback1, bouncetime=10)
#Hall Effect 2
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
GPIO.add_event_detect(27, GPIO.RISING, callback=pulse_callback2, bouncetime=10)
#Start Switch
START_SWITCH_PIN = 22
GPIO.setup(START_SWITCH_PIN, GPIO.IN)
#Flag Switch
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(23, GPIO.RISING, callback=button_callback, bouncetime=200)


# -----------------------------------------
# Sampling Intervals
# -----------------------------------------
pot_rate = 550
pot_interval = 1.0 / pot_rate

steering_rate = 60
steering_interval = 1.0 / steering_rate

slip_rate = 100
slip_interval = 1.0 / slip_rate

hall_rate = 10
hall_interval = 1.0 / hall_rate


# -----------------------------------------
# MAIN LOOP
# -----------------------------------------
try:
    print("Waiting for switch to toggle ON...")
    while True:
        # Wait for switch to be turned ON
        while GPIO.input(START_SWITCH_PIN) == GPIO.LOW:
            time.sleep(0.05)
            
        initialize_slip_sensor()
        send_can_message(0x123, [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        time.sleep(1)
        print("Switch ON: Starting data acquisition...")

        # Reset buffers for this session
        pot1Data = []
        pot2Data = []
        steeringData = []
        slipData = []
        hall1Data = []
        hall2Data = []
        press_times = []

        start_time = time.perf_counter()
        pot_time = start_time
        steering_time = start_time
        slip_time = start_time
        hall_time = start_time

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
                steeringData.append([round(current_time - start_time, 6), round(angle, 2)])
                steering_time = current_time
                
            if (current_time - slip_time) >= slip_interval:
                msg = bus.recv(timeout=0)
                if msg and msg.arbitration_id == 0x2B0:
                    parsed_angle = parse_slip_message(msg)
                    if parsed_angle is not None:
                        slipData.append([round(current_time - start_time, 6), round(parsed_angle, 1)])
                slip_time = current_time

            if (current_time - hall_time) >= hall_interval:
                elapsed_time = current_time - hall_time
                pulses1 = pulse_count1 - last_pulse_count1
                pulses2 = pulse_count2 - last_pulse_count2
                rot1 = pulses1/16.0
                rot2 = pulses2/16.0
                frequency_hz1 = rot1 / elapsed_time
                frequency_hz2 = rot2 / elapsed_time
                hall1Data.append([round(current_time - start_time, 6), round(frequency_hz1, 2)])
                hall2Data.append([round(current_time - start_time, 6), round(frequency_hz2, 2)])
                last_pulse_count1 = pulse_count1
                last_pulse_count2 = pulse_count2
                hall_time = current_time


        print("Switch OFF: Stopping and writing CSV logs...")
        send_can_message(0x123, [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        time.sleep(1)
        for data in pot1Data:
			send_can_data(0xA11, data[0], data[1])
		for data in pot2Data:
			send_can_data(0xA12, data[0], data[1])
		for data in steeringData:
			send_can_data(0xB11, data[0], int(data[1]*100))
        for data in hall1Data:
			send_can_data(0xC11, data[0], int(data[1]*100))
        for data in hall2Data:
			send_can_data(0xC12, data[0], int(data[1]*100))
		for data in press_times:
			send_can_message(0x321, int(data[0] * 1e6).to_bytes(4, 'big'))
		send_can_message(0x123, [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        util_func.csvWriteUSB(pot1Data, "P1", ["Time", "Raw Val"])
        util_func.csvWriteUSB(pot2Data, "P2", ["Time", "Raw Val"])
        util_func.csvWriteUSB(steeringData, "Steering", ["Time", "Angle"])


except KeyboardInterrupt:
    print("\nKeyboard interrupt received. Cleaning up GPIO...")
    GPIO.cleanup()
    print("Done.")

