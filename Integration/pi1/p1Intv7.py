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
i2c = busio.I2C(board.SCL, board.SDA)						 #Use busio and board libraries to initialize i2c channel (ADDR = 0x48)
adc = ADS.ADS1015(i2c)								 #Use Adafruit ADS library to initialize ADC
CONFIG_REGISTER = 0x01								 #Set configuration register to write to ADC device
util_func.set_continuous_mode(adc, CONFIG_REGISTER)				 #Write to device and set mode
adc.data_rate = 3300								 #Set data rate

channel1 = AnalogIn(adc, ADS.P0)						 #Initialize analog channel1
channel2 = AnalogIn(adc, ADS.P1)						 #Initialize analog channel2

# -----------------------------------------
# AS5600 Steering Sensor Setup
# -----------------------------------------
AS5600_ADDR = 0x36								 #AS5600 i2c address, different from ADS i2c setup
RAW_ANGLE_REG = 0x0C								 #Register to read angle from device

def read_angle():
    bus_i2c = smbus2.SMBus(1)							 #Open i2c bus for each reading of AS5600
    data = bus_i2c.read_i2c_block_data(AS5600_ADDR, RAW_ANGLE_REG, 2)		 #Read data, points to register and pulls two bytes
    bus_i2c.close()								 #Close bus on each read
    raw_angle = (data[0] << 8) | data[1]					 #Bitshift data to readable format
    raw_angle = raw_angle & 0x0FFF						 #Only keep first 12-bits ignore extra
    angle = (raw_angle / 4096.0) * 360.0					 #Divide by 2^12 (register size) and multipy by 360 to get degress
    return angle

# -----------------------------------------
# CAN Setup
# -----------------------------------------
bus = can.Bus(channel='can0', interface='socketcan', bitrate=1000000)		 #Initialize CAN Bus with 1Mbps bitrate

def send_can_message(can_id, data):						 #Function sends hex data over CAN given 11-bit hex ID and hex data
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)	 #Create CAN message
    success = False								 #Message sent flag, true if sent false if retry is needed
    while not success:								 #Keep trying to send until successful
        try:
            bus.send(msg)							 #Send Message
            success = True							 #Flag = True if successfully sent
        except can.CanError:							 #If there is an error sending...
            time.sleep(0.001)							 #Sleep shortly then try again

def send_can_data(can_id, timestamp, value):					 #Funtion uses "send_can_message" to send sensor data over CAN Bus
    time_bytes = int(timestamp * 1e6).to_bytes(4, 'big')			 #Change timestamp from seconds to microseconds and turn into bytes
    val_bytes = value.to_bytes(4, 'big')					 #Turn data value into bytes
    can_data = time_bytes + val_bytes						 #Combine time and data into a message
    send_can_message(can_id, can_data)						 #Use previous funtion to send message with this function's data

def initialize_slip_sensor():							 #Funtion send CAN Bus messages to slip sensor to reset and set 0 position
    send_can_message(0x7C0, [0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])	 #Reset calibration status
    time.sleep(1)
    send_can_message(0x7C0, [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])	 #Set angle equal to 0
    time.sleep(1)

# -----------------------------------------
# GPIO Setup (Hall Effect Sensor & Switch)
# -----------------------------------------
pulse_count1 = 0								 #Global for counting total pulses on Hall1
last_pulse_count1 = 0								 #Global for saving last pulse count when hall data is not being logged
pulse_count2 = 0								 #Global for counting total pulsees on Hall2
last_pulse_count2 = 0								 #Global for saving last pulse count when data is not being logged

def pulse_callback1(channel):							 #Funtion for GPIO event, increments pulse_count1
    global pulse_count1
    pulse_count1 += 1
def pulse_callback2(channel):							 #Function for GPIO event, increments pulse_count2
    global pulse_count2
    pulse_count2 += 1
def button_callback(channel):							 #Funtion for GPIO event, appends timestamp to press_times (Flag button)
    elapsed_time = round(time.perf_counter() - start_time, 6)			 #Need to subract start time form current time to get elapsed time
    press_times.append(elapsed_time)						 #Append to list

GPIO.setmode(GPIO.BCM)								 #Set GPIO up
#Hall Effect 1
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_OFF)				 #Set GPIO of Hall1
GPIO.add_event_detect(17, GPIO.RISING, callback=pulse_callback1, bouncetime=10)	 #Set Hall1 event
#Hall Effect 2
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_OFF)				 #Set GPIO of Hall2
GPIO.add_event_detect(27, GPIO.RISING, callback=pulse_callback2, bouncetime=10)	 #Set Hall2 event
#Start Switch
START_SWITCH_PIN = 22								 #GPIO of start switch
GPIO.setup(START_SWITCH_PIN, GPIO.IN)						 #Set input from GPIO for switch
#Flag Switch
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)				 #Set GPIO of flag button
GPIO.add_event_detect(23, GPIO.RISING, callback=button_callback, bouncetime=200) #Add event for start switch to log press_times

# -----------------------------------------
# Sampling Intervals
# -----------------------------------------
pot_rate = 550									 #This is the frequency capture in Hz for potentiometers
pot_interval = 1.0 / pot_rate							 #This is the time between samples

steering_rate = 60
steering_interval = 1.0 / steering_rate

slip_rate = 100
slip_interval = 1.0 / slip_rate

hall_rate = 10
hall_interval = 1.0 / hall_rate

# -----------------------------------------
# MAIN LOOP
# -----------------------------------------
try:											 #We use this try loop in case you are troubleshooting in the terminal. Use ctrl+c to kill loop
    while True:
        while GPIO.input(START_SWITCH_PIN) == GPIO.LOW:					 #Wait for start switch to be turned on
            time.sleep(0.05)

        initialize_slip_sensor()							 #Initialize slip sensor on startup
        send_can_message(0xE1, [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])	 #Send CAN message to rear Pi to start data aquisition
        time.sleep(1)

        pot1Data = []									 # Reset buffers for each session, each list holds timestamp and data value
        pot2Data = []
        steeringData = []
        slipData = []
        hall1Data = []
        hall2Data = []
        press_times = []								 #This guy just holds timestamps

        start_time = time.perf_counter()						 #Allows us to use time elapsed and not time the Pi was turned on
        pot_time = start_time								 #This is the last time that each sensor was polled
        steering_time = start_time
        slip_time = start_time
        hall_time = start_time

        while GPIO.input(START_SWITCH_PIN) == GPIO.HIGH:
            current_time = time.perf_counter()							 #Sets current time on each loop

            if (current_time - pot_time) >= pot_interval:					 #If elepsed time is greater than potentiometer interval
                raw_value1 = max(0, channel1.value)						 #Read ADC channels
                raw_value2 = max(0, channel2.value)						 #Need to do for both potentiometers
                pot1Data.append([round(current_time - start_time, 6), raw_value1])		 #Append channel values and timestamp
                pot2Data.append([round(current_time - start_time, 6), raw_value2])
                pot_time = current_time								 #Set last time potentiometers were polled

            if (current_time - steering_time) >= steering_interval:				 #If elapsed time is greater than steering interval
                angle = read_angle()								 #Use read angle function from earlier
                steeringData.append([round(current_time - start_time, 6), round(angle, 2)])	 #Append data
                steering_time = current_time							 #Set last time steering sensor was polled

            if (current_time - hall_time) >= hall_interval:					 #If elapsed time is greater than hall interval
                elapsed_time = current_time - hall_time						 #Set time for frequency calc
                pulses1 = pulse_count1 - last_pulse_count1					 #Gather pulses from last polling
                pulses2 = pulse_count2 - last_pulse_count2
                rot1 = pulses1/16.0								 #Calculate wheel rotations from brake rotor "teeth"
                rot2 = pulses2/16.0
                frequency_hz1 = rot1 / elapsed_time						 #Wheel frequency = rotations/time
                frequency_hz2 = rot2 / elapsed_time
                hall1Data.append([round(current_time - start_time, 6), round(frequency_hz1, 2)]) #Append frequency data
                hall2Data.append([round(current_time - start_time, 6), round(frequency_hz2, 2)])
                last_pulse_count1 = pulse_count1						 #Set last pulse count for next polling
                last_pulse_count2 = pulse_count2
                hall_time = current_time							 #Set last time hall sensors where polled

            if (current_time - slip_time) >= slip_interval:
                msg = bus.recv(timeout=0)
                if (msg and msg.arbitration_id == 0x2B0):

        send_can_message(0xE1, [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        time.sleep(1)
        for data in pot1Data:
            send_can_data(0xA1, data[0], data[1])
        for data in pot2Data:
            send_can_data(0xA2, data[0], data[1])
        for data in steeringData:
            send_can_data(0xB1, data[0], int(data[1]*100))
        for data in hall1Data:
            send_can_data(0xC1, data[0], int(data[1]*100))
        for data in hall2Data:
            send_can_data(0xC2, data[0], int(data[1]*100))
        for data in slipData:
            send_can_data(0xD1, data[0], data[1])
        for data in press_times:
            send_can_message(0xE2, int(data[0] * 1e6).to_bytes(4, 'big'))
        send_can_message(0xE1, [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

except KeyboardInterrupt:
    print("\nKeyboard interrupt received. Cleaning up GPIO...")
    GPIO.cleanup()
    print("Done.")

