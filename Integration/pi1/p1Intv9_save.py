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
import gc

gc.disable()

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
    raw = (data[0] << 8) | data[1]
    raw &= 0x0FFF
    return (raw / 4096.0) * 360.0

# -----------------------------------------
# CAN Setup
# -----------------------------------------
bus = can.Bus(channel='can0', interface='socketcan', bitrate=1000000)

def send_can_message(can_id, data):
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    success = False
    while not success:
        try:
            bus.send(msg)
            success = True
        except can.CanError:
            print("trying again")
            time.sleep(0.001)


def send_can_data(can_id, timestamp, value):
    time_bytes = int(timestamp * 1e6).to_bytes(4, 'big')
    val_bytes = value.to_bytes(4, 'big')
    send_can_message(can_id, time_bytes + val_bytes)


def initialize_slip_sensor():
    send_can_message(0x7C0, [0x05] + [0x00]*7)
    time.sleep(1)
    send_can_message(0x7C0, [0x03] + [0x00]*7)
    time.sleep(1)

# -----------------------------------------
# Hall-effect instantaneous callbacks
# -----------------------------------------
last_time1 = None
last_time2 = None
hall1Data = []
hall2Data = []
start_time = None

def pulse_callback1(channel):
    global last_time1, hall1Data, start_time
    now = time.perf_counter()
    if last_time1 is not None:
        dt = now - last_time1
        freq = 1.0 / (dt * 16.0)
        ts = round(now - start_time, 6)
        hall1Data.append([ts, round(freq, 2)])
        print("freq:", freq)
    last_time1 = now

def pulse_callback2(channel):
    global last_time2, hall2Data, start_time
    now = time.perf_counter()
    if last_time2 is not None:
        dt = now - last_time2
        freq = 1.0 / (dt * 16.0)
        ts = round(now - start_time, 6)
        hall2Data.append([ts, round(freq, 2)])
        print("freq:", freq)
    last_time2 = now

# -----------------------------------------
# GPIO Setup (Hall Effect Sensor & Switch)
# -----------------------------------------
GPIO.setmode(GPIO.BCM)
# Hall pins: inputs only for now
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
# Start switch
START_SWITCH_PIN = 22
GPIO.setup(START_SWITCH_PIN, GPIO.IN)
# Flag button
press_times = []

def button_callback(channel):
    global press_times, start_time
    elapsed = round(time.perf_counter() - start_time, 6)
    press_times.append(elapsed)

GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(23, GPIO.BOTH, callback=button_callback, bouncetime=200)

# -----------------------------------------
# Sampling intervals
# -----------------------------------------
pot_rate      = 600
pot_interval  = 1.0 / pot_rate
steering_rate = 60
steering_interval = 1.0 / steering_rate
slip_rate     = 100
slip_interval = 1.0 / slip_rate

print("Start")
try:
    while True:
        # Wait for start switch ON
        while GPIO.input(START_SWITCH_PIN) == GPIO.LOW:
            time.sleep(0.05)

        # Enable Hall callbacks
        GPIO.add_event_detect(17, GPIO.RISING, callback=pulse_callback1, bouncetime=2)
        GPIO.add_event_detect(27, GPIO.RISING, callback=pulse_callback2, bouncetime=2)

        # Reset buffers
        pot1Data    = []
        pot2Data    = []
        steeringData = []
        slipData    = []
        hall1Data.clear()
        hall2Data.clear()
        press_times.clear()

        # Timing anchors
        start_time     = time.perf_counter()
        pot_time       = start_time
        steering_time  = start_time
        slip_time      = start_time
        last_time1 = start_time
        last_time2 = start_time
        # Init slip sensor & notify start
        initialize_slip_sensor()
        send_can_message(0xE1, [0x01] + [0x00]*7)
        time.sleep(1)

        # Acquisition loop
        while GPIO.input(START_SWITCH_PIN) == GPIO.HIGH:
            current_time = time.perf_counter()

            # Potentiometers
            if (current_time - pot_time) >= pot_interval:
                v1 = max(0, channel1.value)
                v2 = max(0, channel2.value)
                ts = round(current_time - start_time, 6)
                pot1Data.append([ts, v1])
                pot2Data.append([ts, v2])
                pot_time = current_time

            # Steering
            if (current_time - steering_time) >= steering_interval:
                angle = read_angle()
                ts = round(current_time - start_time, 6)
                steeringData.append([ts, round(angle, 2)])
                steering_time = current_time

            # Slip sensor
            if (current_time - slip_time) >= slip_interval:
                msg = bus.recv(timeout=0)
                if msg and msg.arbitration_id == 0x2B0:
                    raw = (msg.data[1] << 8) | msg.data[0]
                    ts = round(current_time - start_time, 6)
                    slipData.append([ts, raw])
                slip_time = current_time

        # Teardown: stop acquisition
        send_can_message(0xE1, [0x02] + [0x00]*7)
        time.sleep(1)

        # Disable Hall callbacks during CAN send
        GPIO.remove_event_detect(17)
        GPIO.remove_event_detect(27)

        # Send logged data over CAN
        for ts, v in pot1Data:      send_can_data(0xA1, ts, v)
        for ts, v in pot2Data:      send_can_data(0xA2, ts, v)
        for ts, a in steeringData:  send_can_data(0xB1, ts, int(a*100))
        for ts, f in hall1Data:     send_can_data(0xC1, ts, int(f*100))
        print(hall1Data)
        for ts, f in hall2Data:     send_can_data(0xC2, ts, int(f*100))
        print(hall2Data)
        for ts, s in slipData:      send_can_data(0xD1, ts, s)
        for t in press_times:       send_can_message(0xE2, int(t*1e6).to_bytes(4, 'big'))
        send_can_message(0xE1, [0x03] + [0x00]*7)

        # Prepare for next cycle: callbacks will be re-enabled at loop top
        gc.collect()
        print("Done")

except KeyboardInterrupt:
    print("\nKeyboard interrupt received. Cleaning up GPIO...")
    GPIO.cleanup()
    print("Done.")
