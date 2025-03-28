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

# -----------------------------------------
# Global Storage Lists
# -----------------------------------------
pot1Data = []
pot2Data = []
steeringData = []
slipData = []

# -----------------------------------------
# ADS1015 Setup
# -----------------------------------------
i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS.ADS1015(i2c)
CONFIG_REGISTER = 0x01  # Config Register Address from datasheet
# writes the ADS1015 config for continuous sampling.
util_func.set_continuous_mode(adc, CONFIG_REGISTER)
adc.data_rate = 3300

channel1 = AnalogIn(adc, ADS.P0)
channel2 = AnalogIn(adc, ADS.P1)

# -----------------------------------------
# Steering via AS5600 (existing code)
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
# CAN Setup for Slip Sensor
# -----------------------------------------
bus = can.Bus(channel='can0', bustype='socketcan', bitrate=500000)

def send_can_message(can_id, data):
    """Send a CAN message with given ID and data bytes."""
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)

def initialize_slip_sensor():
    """
    Reset and calibrate the Bosch slip/steering sensor at startup,
    matching Slip_Test.py logic.
    """
    print("Initializing slip sensor...")
    # Reset calibration
    send_can_message(0x7C0, [0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    time.sleep(1)
    # Set Zero Position (calibrate)
    send_can_message(0x7C0, [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    print("Slip sensor calibration complete.")
    time.sleep(2)

# -----------------------------------------
# Example function to parse data from ID 0x2B0
# (Optional, if you want to interpret the bytes.)
# -----------------------------------------
def parse_slip_message(msg):
    """
    Convert the first two bytes (little-endian) into a signed 16-bit
    angle value with 0.1 deg/LSB resolution. Return None if invalid (0x7FFF).
    """
    raw = (msg.data[1] << 8) | msg.data[0]
    if raw == 0x7FFF:
        return None  # Sensor invalid or not calibrated
    # Convert to signed 16-bit
    if raw & 0x8000:
        raw -= 65536
    # Apply 0.1 deg/LSB
    angle_deg = raw * 0.1
    return angle_deg

# -----------------------------------------
# Main Loop Setup
# -----------------------------------------

# Sampling intervals
pot_rate = 550  # Hz
pot_interval = 1.0 / pot_rate

steering_rate = 60  # Hz
steering_interval = 1.0 / steering_rate

slip_rate = 100  # Hz
slip_interval = 1.0 / slip_rate

# Initialize the slip sensor before entering the loop
initialize_slip_sensor()

# Start time so that all time logs start at 0
start_time = time.perf_counter()

# Timers
pot_time = time.perf_counter()
steering_time = pot_time
slip_time = pot_time

try:
    while True:
        current_time = time.perf_counter()

		# -----------------------------------------
        # Pot sampling (existing)
		# -----------------------------------------
        if (current_time - pot_time) >= pot_interval:
            raw_value1 = max(0, channel1.value)
            raw_value2 = max(0, channel2.value)
            pot1Data.append([round(current_time - start_time, 6), raw_value1])
            pot2Data.append([round(current_time - start_time, 6), raw_value2])
            pot_time = current_time

		# -----------------------------------------
        # Steering sampling (AS5600)
		# -----------------------------------------
        if (current_time - steering_time) >= steering_interval:
            angle = read_angle()
            steeringData.append([round(current_time - start_time, 6), angle])
            steering_time = current_time

		# -----------------------------------------
        # Slip sensor sampling (CAN 0x2B0 @ 100 Hz)
		# -----------------------------------------
        #if (current_time - slip_time) >= slip_interval:
            #msg = bus.recv(timeout=1)  # non-blocking
            #if msg and msg.arbitration_id == 0x2B0:
                #parsed_angle = parse_slip_message(msg)
                #if parsed_angle is not None:
                    ## Valid angle
                    #slipData.append([round(current_time - start_time, 6),parsed_angle])
            #slip_time = current_time
        if (current_time - start_time) >= 0.01:
            msg = bus.recv(timeout=0)  # Wait for a CAN message
            if msg and msg.arbitration_id == 0x2B0:  # Only process LWS_Standard messages
                parsed_angle = parse_slip_message(msg)
                if parsed_angle is not None:
                    slipData.append([round(current_time - start_time, 6),round(parsed_angle, 1)])

except KeyboardInterrupt:
    print("\nStopping... Writing CSV logs to USB...")

    # Write pot data
    util_func.csvWriteUSB(pot1Data, "P1", ["Time", "Raw Val"])
    util_func.csvWriteUSB(pot2Data, "P2", ["Time", "Raw Val"])

    # Write steering data
    util_func.csvWriteUSB(steeringData, "Steering", ["Time", "Angle"])

    # Write slip data
    util_func.csvWriteUSB(slipData, "Slip", ["Time", "Angle"])

    print("Done.")
