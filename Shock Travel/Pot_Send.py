import time
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import can

# -----------------------------
#  Setup for the ADS1015
# -----------------------------
# Create I2C bus and ADS1015 object
i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS.ADS1015(i2c)

# ADS1015 Configuration Register Address
CONFIG_REGISTER = 0x01  # From datasheet

def set_continuous_mode(adc):
    """Sets ADS1015 to Continuous Conversion Mode by modifying Config Register."""
    # Read current 16-bit configuration register value
    config_value = adc._read_register(CONFIG_REGISTER, 2)  # Returns an int

    # Clear Bit 8 (MODE Bit) to enable Continuous Conversion Mode
    config_value &= ~(1 << 8)

    # Write back modified configuration as a single 16-bit integer
    adc._write_register(CONFIG_REGISTER, config_value)

# Enable continuous mode
set_continuous_mode(adc)

# Set ADS1015 to an appropriate sample rate (e.g., 2400 SPS)
adc.data_rate = 2400

# Initialize channel
pot_channel = AnalogIn(adc, ADS.P0)

# -----------------------------
#  Setup for CAN bus
# -----------------------------
bustype = 'socketcan'
channel = 'can0'
bitrate = 500000
bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=bitrate)

# 12-bit address for the potentiometer (extended ID)
# Example: 0xABC (binary 1010_1011_1100 has 12 bits).
# We set is_extended_id=True so that 0xABC is used in the 29-bit extended-ID field.
POTENTIOMETER_CAN_ID = 0xABC

# -----------------------------
#  Main loop
# -----------------------------
def main():
    print("Reading potentiometer values and sending via CAN. Press Ctrl+C to stop.")

    sample_rate = 2400  # SPS
    sample_interval = 1.0 / sample_rate
    start_time = time.perf_counter()
    next_sample_time = start_time

    try:
        while True:
            # Wait until the next sampling time
            while time.perf_counter() < next_sample_time:
                pass  # Busy wait

            timestamp_sec = time.perf_counter() - start_time

            # Break the timestamp into minutes, seconds, microseconds
            total_seconds = int(timestamp_sec)
            minutes = total_seconds // 60   # 6 bits (0?63)
            seconds = total_seconds % 60    # 6 bits (0?59)
            microseconds = int((timestamp_sec - total_seconds) * 1_000_000)  # up to 999999

            # Make sure we clamp or wrap as needed for 6-bit minute/second fields
            minutes &= 0x3F
            seconds &= 0x3F
            microseconds &= 0xFFFFF  # 20 bits

            # Pack time into 4 bytes:
            #   bits 0?19: microseconds
            #   bits 20?25: seconds
            #   bits 26?31: minutes
            time_val = (microseconds & 0xFFFFF)
            time_val |= (seconds << 20)
            time_val |= (minutes << 26)

            # Read ADC value
            raw_value = pot_channel.value
            # Clamp negative if needed
            if raw_value < 0:
                raw_value = 0

            # The user wants 4 bytes for the ADC reading
            # Even if the ADC is only 12 bits, we'll store it in a 32-bit field.
            pot_bytes = raw_value.to_bytes(4, byteorder='big', signed=False)

            # Convert time_val to 4 bytes
            time_bytes = time_val.to_bytes(4, byteorder='big', signed=False)

            # Combine time (4 bytes) + pot (4 bytes)
            can_data = time_bytes + pot_bytes  # total of 8 bytes

            # Create the CAN message with an Extended ID
            msg = can.Message(
                arbitration_id=POTENTIOMETER_CAN_ID,
                data=can_data,
                is_extended_id=True
            )

            # Send the message
            try:
                bus.send(msg)
                # Optional debug print
                print(
                    f"Time: {timestamp_sec:.6f}s | raw: {raw_value} "
                    f"(m={minutes}, s={seconds}, us={microseconds}) -> CAN ID=0x{POTENTIOMETER_CAN_ID:X}, data={can_data}"
                )
            except can.CanError:
                print("CAN message NOT sent")

            next_sample_time += sample_interval

    except KeyboardInterrupt:
        print("\nStopping data collection...")

if __name__ == "__main__":
    main()
