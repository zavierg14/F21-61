import time
import board
import busio
import csv
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Create I2C bus and ADS1015 object
i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS.ADS1015(i2c)

# ADS1015 Configuration Register Address
CONFIG_REGISTER = 0x01  # Config Register Address from datasheet

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

# Set ADS1015 to an appropriate sample rate (e.g., 2400 SPS for stable sampling)
adc.data_rate = 2400

# Initialize channel in Single-Ended Mode
pot_channel = AnalogIn(adc, ADS.P0)

# List to store data
data_log = []

def save_to_csv(filename="potentiometer_data.csv"):
    """Saves collected data to a CSV file with timestamp column."""
    with open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp (s)", "Raw Value", "Voltage (V)"])  # Header
        writer.writerows(data_log)  # Write collected data
    print(f"\nData saved to {filename}")

def main():
    print("Reading potentiometer values. Press Ctrl+C to stop and save data.")

    sample_rate = 2400  # SPS
    sample_interval = 1.0 / sample_rate
    start_time = time.perf_counter()
    next_sample_time = start_time

    try:
        while True:
            # Wait until the next sampling time
            while time.perf_counter() < next_sample_time:
                pass  # Busy wait

            # Round timestamp to six decimal places
            timestamp = round(time.perf_counter() - start_time, 6)

            # Read ADC values
            raw_value = pot_channel.value
            voltage = round(pot_channel.voltage, 2)

            # Clamp negative values to zero if using single-ended mode
            if raw_value < 0:
                raw_value = 0
                voltage = 0.00

            print(f"Time: {timestamp:.6f}s | Raw Value: {raw_value}, Voltage: {voltage:.2f} V")
            data_log.append([timestamp, raw_value, voltage])

            next_sample_time += sample_interval

    except KeyboardInterrupt:
        print("\nStopping data collection...")
        save_to_csv()

if __name__ == "__main__":
    main()
