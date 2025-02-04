import time
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Create I2C bus and ADS1015 object
i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS.ADS1015(i2c)

def read_potentiometer():
    # Use AnalogIn to read the channel
    pot_channel = AnalogIn(adc, ADS.P0)  # Use P0 for channel A0
    
    # Get raw value and voltage
    raw_value = pot_channel.value
    voltage = pot_channel.voltage

    return raw_value, voltage

def main():
    print("Reading potentiometer value:")
    
    while True:
        raw_value, voltage = read_potentiometer()
        print(f"Raw Value: {raw_value}, Voltage: {voltage:.2f} V")
        #time.sleep(1)

if __name__ == "__main__":
    main()
