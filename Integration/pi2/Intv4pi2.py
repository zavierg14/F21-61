import multiprocessing
import time
import adafruit_ads1x15.ads1015 as ADS
import board
import busio

# Function for ADC reading
def read_adc(queue):
    # ADC setup (same as before)
    i2c = busio.I2C(board.SCL, board.SDA)
    adc = ADS.ADS1015(i2c)
    adc.data_rate = 2400
    pot_channel = ADS.P0  # Corrected to ADS.P0
    while True:
        raw_value = pot_channel.value
        voltage = round(pot_channel.voltage, 3)
        queue.put((raw_value, voltage))  # Put data in the queue
        time.sleep(1 / 2400)  # Adjust sleep to control sampling rate

# Function for processing or saving data
def process_data(queue):
    while True:
        data = queue.get()  # Get data from the queue
        raw_value, voltage = data
        print(f"Raw Value: {raw_value}, Voltage: {voltage}")

if __name__ == '__main__':
    # Create a Queue for inter-process communication
    queue = multiprocessing.Queue()

    # Create processes
    adc_process = multiprocessing.Process(target=read_adc, args=(queue,))
    processing_process = multiprocessing.Process(target=process_data, args=(queue,))

    # Start the processes
    adc_process.start()
    processing_process.start()

    # Wait for the processes to finish (optional)
    adc_process.join()
    processing_process.join()
