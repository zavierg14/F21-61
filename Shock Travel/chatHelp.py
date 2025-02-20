import time
import threading
import numpy as np
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from collections import deque

# Initialize I2C and ADS1015
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1015(i2c)
ads.data_rate = 3300  # Set to max sampling rate
chan = AnalogIn(ads, ADS.P0)

# Circular buffer to store ADC readings
BUFFER_SIZE = 3300  # Store 1 second of data
data_buffer = deque(maxlen=BUFFER_SIZE)

# Thread control
running = True

def read_ads1015():
    """Background thread to continuously read ADS1015 at 3,300 Hz."""
    global data_buffer
    sample_interval = 1.0 / 3300  # Time per sample (approx. 303 µs)
    
    while running:
        start_time = time.perf_counter()
        data_buffer.append(chan.value)  # Store latest reading
        elapsed_time = time.perf_counter() - start_time
        sleep_time = max(0, sample_interval - elapsed_time)  # Adjust timing
        time.sleep(sleep_time)

# Start the background thread
thread = threading.Thread(target=read_ads1015, daemon=True)
thread.start()

# Main loop to retrieve data in batches
try:
    while True:
        time.sleep(0.1)  # Retrieve data every 100 ms (10 Hz)
        
        # Convert buffer to NumPy array for processing
        with threading.Lock():
            data_snapshot = np.array(data_buffer)  

        print(f"Buffered {len(data_snapshot)} samples")
        # Process or save data as needed

except KeyboardInterrupt:
    running = False
    thread.join()
    print("Stopped sampling.")
