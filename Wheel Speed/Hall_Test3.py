import RPi.GPIO as GPIO
import time

# Global variables
pulse_count = 0
last_pulse_count = 0

def pulse_callback(channel):
    """
    Callback function on the rising edge of the Hall sensor output.
    Increments the pulse counter.
    """
    global pulse_count
    pulse_count += 1

def main():
    GPIO.setmode(GPIO.BCM)
    
    # Set up GPIO17 as input
    GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    
    # Detect rising edges on GPIO17
    GPIO.add_event_detect(17, GPIO.RISING, callback=pulse_callback, bouncetime=20)
    
    print("Starting frequency measurement. Press CTRL+C to stop.")
    
    global last_pulse_count
    
    # Track time to measure pulses in one-second intervals
    last_time = time.time()
    
    try:
        while True:
            time.sleep(1)  # One-second measurement interval
            
            current_time = time.time()
            elapsed_time = current_time - last_time
            
            # Calculate how many pulses arrived in this one-second window
            pulses = pulse_count - last_pulse_count
            
            # Frequency in pulses per second
            frequency_hz = pulses / elapsed_time
            
            # If you want revolutions per second (RPS), divide by 16:
            # rps = frequency_hz / 16.0
            
            print(f"Frequency: {frequency_hz:.2f} Hz  (pulses/sec)", end='\r')
            
            # Update counters and time
            last_pulse_count = pulse_count
            last_time = current_time

    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()

