import RPi.GPIO as GPIO
import time

# Global variable to hold pulse count
pulse_count = 0

def pulse_callback(channel):
    """
    Callback function that gets called on the rising edge
    of the Hall sensor output. Increments the pulse counter.
    """
    global pulse_count
    pulse_count += 1
    print(f"Pulse detected! Total pulses so far: {pulse_count}")

def main():
    # Use BCM GPIO pin numbering
    GPIO.setmode(GPIO.BCM)
    
    # Set up GPIO17 as an input, with an internal pull-down resistor.
    # If your sensor is open-collector, you will need an external pull-up
    # resistor instead, and use pull_up_down=GPIO.PUD_OFF.
    GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    # Detect rising edges on GPIO17
    # bouncetime=20 helps debounce signals that might be noisy
    GPIO.add_event_detect(17, GPIO.RISING, callback=pulse_callback, bouncetime=20)
    
    print("Starting pulse counting. Press CTRL+C to stop.")
    try:
        # Keep the script running to catch interrupts
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        # Clean up GPIO on exit
        GPIO.cleanup()

if __name__ == "__main__":
    main()
