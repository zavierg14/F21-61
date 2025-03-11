import lgpio
import time

# Global variable to hold pulse count
pulse_count = 0

def pulse_callback(chip, gpio, level, timestamp):
    """
    Callback function that gets called on the rising edge
    of the Hall sensor output. Increments the pulse counter.
    """
    global pulse_count
    if level == 1:  # Only count rising edges
        pulse_count += 1
        print(f"Pulse detected! Total pulses so far: {pulse_count}")

def main():
    CHIP = 0  # GPIO chip (default is 0 for Raspberry Pi)
    PIN = 17  # GPIO pin for the Hall sensor

    # Open GPIO chip
    h = lgpio.gpiochip_open(CHIP)
    
    # Set GPIO17 as an input with pull-down resistor
    lgpio.gpio_claim_input(h, PIN)
    
    # Set edge detection for rising pulses
    lgpio.gpio_set_edge_alert(h, PIN, lgpio.RISING_EDGE)
    
    # Register the callback function
    lgpio.callback(h, PIN, lgpio.RISING_EDGE, pulse_callback)
    
    print("Starting pulse counting. Press CTRL+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        lgpio.gpiochip_close(h)

if __name__ == "__main__":
    main()
