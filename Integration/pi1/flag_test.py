import RPi.GPIO as GPIO
import time

# Setup
BUTTON_PIN = 27  # Replace with your GPIO pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Start the timer and prepare list
start_time = time.perf_counter()
press_times = []

# Callback for button press
def button_callback(channel):
    elapsed_time = time.perf_counter() - start_time
    press_times.append(elapsed_time)
    print(f"Button pressed at {elapsed_time:.4f} seconds")

# Add event detection
GPIO.add_event_detect(BUTTON_PIN, GPIO.RISING, callback=button_callback, bouncetime=200)

print("Tracking button presses (press CTRL+C to stop)...")
try:
    while True:
        time.sleep(0.0001)
except KeyboardInterrupt:
    print("\nButton press times (in seconds after start):")
    for t in press_times:
        print(f"{t:.4f}")
finally:
    GPIO.cleanup()
