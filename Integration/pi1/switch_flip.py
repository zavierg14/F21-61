import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
START_SWITCH_PIN = 27
GPIO.setup(START_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("Waiting for switch to start data acquisition...")
while GPIO.input(START_SWITCH_PIN):
    time.sleep(0.05)  # Debounce / Polling delay
print("Switch flipped. Starting data acquisition...")
