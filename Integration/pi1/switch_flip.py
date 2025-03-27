import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

START_SWITCH_PIN = 27
GPIO.setup(START_SWITCH_PIN, GPIO.IN)  # No pull-up/down needed ? you're supplying 3.3V

print("Waiting for switch to be flipped ON...")

# Wait for rising edge (LOW -> HIGH)
while GPIO.input(START_SWITCH_PIN) == GPIO.LOW:
    time.sleep(0.05)

print("Switch flipped. Starting data acquisition...")
