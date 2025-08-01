import RPi.GPIO as GPIO
import time

# GPIO pin where the water flow sensor is connected
FLOW_SENSOR_PIN = 18  

# Flow rate variables
pulse_count = 0
flow_rate = 0.0
calibration_factor = 7.5  # Adjust based on your sensor specs

def pulse_counter(channel):
    """Interrupt function to count pulses"""
    global pulse_count
    pulse_count += 1

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(FLOW_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(FLOW_SENSOR_PIN, GPIO.RISING, callback=pulse_counter)

try:
    while True:
        pulse_count = 0
        time.sleep(1)  # Measure for 1 second

        # Calculate flow rate (Liters per minute)
        flow_rate = (pulse_count / calibration_factor)

        print(f"Flow Rate: {flow_rate:.2f} L/min")
except KeyboardInterrupt:
    print("\nStopped by user")
    GPIO.cleanup()

