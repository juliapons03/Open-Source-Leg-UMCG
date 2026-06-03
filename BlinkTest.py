"""
Simple LED Blink Script

Toggles the LED at /sys/class/leds/ACT/brightness every 1 second.
Handles Ctrl+C safely by turning the LED off before exiting.
"""
#1/use/bin/python3
import time

led_path = "/sys/class/leds/ACT/brightness"

try:
    while True:
        with open(led_path, 'w') as f:
            f.write("1")  # LED ON
        time.sleep(1)
        with open(led_path, 'w') as f:
            f.write("0")  # LED OFF
        time.sleep(1)
except KeyboardInterrupt:
    with open(led_path, 'w') as f:
        f.write("0")  # Turn LED off

