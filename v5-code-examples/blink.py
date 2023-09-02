"""
General blink routine that blinks the on-board neopixel LED green,
with 1 second delay between blinks. 
"""
import time 
from lib.pycubed import cubesat


print("Blinking neopixel...")
while True:
    cubesat.RGB = (0, 255, 0) # on
    time.sleep(1)
    cubesat.RGB = (0, 0, 0) # off 
    time.sleep(1)