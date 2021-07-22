'''
General rainbow routine for neopixel adapted from Adafruit Neopixel example.
Should work on all PyCubed boards with a "NEOPIXEL" pin defined in firmware.

M.Holliday
pycubed.org
'''
from pycubed import cubesat
import time

cubesat.neopixel.auto_write=False
# cubesat.neopixel.brightness=1

def wheel(pos):
    if pos < 0 or pos > 255:
        return (0, 0, 0)
    if pos < 85:
        return (255 - pos * 3, pos * 3, 0)
    if pos < 170:
        pos -= 85
        return (0, 255 - pos * 3, pos * 3)
    pos -= 170
    return (pos * 3, 0, 255 - pos * 3)

def rainbow_cycle(wait):
    for j in range(255):
        cubesat.RGB = wheel(j & 255)
        cubesat.neopixel.show()
        time.sleep(wait)

######################### MAIN LOOP ##############################
while True:
     rainbow_cycle(0.1) # change value to adjust speed
