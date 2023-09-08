"""
Scans the given I^2C pins for responsive device(s) and returns their respective addresses.
"""
import time
from lib.pycubed import cubesat

 
while not cubesat.i2c1.try_lock():
    pass
 
while True:
    print("I2C addresses found:", [hex(device_add) for device_add in cubesat.i2c1.scan()])
    time.sleep(2)