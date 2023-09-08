"""
Routine to configure on-board UART communication pins. 

CircuitPython UART documentation:
https://docs.circuitpython.org/en/latest/shared-bindings/busio/#busio.UART
"""
import time 
from lib.pycubed import cubesat

while True:
    # Check if there's anything in the UART buffer
    if cubesat.uart.in_waiting:
        data = cubesat.uart.read(32) # Read at most 32 bytes 

        # Data is a bytearray, want to convert it into a string for printing
        data_string = "".join([chr(b) for b in data])
        print(data_string)
    time.sleep(0.1)