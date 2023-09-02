"""
Configure on-board general-purpose input/output (GPIO) pins.

CircuitPython digitalio documentation:
https://docs.circuitpython.org/en/latest/shared-bindings/digitalio/index.html
"""
import board 
import digitalio


gpio = digitalio.DigitalInOut(board.PB17) # Designate pin here
gpio.switch_to_output()
gpio.value = True # Set GPIO pin PB17 to 3.3V (logic-high)

gpio.switch_to_input()
print(gpio.value) # Read logic-high or logic-low conditions