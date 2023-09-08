"""
Measures analog voltages with the internal SAMD51 16-bit analog-to-digital converter (ADC).

CircuitPython analogio documentation:
https://docs.circuitpython.org/en/latest/shared-bindings/analogio/index.html
"""
import board
import analogio


ain5 = analogio.AnalogIn(board.AIN5) # Designate pin here

raw_ain5 = ain5.value # Measure voltage at pin AIN5 (raw 16-bit value)
print(raw_ain5)

scaled_ain5 = (ain5.value * 3.3) / (2 ** 16) # See what raw value is scaled from 0v to 3.3V
print(scaled_ain5)