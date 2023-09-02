"""
Routine that checks electrical characteristics of PyCubed.
"""
from lib.pycubed import cubesat

print(f"Battery pack voltage: {cubesat.battery_voltage}V")
print(f"Battery charing current (solar): {cubesat.charge_current}mA")
print(f"System voltage: {cubesat.system_voltage}V")
print(f"System current draw: {cubesat.current_draw}mA")

