"""
PyCubed mainboardv05 deep-sleep power testing

* Make sure RBF jumpers are REMOVED (for best power consumption performance)
* No USB cable connected
* Powered via battery screw terminals (use adequate gauge wire)
* Bypass high-side and low-side inhibits (use adequate gauge wire)
"""

import alarm, time
from lib.pycubed import cubesat

print("Entering low power mode") 
cubesat.powermode("minimum")

# Blink RGB blue/green for ~5s to notify user
cubesat.neopixel.brightness=0.2
for _ in range(10):
    cubesat.RGB=(0,255,0)
gith    cubesat.RGB=(0,0,255)
    time.sleep(0.5)
# Turn RGB off
cubesat.RGB=(0,0,0)
cubesat.neopixel.brightness=0

# Set an alarm for 60 seconds from now.
time_alarm = alarm.time.TimeAlarm(monotonic_time=time.monotonic() + 60)

print("Deep sleeping for 60 seconds...")
# Deep sleep until the alarm goes off. Then restart the program.
alarm.exit_and_deep_sleep_until_alarms(time_alarm)