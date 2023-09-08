"""
Routine for the radio to listen for messages.  
"""
import time
from lib.pycubed import cubesat 


# Safety check for antenna 
print("[IMPORTANT]: Verify that the antenna is attached before proceeding.")
antenna_attached = input("Press any key to continue: ")

if antenna_attached:
    if cubesat.hardware["Radio1"]:
        print("Listening for messages...")
        while True:
            packet = cubesat.radio1.receive() # Capture data packet
            if packet is None:
                pass
            else:
                print(f"Received (raw bytes): {packet}")
                rssi = cubesat.radio1.rssi

                print(f"Received signal strength: {rssi}")
    else:
        cubesat.RGB(255, 0, 0)
        print("[ERROR]: Could not connect to radio. Press Ctrl+C to exit.")
        while True:
            time.sleep(1)