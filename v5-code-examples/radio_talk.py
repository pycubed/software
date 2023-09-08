"""
Routine for the radio to send messages. 
"""
import time 
from lib.pycubed import cubesat    


# Safety check for antenna 
print("[IMPORTANT]: Verify that the antenna is attached before proceeding.")
antenna_attached = input("Press any key to continue: ")

if antenna_attached:
    if cubesat.hardware["Radio1"]:
        print("Sending messages...")
        message_count = 1 # Keep track of number of messages 

        while True:
            print(f"Sending message #{message_count}")
            cubesat.radio1.send(f"Hello world! #{message_count}")
            message_count += 1 # Increment message count
            time.sleep(2) # Delay 2 seconds before sending another message
    else:
        cubesat.RGB(255, 0, 0)
        print("[ERROR]: Could not connect to radio. Press Ctrl+C to exit.")
        while True:
            time.sleep(1)