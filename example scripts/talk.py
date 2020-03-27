from pycubed import cubesat
import time

cubesat.radio1.tx_power=23

count = 0
if cubesat.hardware['Radio1']:
    while True:
        count += 1
        print('Sending Message...'+str(count))
        cubesat.radio1.send('Hello World: '+str(count))
        time.sleep(2)