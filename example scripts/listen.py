from pycubed import cubesat

if cubesat.hardware['Radio1']:
    while True:
        packet = cubesat.radio1.receive()
        if packet is None:
            pass
        else:
            print('Received (raw bytes): {0}'.format(packet))
            rssi = cubesat.radio1.rssi
            print('Received signal strength: {0} dB'.format(rssi))