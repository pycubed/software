from pycubed import cubesat

if cubesat.hardware['Radio']:
    while True:
        packet = cubesat.radio.receive()
        if packet is None:
            pass
        else:
            print('Received (raw bytes): {0}'.format(packet))
            rssi = cubesat.radio.rssi
            print('Received signal strength: {0} dB'.format(rssi))