"""
`adm1176`
====================================================

CircuitPython driver for the adm1176 hot swap controller and I2C power monitor

* Author(s): Max Holliday

Implementation Notes
--------------------

"""

from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice


def _to_signed(num):
    if num > 0x7FFF:
        num -= 0x10000
    return num

DATA_V_MASK = const(0xF0)
DATA_I_MASK = const(0x0F)

class ADM1176:

    def __init__(self, i2c_bus, addr=0x4A):
        self.i2c_device = I2CDevice(i2c_bus, addr)
        self.i2c_addr = addr
        self.sense_resistor=1

        self.config('V_CONT,I_CONT')
    _cmd=bytearray(1)
    _cmd[0] = 0x00
    _extcmd=bytearray(2)
    _extcmd[0] = 0x83

    _BUFFER = bytearray(3)
    _STATUS = bytearray(1)
    def config(self, value):
        if 'V_CONT' in value:
            self._cmd[0] |= (1<<0)
        if 'V_ONCE' in value:
            self._cmd[0] |= (1<<1)
        if 'I_CONT' in value:
            self._cmd[0] |= (1<<2)
        if 'I_ONCE' in value:
            self._cmd[0] |= (1<<3)
        if 'VRANGE' in value:
            self._cmd[0] |= (1<<4)
        with self.i2c_device as i2c:
            i2c.write(self._cmd)

    def read(self):
        with self.i2c_device as i2c:
            i2c.readinto(self._BUFFER)
        raw_voltage = ((self._BUFFER[0]<<8) | (self._BUFFER[2]&DATA_V_MASK))>>4
        raw_current = (self._BUFFER[1]<<4) | (self._BUFFER[2]&DATA_I_MASK)
        _voltage = (26.35/4096)*raw_voltage # volts
        _current = ((0.10584/4096)*raw_current)/self.sense_resistor # amperes
        return (_voltage,_current)

    @property
    def OFF(self):
        self._extcmd[1]=1
        with self.i2c_device as i2c:
            i2c.write(self._extcmd)

    @property
    def ON(self):
        self._extcmd[1]=0
        with self.i2c_device as i2c:
            i2c.write(self._extcmd)
        self.config('V_CONT,I_CONT')