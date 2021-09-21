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
_cmd=bytearray(1)
_extcmd=bytearray(b'\x00\x04')
_BUFFER = bytearray(3)
_STATUS = bytearray(1)

class ADM1176:

    def __init__(self, i2c_bus, addr=0x4A):
        self.i2c_device = I2CDevice(i2c_bus, addr, probe=False)
        self.i2c_addr = addr
        self.sense_resistor=1
        self.config('V_CONT,I_CONT')

    def config(self, value):
        _cmd[0]=0
        if 'V_CONT' in value:
            _cmd[0] |= (1<<0)
        if 'V_ONCE' in value:
            _cmd[0] |= (1<<1)
        if 'I_CONT' in value:
            _cmd[0] |= (1<<2)
        if 'I_ONCE' in value:
            _cmd[0] |= (1<<3)
        if 'VRANGE' in value:
            _cmd[0] |= (1<<4)
        with self.i2c_device as i2c:
            i2c.write(_cmd)

    def read(self):
        with self.i2c_device as i2c:
            i2c.readinto(_BUFFER)
        # [print(hex(i),end=',') for i in _BUFFER]
        raw_voltage = ((_BUFFER[0]<<8) | (_BUFFER[2]&DATA_V_MASK))>>4
        raw_current = (_BUFFER[1]<<4) | (_BUFFER[2]&DATA_I_MASK)
        _voltage = (26.35/4096)*raw_voltage # volts
        _current = ((0.10584/4096)*raw_current)/self.sense_resistor # amperes
        return (_voltage,_current)

    @property
    def OFF(self):
        _extcmd[0] = 0x83
        _extcmd[1]=1
        with self.i2c_device as i2c:
            i2c.write(_extcmd)

    @property
    def ON(self):
        _extcmd[0] = 0x83
        _extcmd[1]=0
        with self.i2c_device as i2c:
            i2c.write(_extcmd)
        self.config('V_CONT,I_CONT')

    @property
    def overcurrent_level(self,value=0xFF):
        # enable over current alert
        _extcmd[0] = 0x81
        _extcmd[1] |= (1<<1)
        with self.i2c_device as i2c:
            i2c.write(_extcmd)
        # set over current threshold
        _extcmd[0] = 0x82
        # set current threshold to value. def=FF which is ADC full scale
        _extcmd[1] = value
        with self.i2c_device as i2c:
            i2c.write(_extcmd)

    @property
    def clear(self):
        _extcmd[0] = 0x81
        temp=_extcmd[1]
        _extcmd[1] |= (1<<4)
        with self.i2c_device as i2c:
            i2c.write(_extcmd)
        _extcmd[1] = temp

    @property
    def status(self):
        _cmd[0] |= (1<<6)
        with self.i2c_device as i2c:
            i2c.write(_cmd)
            i2c.readinto(_STATUS)
        _cmd[0] &= ~(1<<6)
        with self.i2c_device as i2c:
            i2c.write(_cmd,stop=False)
        print(bin(_STATUS[0]))
        return _STATUS[0]
