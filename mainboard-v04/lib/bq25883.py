"""
`bq25883`
====================================================

CircuitPython driver for the BQ25883 2-cell USB boost-mode charger.

* Author(s): Max Holliday

Implementation Notes
--------------------

"""

from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice

from adafruit_register.i2c_struct import ROUnaryStruct, UnaryStruct
from adafruit_register.i2c_bits import ROBits, RWBits
from adafruit_register.i2c_bit import ROBit, RWBit


# Registers
_BATV_LIM       = const(0x00)
_CHRGI_LIM      = const(0x01) 
_VIN_LIM        = const(0x02) 
_IIN_LIM        = const(0x03) 
_TERM_CTRL      = const(0x04) 
_CHRGR_CRTL1    = const(0x05) 
_CHRGR_CRTL2    = const(0x06) 
_CHRGR_CRTL3    = const(0x07) 
_CHRGR_CRTL4    = const(0x08) 
_OTG_CTRL       = const(0x09) 
_ICO_LIM        = const(0x0A) 
_CHRG_STAT1     = const(0x0B) 
_CHRG_STAT2     = const(0x0C) 
_NTC_STAT       = const(0x0D) 
_FAULT_STAT     = const(0x0E)
_CHRGR_FLAG1    = const(0x0F) 
_CHRGR_FLAG2    = const(0x10) 
_FAULT_FLAG     = const(0x11) 
_CHRGR_MSK1     = const(0x12) 
_CHRGR_MSK2     = const(0x13) 
_FAULT_MSK      = const(0x14) 
_ADC_CTRL       = const(0x15)
_ADC_FN_CTRL    = const(0x16) 
_IBUS_ADC1      = const(0x17) 
_IBUS_ADC0      = const(0x18) 
_ICHG_ADC1      = const(0x19) 
_ICHG_ADC0      = const(0x1A) 
_VBUS_ADC1      = const(0x1B) 
_VBUS_ADC0      = const(0x1C)
_VBAT_ADC1      = const(0x1D) 
_VBAT_ADC0      = const(0x1E) 
_VSYS_ADC1      = const(0x1F) 
_VSYS_ADC0      = const(0x20) 
_TS_ADC1        = const(0x21) 
_TS_ADC0        = const(0x22) 
_TDIE_ADC1      = const(0x23)
_TDIE_ADC0      = const(0x24)
_PART_INFO      = const(0x25)

def _to_signed(num):
    if num > 0x7FFF:
        num -= 0x10000
    return num

class BQ25883:
    _pn                  = ROBits(4,_PART_INFO,3,1,False)
    _fault_status        = ROBits(8,_FAULT_STAT,0,1,False)
    _chrgr_status1       = ROBits(8,_CHRG_STAT1,0,1,False)
    _chrgr_status2       = ROBits(8,_CHRG_STAT2,0,1,False)
    _chrg_status         = ROBits(3,_CHRG_STAT1,0,1,False)
    _otg_ctrl            = ROBits(8,_OTG_CTRL,0,1,False)
    _chrg_ctrl2          = ROBits(8,_CHRGR_CRTL2,0,1,False)
    _wdt                 = RWBits(2,_CHRGR_CRTL1,4,1,False)
    _ntc_stat            = RWBits(3,_NTC_STAT,0,1,False)
    _pfm_dis             =  RWBit(_CHRGR_CRTL3,7,1,False)
    _en_chrg             =  RWBit(_CHRGR_CRTL2, 3, 1, False)
    _reg_rst             =  RWBit(_PART_INFO, 7, 1, False)
    _stat_dis            =  RWBit(_CHRGR_CRTL1, 6, 1, False)
    _inlim               =  RWBit(_CHRGI_LIM, 6, 1, False)

    def __init__(self, i2c_bus, addr=0x6B):
        self.i2c_device = I2CDevice(i2c_bus, addr)
        self.i2c_addr = addr
        assert self._pn == 3, "Unable to find BQ25883"

    @property
    def status(self):
        print('Fault:',bin(self._fault_status))   
        print('Charger Status 1:',bin(self._chrgr_status1))
        print('Charger Status 2:',bin(self._chrgr_status2))
        print('Charge Status:',bin(self._chrg_status))  
        print('Charge Control2:',bin(self._chrg_ctrl2))  
        print('NTC Status:',bin(self._ntc_stat))  
        print('OTG:',hex(self._otg_ctrl))


    @property
    def charging(self):
        print('Charge Control2:',bin(self._chrg_ctrl2))    
    @charging.setter
    def charging(self,value):
        assert type(value) == bool
        self._en_chrg=value

    @property
    def wdt(self):
        print('Watchdog Timer:',bin(self._wdt))    
    @wdt.setter
    def wdt(self,value):
        if not value:
            self._wdt=0
        else:
            self._wdt=value

    @property
    def led(self):
        print('Status LED:',bin(self._stat_dis))    
    @led.setter
    def led(self,value):
        assert type(value) == bool
        self._stat_dis=not value