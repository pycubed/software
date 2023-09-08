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
from adafruit_register.i2c_bits import ROBits, RWBits
from adafruit_register.i2c_bit import ROBit, RWBit

# Registers
# _BATV_LIM       = const(0x00)#RW
_CHRGI_LIM      = const(0x01)#RW
# _VIN_LIM        = const(0x02)#RW
_IIN_LIM        = const(0x03)#RW
# _TERM_CTRL      = const(0x04)#RW
_CHRGR_CRTL1    = const(0x05)#RW
_CHRGR_CRTL2    = const(0x06)#RW
# _CHRGR_CRTL3    = const(0x07)#RW
# _CHRGR_CRTL4    = const(0x08)#RW
# _OTG_CTRL       = const(0x09)#RW
# _ICO_LIM        = const(0x0A)
# _CHRG_STAT1     = const(0x0B)
# _CHRG_STAT2     = const(0x0C)
# _NTC_STAT       = const(0x0D)
# _FAULT_STAT     = const(0x0E)
# _CHRGR_FLAG1    = const(0x0F)
# _CHRGR_FLAG2    = const(0x10)
# _FAULT_FLAG     = const(0x11)
# _CHRGR_MSK1     = const(0x12)#partial RW
# _CHRGR_MSK2     = const(0x13)#partial RW
# _FAULT_MSK      = const(0x14)#partial RW
# _ADC_CTRL       = const(0x15)#partial RW
# _ADC_FN_CTRL    = const(0x16)#partial RW

# _IBUS_ADC1      = const(0x17)
# _IBUS_ADC0      = const(0x18)

# _ICHG_ADC1      = const(0x19)
# _ICHG_ADC0      = const(0x1A)
# _VBUS_ADC1      = const(0x1B)
# _VBUS_ADC0      = const(0x1C)
# _VBAT_ADC1      = const(0x1D)
# _VBAT_ADC0      = const(0x1E)
# _VSYS_ADC1      = const(0x1F)
# _VSYS_ADC0      = const(0x20)
# _TS_ADC1        = const(0x21)
# _TS_ADC0        = const(0x22)
# _TDIE_ADC1      = const(0x23)
# _TDIE_ADC0      = const(0x24)

_PART_INFO      = const(0x25)#partial RW

class BQ25883:
    _pn                  = ROBits(4,_PART_INFO,3,1,False)
    # _fault_status        = ROBits(8,_FAULT_STAT,0,1,False)
    # _chrgr_status1       = ROBits(8,_CHRG_STAT1,0,1,False)
    # _chrgr_status2       = ROBits(8,_CHRG_STAT2,0,1,False)
    # _chrg_status         = ROBits(3,_CHRG_STAT1,0,1,False)
    # _otg_ctrl            = ROBits(8,_OTG_CTRL,0,1,False)
    _chrg_ctrl2          = ROBits(8,_CHRGR_CRTL2,0,1,False)
    # _ntc_stat            = ROBits(3,_NTC_STAT,0,1,False)
    # _ichrg_adc0          = ROBits(7,_ICHG_ADC0,0,1,False)
    # _ichrg_adc1          = ROBits(8,_ICHG_ADC1,0,1,False)
    # _ichrg_adc           = ROBits(16,_ICHG_ADC1,0,2,False)
    # _vbatt_adc           = ROBits(16,_VBAT_ADC1,0,2,False)
    # _vbatt_adc0          = ROBits(8,_VBAT_ADC0,0,1,False)
    # _vbatt_adc1          = ROBits(8,_VBAT_ADC1,0,1,False)

    # _vbatt_limit         = RWBits(8,_BATV_LIM,0,1,False)
    _wdt                 = RWBits(2,_CHRGR_CRTL1,4,1,False)
    _chrg_timer          = RWBits(2,_CHRGR_CRTL1,1,1,False)
    _ichrg               = RWBits(6,_CHRGI_LIM,0,1,False)
    _iinlim              = RWBits(5,_IIN_LIM,0,1,False)
    # _adc_res             = RWBits(2,_ADC_CTRL,4,1,False)

    # _pfm_dis             =  RWBit(_CHRGR_CRTL3,7,1,False)
    _en_chrg             =  RWBit(_CHRGR_CRTL2, 3, 1, False)
    # _reg_rst             =  RWBit(_PART_INFO, 7, 1, False)
    _stat_dis            =  RWBit(_CHRGR_CRTL1, 6, 1, False)

    # _en_ichrg_adc        =  RWBit(_ADC_FN_CTRL, 6, 1, False)
    # _en_ibus_adc         =  RWBit(_ADC_FN_CTRL, 7, 1, False)
    # _en_vbat_adc         =  RWBit(_ADC_FN_CTRL, 4, 1, False)
    # _en_adc              =  RWBit(_ADC_CTRL, 7, 1, False)
    # _adc_rate            =  RWBit(_ADC_CTRL, 6, 1, False)

    def __init__(self, i2c_bus, addr=0x6B):
        self.i2c_device = I2CDevice(i2c_bus, addr,probe=False)
        self.i2c_addr = addr
        if not self._pn == 3: print("Unable to find BQ25883")
        self._iinlim=0 # set input current limit to 500mA
        self._chrg_timer=0b00 # 5 hours
        # self.adc=False

    # @property
    # def status(self):
    #     print('Fault:',bin(self._fault_status))
    #     print('Charger Status 1:',bin(self._chrgr_status1))
    #     print('Charger Status 2:',bin(self._chrgr_status2))
    #     print('Charge Status:',bin(self._chrg_status))
    #     print('Charge Control2:',bin(self._chrg_ctrl2))
    #     print('NTC Status:',bin(self._ntc_stat))
    #     print('OTG:',hex(self._otg_ctrl))


    @property
    def charging(self):
        print('Charge Control2:',bin(self._chrg_ctrl2))
    @charging.setter
    def charging(self,value):
        assert type(value) == bool
        self._en_chrg=value

    @property
    def charging_current(self):
        print('Charger Current Limit (ICHRG):',hex(self._ichrg))
    @charging_current.setter
    def charging_current(self,value):
        # default:0x1e=1500mA, 0x8=400mA
        self._ichrg=value

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

    # def measure_current(self):
    #     return (self._ichrg_adc0,self._ichrg_adc1)

    # @property
    # def enable_adc(self):
    #     return self.adc

    # @enable_adc.setter
    # def enable_adc(self,value):
    #     self._en_adc=False
    #     if value:
    #         self._adc_rate=0b00
    #         self._en_ichrg_adc=True
    #         self._en_ibus_adc=True
    #         self._en_vbat_adc=True
    #         self._en_adc=True
    #         if not self._en_adc:
    #             print('error starting adc. ')
    #         self.adc=True
    #     else:
    #         self.adc=False



