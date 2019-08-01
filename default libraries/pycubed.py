"""
CircuitPython driver for PyCubed satellite board

* Author(s): Max Holliday
"""
import adafruit_lsm9ds1
import adafruit_gps
import adafruit_sdcard
import adafruit_rfm9x
import board, microcontroller
import busio
import digitalio
import analogio
import storage, sys

class Satellite:
    def __init__(self):
        """
        Big init routine as the whole board is brought up. 
        """
        self.hardware = {
                       'IMU':    False,
                       'radio':  False,
                       'SDcard': False,
                       'GPS':    False,
                       'MRAM':   False,
                       'WDT':    False,
                       }

        # Define LEDs:
        self._led = digitalio.DigitalInOut(board.LED)
        self._led.switch_to_output()

        # Define burn wires:
        self._relayA = digitalio.DigitalInOut(board.RELAY_A)
        self._relayB = digitalio.DigitalInOut(board.RELAY_B)        
        self._relayA.switch_to_output()
        self._relayB.switch_to_output()
        self._deployA = False
        self._deployB = False  

        # Define battery voltage
        self._vbatt = analogio.AnalogIn(board.BATTERY)

        # Define battery charge current
        self._ichrg = analogio.AnalogIn(board.L1PROG)

        # Define battery current draw
        self._idraw = analogio.AnalogIn(board.IBATT)
        
        # Define SPI,I2C,UART
        self._i2c = busio.I2C(board.SCL,board.SDA)
        self._spi  = busio.SPI(board.SCK,MOSI=board.MOSI,MISO=board.MISO)
        self._uart = busio.UART(board.TX,board.RX)

        # Define MRAM (manual-mode)
        self._mram_cs = digitalio.DigitalInOut(microcontroller.pin.PB11)
        self._mram_cs.switch_to_output(value=True)

        # Define sdcard
        self._sdcs = digitalio.DigitalInOut(board.xSDCS)
        self._sdcs.switch_to_output(value=True)

        # Define radio
        self._rf_cs = digitalio.DigitalInOut(board.RF_CS)
        self._rf_rst = digitalio.DigitalInOut(board.RF_RST)
        self._rf_cs.switch_to_output(value=True)
        self._rf_rst.switch_to_output(value=True)

        # Define GPS
        self._en_gps = digitalio.DigitalInOut(board.EN_GPS)
        self._en_gps.switch_to_output()

        # Initialize sdcard
        try:
            self._sd   = adafruit_sdcard.SDCard(self._spi, self._sdcs)
            self._vfs = storage.VfsFat(self._sd)
            storage.mount(self._vfs, "/sd")
            sys.path.append("/sd")
            self.hardware['SDcard'] = True
        except Exception as e:
            print('[ERROR]',e)
        
        # Initialize IMU
        try:
            self.IMU = adafruit_lsm9ds1.LSM9DS1_I2C(self._i2c)
            self.hardware['IMU'] = True
        except Exception as e:
            print('[ERROR]',e)

        # Initialize radio
        try:
            self.radio = adafruit_rfm9x.RFM9x(self._spi, self._rf_cs, self._rf_rst, 433.0)
            self.hardware['Radio'] = True
        except Exception as e:
            print('[ERROR]',e)

        # Initialize GPS
        # try:
        #     self.GPS = adafruit_gps.GPS(self._uart)
        #     self.hardware['GPS'] = True
        # except Exception as e:
        #     print('[ERROR]',e)
        
    @property
    def acceleration(self):
        return self.IMU.acceleration

    @property
    def magnetic(self):
        return self.IMU.magnetic

    @property
    def gyro(self):
        return self.IMU.gyro

    @property
    def temperature(self):
        return self.IMU.temperature # Celsius 

    @property
    def temperature_cpu(self):
        return microcontroller.cpu.temperature # Celsius 

    @property
    def LED(self):
        return self._led.value

    @LED.setter
    def LED(self,value):
        self._led.value = value

    @property
    def battery_voltage(self):
        _voltage = self._vbatt.value * 3.3 / (2 ** 16)
        _voltage = _voltage * (316/110) # 316/110 voltage divider
        return _voltage # in volts
    @property
    def charge_current(self):
        _charge = self._ichrg.value * 3.3 / (2 ** 16)
        _charge = ((_charge*988)/6040)*1000 
        return _charge # in mA
    @property
    def current_draw(self):
        _draw = self._idraw.value * 3.3 / (2 ** 16)
        _draw = _draw*1000 
        return _draw # in mA
    @property
    def deploy(self):
        return (self._deployA,self._deployB)
    @deploy.setter
    def deploy(self,burnA,burnB,delay):
        if burnA:
            if not self._deployA:
                self._burn1 = digitalio.DigitalInOut(board.BURN1)
                self._burn2 = digitalio.DigitalInOut(board.BURN2)
                self._burn1.switch_to_output()
                self._burn2.switch_to_output()
                self._relayA.value = 1
                time.sleep(1)
                self._burn1.value  = 1
                self._burn2.value  = 1
                time.sleep(delay)
                self._burn1.value  = 0
                self._burn2.value  = 0
                self._relayA.value = 0
                self._deployA = True
        if burnB:
            if not self._deployB:
                self._burn3 = digitalio.DigitalInOut(board.BURN3)
                self._burn4 = digitalio.DigitalInOut(board.BURN4)
                self._burn5 = digitalio.DigitalInOut(board.BURN5)  
                self._burn3.switch_to_output()
                self._burn4.switch_to_output()
                self._burn5.switch_to_output()
                self._relayB.value = 1
                time.sleep(1)
                self._burn3.value  = 1
                self._burn4.value  = 1
                self._burn5.value  = 1
                time.sleep(delay)
                self._burn3.value  = 0
                self._burn4.value  = 0
                self._burn5.value  = 0
                self._relayB.value = 0
                self._deployB = True
        return (self._deployA,self._deployB)
         
cubesat = Satellite()