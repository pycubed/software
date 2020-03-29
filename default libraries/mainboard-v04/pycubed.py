"""
CircuitPython driver for PyCubed satellite board

PyCubed mainboard-v04

* Author(s): Max Holliday

TODO
- improve burn wire handling & syntax
- 

"""
import adafruit_gps
import adafruit_sdcard
import adafruit_rfm9x
import board, microcontroller
import busio, time
import digitalio
import analogio
import storage, sys
import pulseio, neopixel
import bq25883
import adm1176
import bmx160

class Satellite:
    def __init__(self):
        """
        Big init routine as the whole board is brought up. 
        """
        self.hardware = {
                       'IMU':    False,
                       'Radio1': False,
                       'Radio2': False,
                       'SDcard': False,
                       'GPS':    False,
                       'MRAM':   False,
                       'WDT':    False,
                       'USB':    False,
                       'PWR':    False,
                       }

        # Define burn wires:
        self._relayA = digitalio.DigitalInOut(board.RELAY_A)
        self._relayA.switch_to_output(drive_mode=digitalio.DriveMode.OPEN_DRAIN)
        self._deployA = False

        # Define battery voltage
        self._vbatt = analogio.AnalogIn(board.BATTERY)

        # Define MPPT charge current measurement
        self._ichrg = analogio.AnalogIn(board.L1PROG)
        
        # Define SPI,I2C,UART
        self.i2c  = busio.I2C(board.SCL,board.SDA)
        self.spi  = busio.SPI(board.SCK,MOSI=board.MOSI,MISO=board.MISO)
        self.uart = busio.UART(board.TX,board.RX, baudrate=9600, timeout=10)

        # Define GPS
        self.en_gps = digitalio.DigitalInOut(board.EN_GPS)
        self.en_gps.switch_to_output()

        # Define sdcard
        self._sdcs = digitalio.DigitalInOut(board.xSDCS)
        self._sdcs.switch_to_output(value=True)

        # Define radio
        self._rf_cs1 = digitalio.DigitalInOut(board.RF1_CS)
        self._rf_rst1 = digitalio.DigitalInOut(board.RF1_RST)
        self._rf_cs1.switch_to_output(value=True)
        self._rf_rst1.switch_to_output(value=True)
        self._rf_cs2 = digitalio.DigitalInOut(board.RF2_CS)
        self._rf_rst2 = digitalio.DigitalInOut(board.RF2_RST)
        self._rf_cs2.switch_to_output(value=True)
        self._rf_rst2.switch_to_output(value=True)

        # Define MRAM (manual-mode)
        # self._mram_cs = digitalio.DigitalInOut(microcontroller.pin.PB11)
        # self._mram_cs.switch_to_output(value=True)

        # Initialize Neopixel
        try:
            self.neopixel = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.2, pixel_order=neopixel.GRBW)
            self.neopixel[0] = (0,0,0)
            self.hardware['Neopixel'] = True
        except Exception as e:
            print('[WARNING][Neopixel]',e)

        # Initialize USB charger
        try: 
            self.usb = bq25883.BQ25883(self.i2c)
            self.usb.charging = False
            self.usb.wdt = False
            self.usb.led=False
            self.usb_charging=False
            self.hardware['USB'] = True
        except Exception as e:
            print('[ERROR][USB Charger]',e)        

        # Initialize sdcard
        try:
            self._sd   = adafruit_sdcard.SDCard(self.spi, self._sdcs)
            self._vfs = storage.VfsFat(self._sd)
            storage.mount(self._vfs, "/sd")
            sys.path.append("/sd")
            self.hardware['SDcard'] = True
        except Exception as e:
            print('[ERROR][SD Card]',e)
        
        # Initialize Power Monitor
        try:
            self.pwr = adm1176.ADM1176(self.i2c)
            self.pwr.sense_resistor = 1
            self.hardware['PWR'] = True
        except Exception as e:
            print('[ERROR][Power Monitor]',e)    

        # Initialize IMU
        try:
            self.IMU = bmx160.BMX160_I2C(self.i2c)
            self.hardware['IMU'] = True
        except Exception as e:
            print('[ERROR]',e)

        # Initialize radio(s)
        try:
            self.radio1 = adafruit_rfm9x.RFM9x(self.spi, self._rf_cs1, self._rf_rst1, 433.0)
            self.hardware['Radio1'] = True
        except Exception as e:
            print('[ERROR][RADIO 1]',e)
        try:
            self.radio2 = adafruit_rfm9x.RFM9x(self.spi, self._rf_cs2, self._rf_rst2, 433.0)
            self.hardware['Radio2'] = True
        except Exception as e:
            print('[ERROR][RADIO 2]',e)

        # Initialize GPS
        # try:
        #     self.GPS = adafruit_gps.GPS(self.uart)
        #     self.hardware['GPS'] = True
        # except Exception as e:
        #     print('[ERROR]',e)
        
    @property
    def acceleration(self):
        return self.IMU.accel

    @property
    def magnetic(self):
        return self.IMU.mag

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
    def RGB(self):
        return self.neopixel[0]
    @RGB.setter
    def RGB(self,value):
        if self.hardware['Neopixel']:
            try:
                self.neopixel[0] = value
            except Exception as e:
                print('[WARNING]',e)
    
    @property
    def charge_batteries(self):
        return self.usb_charging
    @charge_batteries.setter
    def charge_batteries(self,value):
        self.usb_charging=value
        self.usb.led=value
        self.usb.charging=value

    @property
    def battery_voltage(self):
        _voltage = self._vbatt.value * 3.3 / (2 ** 16)
        _voltage = _voltage * (316/110) # 316/110 voltage divider
        return _voltage # volts

    @property
    def system_voltage(self):
        if self.hardware['PWR']:
            try:
                return self.pwr.read()[0] # volts
            except Exception as e:
                print('[WARNING]',e)
        else:
            print('[WARNING] Power monitor not initialized')

    @property
    def current_draw(self):
        if self.hardware['PWR']:
            idraw=0
            try:
                for _ in range(50): # average 50 readings
                    idraw+=self.pwr.read()[1]
                return (idraw/50)*1000 # mA
            except Exception as e:
                print('[WARNING]',e)
        else:
            print('[WARNING] Power monitor not initialized')

    @property
    def charge_current(self):
        _charge = self._ichrg.value * 3.3 / (2 ** 16)
        _charge = ((_charge*988)/6040)*1000 
        return _charge # mA

    @property
    def reset_boot_count(self):
        microcontroller.nvm[0]=0

    @property
    def unique_file(self):
        import os
        if not self.hardware['SDcard']: 
            return False
        try:
            name = 'DATA_000'
            files = []
            for i in range(0,50):
                _filename = name[:-2]+str(int(i/10))+str(int(i%10))+'.txt'
                if _filename not in os.listdir('/sd/'):
                    with open('/sd/'+_filename, "a") as f:
                        time.sleep(0.01)
                    self.filename = '/sd/'+_filename
                    print('filename is:',self.filename)
                    return True
        except Exception as e:
            print('[ERROR] Unique File:', e)
            self.RGB = (255,0,0)
            return False

    def save(self, dataset, savefile=None):
        if savefile == None:
            savefile = self.filename
        try:
            with open(savefile, "a") as file:
                for item in dataset:
                    for i in item:
                        if isinstance(i,float):
                            file.write('{:.9E},'.format(i))
                        else:
                            file.write('{},'.format(i))
                    file.write('\n')
            return True
        except Exception as e:
            print('[ERROR] SD Save:', e)
            self.RGB = (255,0,0)
            return False

    # this deployment function is a placeholder
    def deploy(self,burnA=False,dutycycle=0,freq=5000,duration=1):
        print('BURNING with duty cycle of:',dutycycle)
        # if not self._deployA:
        burn = pulseio.PWMOut(board.PA22, frequency=freq, duty_cycle=0)
        self._relayA.drive_mode=digitalio.DriveMode.PUSH_PULL
        self._relayA.value = 1
        time.sleep(1)
        burn.duty_cycle=dutycycle
        time.sleep(duration)
        self._relayA.value = 0
        burn.duty_cycle=0
        self._deployA = True
        burn.deinit()
        self._relayA.drive_mode=digitalio.DriveMode.OPEN_DRAIN
        return self._deployA
         
cubesat = Satellite()