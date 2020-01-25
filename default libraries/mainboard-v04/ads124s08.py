from time import sleep, monotonic
from adafruit_bus_device import spi_device
from math import sqrt

default = bytes([0x41,0x09,0x00,0xCC,0x08,0x98,0x3A,0x00,0xFF,0x00,0x18])
registers = [
                    "(ID):        ",
                    "(STATUS):    ",
                    "(INPMUX):    ",
                    "(PGA):       ",
                    "(DATARATE):  ",
                    "(REF):       ",
                    "(IDACMAG):   ",
                    "(IDACMUX):   ",
                    "(VBIAS):     ",
                    "(SYS):       ",
                    "(OFCAL0):    ",
                    "(OFCAL1):    ",
                    "(OFCAL2):    ",
                    "(FSCAL0):    ",
                    "(FSCAL1):    ",
                    "(FSCAL2):    ",
                    "(GPIODAT):   ",
                    "(GPIOCON):   "]




class _ADS124S08:
    
    _registers = bytearray(18)

    def __init__(self, refV, pgaGain=1, drdy_pin=0):
        self.DRDY = False
        self._refV=refV # default refV = 2.5V
        # Setup DRDY pin.
        if drdy_pin:
            self.DRDY = True
            self._drdy= drdy_pin
        self.pgaGain = pgaGain
        self._LSBSIZE()
        self._init_adc()
        self.startTrig = True
        self._status=bytearray(1)

    def _LSBSIZE(self):
        pgaGain = self.pgaGain
        val = self._refV/(pgaGain*pow(2,23))
        self.LSBsize = val
        return val

    def _init_adc(self):
        self.reset()
        self.start()
        self.regreadout()

    def readValue(self):
        zzz = bytearray(3)
        with self.spi_device as spi:
            sleep(2.5e-8) 
            spi.write(bytes([0x12]))
            spi.readinto(zzz)
            return self.dataconvert(zzz)
    
    def readValue_drdy(self):
        readbuf = bytearray(3)
        _cnt=0
        while self._drdy.value is True: # wait until DRDY goes low
            # _cnt+=1
            # if _cnt > 10000:
            #     print('DRDY TIMEOUT')
            #     return 0
            # else:
            #     pass
            pass
        with self.spi_device as spi:
            sleep(4e-7)  # wait 4*tclk        
            spi.readinto(readbuf) 
        readData = self.dataconvert(readbuf)
        return readData

    def reset(self):
        with self.spi_device as spi:
            spi.write(bytes(0x0A))
            sleep(0.1)
            spi.write(default)

    def start(self):
        with self.spi_device as spi:
            spi.write(bytes([0x0A, 0x08]))
        sleep(0.1)
    
    def stop(self):
        with self.spi_device as spi:
            spi.write(bytes(0x0A))

    def wake(self):
        with self.spi_device as spi:
            spi.write(bytes(0x02))

    def wreg(self,start,cmd): #xtb.wreg(0x42,[0xCC])
        length = len(cmd) - 1
        with self.spi_device as spi:
            spi.write(bytes([start]+[length]+cmd))
    
    def regreadout(self):
        
        with self.spi_device as spi:
            spi.write(bytes([0x20, 0x12]))
            spi.readinto(self._registers)
        
        assert self._registers[0] == 8

        print("-"*31)
        for i, j in enumerate(self._registers):
            print(registers[i], hex(j))
        print("-"*31)
        return self._registers
    
    def GPIO(self, GPIODAT, GPIOCON):
        with self.spi_device as spi:
            spi.write(bytes([0x50, 0x01, GPIODAT, GPIOCON]))
    
    def status(self,debug=False):
        with self.spi_device as spi:
            spi.write(bytes([0x21, 0x01]))
            spi.readinto(self._status)
        status_byte = {'_POR'    :self._status[0]>>7&1,
                       '_RDY'    :self._status[0]>>6&1,
                       '_P_RAILP':self._status[0]>>5&1,
                       '_P_RAILN':self._status[0]>>4&1,
                       '_N_RAILP':self._status[0]>>3&1,
                       '_N_RAILP':self._status[0]>>2&1,
                       '_REFL1'  :self._status[0]>>1&1,
                       '_REFL0'  :self._status[0]>>0&1}
        if debug: [print(k,v) for k,v in status_byte.items()]
        return self._status[0]

    def calibrate(self):
        with self.spi_device as spi:
            spi.write(bytes([0x19]))

    def dataconvert(self, raw):
        self._LSBSIZE()
        rawDataIN = 0 | raw[0]
        rawDataIN = (rawDataIN << 8) | raw[1]
        rawDataIN = (rawDataIN << 8) | raw[2]
        if (((1 << 23) & rawDataIN) != 0):
        # if (rawDataIN > 8.38861e6):
            dataOUT = (((~rawDataIN) & ((1 << 24) - 1)) + 1) * self.LSBsize * -1
        else:
            dataOUT = rawDataIN*self.LSBsize
        return dataOUT

    def temperature(self,ref=0x39):
        _tbuff=0
        with self.spi_device as spi:
            spi.write(bytes([0x20, 0x12]))
            spi.readinto(self._registers)
        with self.spi_device as spi:
            spi.write(bytes([0x42, 0x07, 0xCC, 0x00, 0x12, ref, 0x00, 0xFF, 0x38, 0x58]))
        for _ in range(5):
            _tbuff += self.readValue_drdy()*1000
        _tbuff = _tbuff/5
        with self.spi_device as spi:
            spi.write(bytes([0x40, 0x12])+self._registers)        
        _output = (-1*((129.00-_tbuff)*0.403)+25)
        return _output

    def test(self, inp, inn, pga=0xAA, datarate=0x91, ref=0x39, printout=False):
        zeroBuf = []
        t = self.temperature()    
        pgaOLD = self.pgaGain
        self.pgaGain = 4
        inpMux  = (inp << 4) | inn 
        # VBIAS = AVDD+AVSS/2, SYS = inputs shorted to mid supply
        cmd     = bytes([0x42,0x07, inpMux, pga, datarate, ref, 0x00, 0xFF, 0x00, 0x38])
        with self.spi_device as spi:
            spi.write(cmd)
        for _ in range(30):
            testValue = abs(self.readValue_drdy())
            zeroBuf.append(testValue)
        
        mean = sum(zeroBuf) / len(zeroBuf)
        var = sum(pow(x-mean,2) for x in zeroBuf) / len(zeroBuf)
        stdev = sqrt(var)
        if printout:
            [print('%E' % d) for d in zeroBuf]
            print('temp: %f, mean: %E, variance: %E, stdev: %E' % (t, mean, var, stdev))
        self.pgaGain = pgaOLD
        return (t, mean,var,stdev)
    
    def IVsweep(self, inn, inp, start=0x00, stop=0x0A, idacMux=15):
            buffer = []
            for idacMag in range(start, stop):
                buffer.append((hex(idacMag), self.readpins(inn, inp, idacMag=idacMag, idacMux=idacMux, delayT=0.05)))
            return buffer

    def readpins(self, inp, inn, idacMag=0, idacMux=15, vb='off', vbhex='off', pga=0xA8, datarate=0x18, ref=0x39,sys=0x10, delayT=0.01, hall=False, burst=0, wait=False):
        '''

        '''
        vbPin = 0x80
        if vb != 'off' and vb < 6:
            vbPin = (0x80 | 1<<vb) 
        elif vbhex != 'off':
            vbPin = vbhex    
        inpMux  = (inp << 4) | inn               
        burstbuff = []
        cmd     = bytes([0x42,0x07, inpMux, pga, datarate, ref, idacMag, (0xF0 | idacMux), vbPin, sys])
        
        with self.spi_device as spi:
            spi.write(cmd)
        
        if wait:
            return

        if burst > 0:
            for _ in range(burst):
                burstbuff.append(self.readValue_drdy())
            return burstbuff

        if self.DRDY: # DRDY mode
            bufferA = self.readValue_drdy()
        else: # non-DRDY mode
            sleep(delayT)
            bufferA = self.readValue()
        if hall:
            with self.spi_device as spi:
                vapp = (idacMux << 4) | vb 
                spi.write(bytes([0x42,0x00,vapp]))
            if self.DRDY: # DRDY mode
                bufferB = self.readValue_drdy()
                return (bufferA, bufferB)
        return bufferA

    def rawInput(self, inputBuffer, delay):
        with self.spi_device as spi:
            spi.write(inputBuffer)
        sleep(delay)

class XTB(_ADS124S08):
    # XTB class from ADS124S08    
    def __init__(self, spi, xtb_cs, drdy=0, baudrate=6000000, phase=1, polarity=0, refV=2.5):
        self.spi_device = spi_device.SPIDevice(spi, xtb_cs, baudrate=baudrate, phase=phase, polarity=polarity)
        super().__init__(refV, drdy_pin=drdy)

