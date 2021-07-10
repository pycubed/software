import time
try:
    import struct
except ImportError:
    import ustruct as struct

from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_bus_device.spi_device import SPIDevice
from micropython import const
from adafruit_register.i2c_struct import Struct, UnaryStruct
from adafruit_register.i2c_bits import ROBits, RWBits
from adafruit_register.i2c_bit import ROBit, RWBit

# Chip ID
BMX160_CHIP_ID = const(0xD8)

# Soft reset command
BMX160_SOFT_RESET_CMD      = const(0xb6)
BMX160_SOFT_RESET_DELAY    = 0.001

# Command
BMX160_COMMAND_REG_ADDR    = const(0x7E)

# BMX160 Register map
BMX160_CHIP_ID_ADDR        = const(0x00)
BMX160_ERROR_REG_ADDR      = const(0x02)
BMX160_PMU_STATUS_ADDR     = const(0x03)
BMX160_SENSOR_TIME_ADDR    = const(0x18)
BMX160_MAG_DATA_ADDR       = const(0x04)
BMX160_GYRO_DATA_ADDR      = const(0x0C)
BMX160_ACCEL_DATA_ADDR     = const(0x12)
BMX160_STATUS_ADDR         = const(0x1B)
BMX160_INT_STATUS_ADDR     = const(0x1C)
BMX160_TEMP_DATA_ADDR      = const(0x20)
BMX160_FIFO_LENGTH_ADDR    = const(0x22)
BMX160_FIFO_DATA_ADDR      = const(0x24)
BMX160_ACCEL_CONFIG_ADDR   = const(0x40)
BMX160_ACCEL_RANGE_ADDR    = const(0x41)
BMX160_GYRO_CONFIG_ADDR    = const(0x42)
BMX160_GYRO_RANGE_ADDR     = const(0x43)
BMX160_MAG_ODR_ADDR        = const(0x44)
BMX160_FIFO_DOWN_ADDR      = const(0x45)
BMX160_FIFO_CONFIG_0_ADDR  = const(0x46)
BMX160_FIFO_CONFIG_1_ADDR  = const(0x47)
# BMX160_MAG_IF_0_ADDR       = const(0x4B)
BMX160_MAG_IF_0_ADDR       = const(0x4C)
BMX160_MAG_IF_1_ADDR       = const(0x4D)
BMX160_MAG_IF_2_ADDR       = const(0x4E)
BMX160_MAG_IF_3_ADDR       = const(0x4F)
BMX160_INT_ENABLE_0_ADDR   = const(0x50)
BMX160_INT_ENABLE_1_ADDR   = const(0x51)
BMX160_INT_ENABLE_2_ADDR   = const(0x52)
BMX160_INT_OUT_CTRL_ADDR   = const(0x53)
BMX160_INT_LATCH_ADDR      = const(0x54)
BMX160_INT_MAP_0_ADDR      = const(0x55)
BMX160_INT_MAP_1_ADDR      = const(0x56)
BMX160_INT_MAP_2_ADDR      = const(0x57)
BMX160_INT_DATA_0_ADDR     = const(0x58)
BMX160_INT_DATA_1_ADDR     = const(0x59)
BMX160_INT_LOWHIGH_0_ADDR  = const(0x5A)
BMX160_INT_LOWHIGH_1_ADDR  = const(0x5B)
BMX160_INT_LOWHIGH_2_ADDR  = const(0x5C)
BMX160_INT_LOWHIGH_3_ADDR  = const(0x5D)
BMX160_INT_LOWHIGH_4_ADDR  = const(0x5E)
BMX160_INT_MOTION_0_ADDR   = const(0x5F)
BMX160_INT_MOTION_1_ADDR   = const(0x60)
BMX160_INT_MOTION_2_ADDR   = const(0x61)
BMX160_INT_MOTION_3_ADDR   = const(0x62)
BMX160_INT_TAP_0_ADDR      = const(0x63)
BMX160_INT_TAP_1_ADDR      = const(0x64)
BMX160_INT_ORIENT_0_ADDR   = const(0x65)
BMX160_INT_ORIENT_1_ADDR   = const(0x66)
BMX160_INT_FLAT_0_ADDR     = const(0x67)
BMX160_INT_FLAT_1_ADDR     = const(0x68)
BMX160_FOC_CONF_ADDR       = const(0x69)
BMX160_CONF_ADDR           = const(0x6A)

BMX160_ACCEL_BW_NORMAL_AVG4 = const(0x02)
BMX160_GYRO_BW_NORMAL_MODE  = const(0x02)

BMX160_SELF_TEST_ADDR                = const(0x6D)
# Self test configurations
BMX160_ACCEL_SELF_TEST_CONFIG        = const(0x2C)
BMX160_ACCEL_SELF_TEST_POSITIVE_EN   = const(0x0D)
BMX160_ACCEL_SELF_TEST_NEGATIVE_EN   = const(0x09)
BMX160_ACCEL_SELF_TEST_LIMIT         = const(8192)

# Power mode settings
# Accel power mode
BMX160_ACCEL_NORMAL_MODE             = const(0x11)
BMX160_ACCEL_LOWPOWER_MODE           = const(0x12)
BMX160_ACCEL_SUSPEND_MODE            = const(0x10)

BMX160_ACCEL_MODES = [BMX160_ACCEL_NORMAL_MODE,
                      BMX160_ACCEL_LOWPOWER_MODE,
                      BMX160_ACCEL_SUSPEND_MODE]

# Gyro power mode
BMX160_GYRO_SUSPEND_MODE             = const(0x14)
BMX160_GYRO_NORMAL_MODE              = const(0x15)
BMX160_GYRO_FASTSTARTUP_MODE         = const(0x17)

BMX160_GYRO_MODES = [BMX160_GYRO_SUSPEND_MODE,
                     BMX160_GYRO_NORMAL_MODE,
                     BMX160_GYRO_FASTSTARTUP_MODE]

# Mag power mode
BMX160_MAG_SUSPEND_MODE              = const(0x18)
BMX160_MAG_NORMAL_MODE               = const(0x19)
BMX160_MAG_LOWPOWER_MODE             = const(0x1A)

# Accel Range
BMX160_ACCEL_RANGE_2G                = const(0x03)
BMX160_ACCEL_RANGE_4G                = const(0x05)
BMX160_ACCEL_RANGE_8G                = const(0x08)
BMX160_ACCEL_RANGE_16G               = const(0x0C)

# BMX160_ACCEL_RANGE_CONSTANTS = [BMX160_ACCEL_RANGE_16G,
#                                 BMX160_ACCEL_RANGE_8G,
#                                 BMX160_ACCEL_RANGE_4G,
#                                 BMX160_ACCEL_RANGE_2G]
# BMX160_ACCEL_RANGE_VALUES = [16, 8, 4, 2]
BMX160_ACCEL_RANGE_CONSTANTS = [BMX160_ACCEL_RANGE_2G,
                                BMX160_ACCEL_RANGE_4G,
                                BMX160_ACCEL_RANGE_8G,
                                BMX160_ACCEL_RANGE_16G]
BMX160_ACCEL_RANGE_VALUES = [2, 4, 8, 16]


# Gyro Range
BMX160_GYRO_RANGE_2000_DPS           = const(0x00)
BMX160_GYRO_RANGE_1000_DPS           = const(0x01)
BMX160_GYRO_RANGE_500_DPS            = const(0x02)
BMX160_GYRO_RANGE_250_DPS            = const(0x03)
BMX160_GYRO_RANGE_125_DPS            = const(0x04)

BMX160_GYRO_RANGE_CONSTANTS = [BMX160_GYRO_RANGE_2000_DPS,
                               BMX160_GYRO_RANGE_1000_DPS,
                               BMX160_GYRO_RANGE_500_DPS,
                               BMX160_GYRO_RANGE_250_DPS,
                               BMX160_GYRO_RANGE_125_DPS]
BMX160_GYRO_RANGE_VALUES = [2000, 1000, 500, 250, 125]

# Delay in ms settings
BMX160_ACCEL_DELAY                   = 0.005
BMX160_GYRO_DELAY                    = 0.0081
BMX160_ONE_MS_DELAY                  = 0.001
BMX160_MAG_COM_DELAY                 = 0.001
BMX160_GYRO_SELF_TEST_DELAY          = 0.002
BMX160_ACCEL_SELF_TEST_DELAY         = 0.005

# Output Data Rate settings
# Accel Output data rate
BMX160_ACCEL_ODR_RESERVED            = const(0x00)
BMX160_ACCEL_ODR_0_78HZ              = const(0x01)
BMX160_ACCEL_ODR_1_56HZ              = const(0x02)
BMX160_ACCEL_ODR_3_12HZ              = const(0x03)
BMX160_ACCEL_ODR_6_25HZ              = const(0x04)
BMX160_ACCEL_ODR_12_5HZ              = const(0x05)
BMX160_ACCEL_ODR_25HZ                = const(0x06)
BMX160_ACCEL_ODR_50HZ                = const(0x07)
BMX160_ACCEL_ODR_100HZ               = const(0x08)
BMX160_ACCEL_ODR_200HZ               = const(0x09)
BMX160_ACCEL_ODR_400HZ               = const(0x0A)
BMX160_ACCEL_ODR_800HZ               = const(0x0B)
BMX160_ACCEL_ODR_1600HZ              = const(0x0C)
BMX160_ACCEL_ODR_RESERVED0           = const(0x0D)
BMX160_ACCEL_ODR_RESERVED1           = const(0x0E)
BMX160_ACCEL_ODR_RESERVED2           = const(0x0F)

BMX160_ACCEL_ODR_CONSTANTS = [BMX160_ACCEL_ODR_1600HZ,
                              BMX160_ACCEL_ODR_800HZ,
                              BMX160_ACCEL_ODR_400HZ,
                              BMX160_ACCEL_ODR_200HZ,
                              BMX160_ACCEL_ODR_100HZ,
                              BMX160_ACCEL_ODR_50HZ,
                              BMX160_ACCEL_ODR_25HZ,
                              BMX160_ACCEL_ODR_12_5HZ,
                              BMX160_ACCEL_ODR_6_25HZ,
                              BMX160_ACCEL_ODR_3_12HZ,
                              BMX160_ACCEL_ODR_1_56HZ,
                              BMX160_ACCEL_ODR_0_78HZ]
BMX160_ACCEL_ODR_VALUES = [1600, 800, 400, 200, 100, 50, 25, 12.5, 6.25, 3.12, 1.56, 0.78]

# Gyro Output data rate
BMX160_GYRO_ODR_RESERVED             = const(0x00)
BMX160_GYRO_ODR_25HZ                 = const(0x06)
BMX160_GYRO_ODR_50HZ                 = const(0x07)
BMX160_GYRO_ODR_100HZ                = const(0x08)
BMX160_GYRO_ODR_200HZ                = const(0x09)
BMX160_GYRO_ODR_400HZ                = const(0x0A)
BMX160_GYRO_ODR_800HZ                = const(0x0B)
BMX160_GYRO_ODR_1600HZ               = const(0x0C)
BMX160_GYRO_ODR_3200HZ               = const(0x0D)

BMX160_GYRO_ODR_CONSTANTS = [BMX160_GYRO_ODR_3200HZ,
                             BMX160_GYRO_ODR_1600HZ,
                             BMX160_GYRO_ODR_800HZ,
                             BMX160_GYRO_ODR_400HZ,
                             BMX160_GYRO_ODR_200HZ,
                             BMX160_GYRO_ODR_100HZ,
                             BMX160_GYRO_ODR_50HZ]
BMX160_GYRO_ODR_VALUES = [3200, 1600, 800, 400, 200, 100, 50]

# Auxiliary sensor Output data rate
BMX160_MAG_ODR_RESERVED              = const(0x00)
BMX160_MAG_ODR_0_78HZ                = const(0x01)
BMX160_MAG_ODR_1_56HZ                = const(0x02)
BMX160_MAG_ODR_3_12HZ                = const(0x03)
BMX160_MAG_ODR_6_25HZ                = const(0x04)
BMX160_MAG_ODR_12_5HZ                = const(0x05)
BMX160_MAG_ODR_25HZ                  = const(0x06)
BMX160_MAG_ODR_50HZ                  = const(0x07)
BMX160_MAG_ODR_100HZ                 = const(0x08)
BMX160_MAG_ODR_200HZ                 = const(0x09)
BMX160_MAG_ODR_400HZ                 = const(0x0A)
BMX160_MAG_ODR_800HZ                 = const(0x0B)

# Accel, gyro and aux. sensor length and also their combined length definitions in FIFO
BMX160_FIFO_G_LENGTH                 = const(6)
BMX160_FIFO_A_LENGTH                 = const(6)
BMX160_FIFO_M_LENGTH                 = const(8)
BMX160_FIFO_GA_LENGTH                = const(12)
BMX160_FIFO_MA_LENGTH                = const(14)
BMX160_FIFO_MG_LENGTH                = const(14)
BMX160_FIFO_MGA_LENGTH               = const(20)

# I2C address
BMX160_I2C_ADDR            = const(0x68)
BMX160_I2C_ALT_ADDR        = const(0x69)  # alternate address
# Interface settings
BMX160_SPI_INTF            = const(1)
BMX160_I2C_INTF            = const(0)
BMX160_SPI_RD_MASK         = const(0x80)
BMX160_SPI_WR_MASK         = const(0x7F)

# Error related
BMX160_OK                  = const(0)
BMX160_ERROR               = const(-1)

# Each goes with a different sensitivity in decreasing order
AccelSensitivity2Gravity_values = [2048, 4086, 8192, 16384]   # accelerometer sensitivity. See Section 1.2, Table 2
GyroSensitivity2DegPerSec_values = [16.4, 32.8, 65.6, 131.2, 262.4]  # Section 1.2, Table 3

g_TO_METERS_PER_SECOND_SQUARED = 1/9.80665 # in m/s^2

AccelSensitivity2Gravity = const(16384)  # accelerometer sensitivity. See Section 1.2, Table 2
GyroSensitivity2DegPerSec = 131.2        # gyroscope sensitivity. See Section 1.2, Table 3

class _ScaledReadOnlyStruct(Struct):
    def __init__(self, register_address, struct_format, scale):
        super(_ScaledReadOnlyStruct, self).__init__(
            register_address, struct_format)
        self.scale = scale

    # NOTE: I think super() may be an allocating operation.
    def __get__(self, obj, objtype=None):
        result = super(_ScaledReadOnlyStruct, self).__get__(obj, objtype)
        return tuple(self.scale * v for v in result)

    def __set__(self, obj, value):
        raise NotImplementedError()


# TODO replace _SclaedReadOnlyStruct with Struct in BMX160 so that the
# scale factor can be changed as a function of range mode


class BMX160:
    """
    Driver for the BMX160 accelerometer, magnetometer, gyroscope.

    In the buffer, bytes are allocated as follows:
        - mag 0-5
        - rhall 6-7 (not relevant?)
        - gyro 8-13
        - accel 14-19
        - sensor time 20-22
    """

    # multiplicative constants

    # NOTE THESE FIRST TWO GET SET IN THE INIT SEQUENCE
    ACC_SCALAR = 1/(AccelSensitivity2Gravity * g_TO_METERS_PER_SECOND_SQUARED) # 1 m/s^2 = 0.101971621 g
    GYR_SCALAR = 1/GyroSensitivity2DegPerSec_values[4]
    MAG_SCALAR = 1/16
    TEMP_SCALAR = 0.5**9

    _accel = Struct(BMX160_ACCEL_DATA_ADDR, '<hhh') # this is the default scalar, but it should get reset anyhow in init
    _gyro  = Struct(BMX160_GYRO_DATA_ADDR, '<hhh')
    _mag   = Struct(BMX160_MAG_DATA_ADDR, '<hhh')
    _temp  = Struct(BMX160_TEMP_DATA_ADDR, '<h')


    ### STATUS BITS
    status         = ROBits(8, BMX160_STATUS_ADDR, 0)
    status_acc_pmu = ROBits(2, BMX160_PMU_STATUS_ADDR, 4)
    status_gyr_pmu = ROBits(2, BMX160_PMU_STATUS_ADDR, 2)
    status_mag_pmu = ROBits(2, BMX160_PMU_STATUS_ADDR, 0)
    cmd = RWBits(8, BMX160_COMMAND_REG_ADDR, 0)
    foc = RWBits(8, BMX160_FOC_CONF_ADDR, 0)

    # see ERR_REG in section 2.11.2
    _error_status = ROBits(8, BMX160_ERROR_REG_ADDR, 0)
    error_code    = ROBits(4, BMX160_ERROR_REG_ADDR, 1)
    drop_cmd_err  = ROBit(BMX160_ERROR_REG_ADDR, 6)
    fatal_err     = ROBit(BMX160_ERROR_REG_ADDR, 0)

    # straight from the datasheet. Need to be renamed and better explained
    @property
    def drdy_acc(self): return (self.status >> 7) & 1
    @property
    def drdy_gyr(self): return (self.status >> 6) & 1
    @property
    def drdy_mag(self): return (self.status >> 5) & 1
    @property
    def nvm_rdy(self): return (self.status >> 4) & 1
    @property
    def foc_rdy(self): return (self.status >> 3) & 1
    @property
    def mag_man_op(self): return (self.status >> 2) & 1
    @property
    def gyro_self_test_ok(self): return (self.status >> 1)  & 1

    _BUFFER = bytearray(40)
    _smallbuf = bytearray(6)

    _gyro_range = RWBits(8, BMX160_GYRO_RANGE_ADDR, 0)
    _accel_range = RWBits(8, BMX160_ACCEL_RANGE_ADDR, 0)

    # _gyro_bandwidth = NORMAL
    # _gyro_powermode = NORMAL
    _gyro_odr = 25    # Hz

    # _accel_bandwidth = NORMAL
    # _accel_powermode = NORMAL
    _accel_odr = 25  # Hz

    # _mag_bandwidth = NORMAL
    # _mag_powermode = NORMAL
    _mag_odr = 25    # Hz
    _mag_range = 250 # deg/sec


    def __init__(self):
        # soft reset & reboot
        self.cmd = BMX160_SOFT_RESET_CMD
        time.sleep(BMX160_SOFT_RESET_DELAY)
        # Check ID registers.
        ID = self.read_u8(BMX160_CHIP_ID_ADDR)
        if ID != BMX160_CHIP_ID:
            raise RuntimeError('Could not find BMX160, check wiring!')

        # print("status:", format_binary(self.status))
        # set the default settings
        self.init_mag()
        self.init_accel()
        self.init_gyro()
        # print("status:", format_binary(self.status))

    ######################## SENSOR API ########################

    def read_all(self):
        return self.read_bytes(BMX160_MAG_DATA_ADDR, 20, self._BUFFER)

    # synonymous
    @property
    def error_status(self):
        return format_binary(self._error_status)
    @property
    def query_error(self):
        return format_binary(self._error_status)

    ### ACTUAL API
    @property
    def gyro(self):
        return tuple(x * self.GYR_SCALAR for x in self._gyro)

    @property
    def accel(self):
        return tuple(x * self.ACC_SCALAR for x in self._accel)

    @property
    def mag(self):
        return tuple(x * self.MAG_SCALAR for x in self._mag)

    @property
    def temperature(self):
        return self._temp[0]*self.TEMP_SCALAR+23
    @property
    def temp(self):
        return self._temp[0]*self.TEMP_SCALAR+23

    @property
    def sensortime(self):
        tbuf = self.read_bytes(BMX160_SENSOR_TIME_ADDR, 3, self._smallbuf)
        t0, t1, t2 = tbuf[:3]
        t = (t2 << 16) | (t1 << 8) | t0
        t *= 0.000039 # the time resolution is 39 microseconds
        return t

    ######################## SETTINGS RELATED ########################

    ############## GYROSCOPE SETTINGS  ##############
    # NOTE still missing BW / OSR config, but those are more complicated

    def init_gyro(self):
        # BW doesn't have an interface yet
        self._gyro_bwmode = BMX160_GYRO_BW_NORMAL_MODE
        # These rely on the setters to do their magic.
        self.gyro_range = BMX160_GYRO_RANGE_500_DPS
        # self.GYR_SCALAR = 1
        # self.GYR_SCALAR = 1/GyroSensitivity2DegPerSec_values[1]

        self.gyro_odr = 25
        self.gyro_powermode = BMX160_GYRO_NORMAL_MODE

    @property
    def gyro_range(self):
        return self._gyro_range

    @gyro_range.setter
    def gyro_range(self, rangeconst):
        """
        The input is expected to be the BMX160-constant associated with the range.

        deg/s | bmxconst value | bmxconst_name
        ------------------------------------------------------
        2000  |   0            |  BMX160_GYRO_RANGE_2000_DPS
        1000  |   1            |  BMX160_GYRO_RANGE_1000_DPS
        500   |   2            |  BMX160_GYRO_RANGE_500_DPS
        250   |   3            |  BMX160_GYRO_RANGE_250_DPS
        125   |   4            |  BMX160_GYRO_RANGE_125_DPS

        ex: bmx.gyro_range = BMX160_GYRO_RANGE_500_DPS
        equivalent to: bmx.gyro_range = 2
        """

        if rangeconst in BMX160_GYRO_RANGE_CONSTANTS:
            self._gyro_range = rangeconst
            # read out the value to see if it changed successfully
            rangeconst = self._gyro_range
            val = BMX160_GYRO_RANGE_VALUES[rangeconst]
            self.GYR_SCALAR = (val / 32768.0)
        else:
            pass


    @property
    def gyro_odr(self):
        return self._gyro_odr

    @gyro_odr.setter
    def gyro_odr(self, odr):
        """
        Set the output data rate of the gyroscope. The possible ODRs are 1600, 800, 400, 200, 100,
        50, 25, 12.5, 6.25, 3.12, 1.56, and 0.78 Hz. Note, setting a value between the listed ones
        will round *downwards*.
        """
        res = self.generic_setter(odr, BMX160_GYRO_ODR_VALUES,
                                  BMX160_GYRO_ODR_CONSTANTS,
                                  BMX160_GYRO_CONFIG_ADDR,
                                  "gyroscope odr")
        if res != None:
            self._gyro_odr = res[1]

    @property
    def gyro_powermode(self):
        return self._gyro_powermode

    @gyro_powermode.setter
    def gyro_powermode(self, powermode):
        """
        Set the power mode of the gyroscope. Unlike other setters, this one has to directly take the
        BMX160-const associated with the power mode. The possible modes are:
        `BMX160_GYRO_SUSPEND_MODE`
        `BMX160_GYRO_NORMAL_MODE`
        `BMX160_GYRO_FASTSTARTUP_MODE`
        """
        if powermode not in BMX160_GYRO_MODES:
            print("Unknown gyroscope powermode: " + str(powermode))
            return

        self.write_u8(BMX160_COMMAND_REG_ADDR, powermode)
        if int(self.query_error) == 0:
            self._gyro_powermode = powermode
        else:
            settingswarning("gyroscope power mode")

        # NOTE: this delay is a worst case. If we need repeated switching
        # we can choose the delay on a case-by-case basis.
        time.sleep(BMX160_GYRO_DELAY)


    ############## ACCELEROMETER SETTINGS  ##############

    def init_accel(self):
        # BW doesn't have an interface yet
        # self.write_u8(BMX160_ACCEL_CONFIG_ADDR, BMX160_ACCEL_BW_NORMAL_AVG4)
        # self._accel_bwmode = BMX160_ACCEL_BW_NORMAL_AVG4
        # These rely on the setters to do their magic.
        self.accel_range = BMX160_ACCEL_RANGE_8G
        self.accel_odr = 25
        self.accel_powermode = BMX160_ACCEL_NORMAL_MODE

    @property
    def accel_range(self):
        return self._accel_range

    @accel_range.setter
    def accel_range(self, rangeconst):
        """
        The input is expected to be the BMX160-constant associated with the range.

        deg/s | bmxconst value | bmxconst name
        ------------------------------------------------------
        2     |   3            | BMX160_ACCEL_RANGE_2G
        4     |   5            | BMX160_ACCEL_RANGE_4G
        8     |   8            | BMX160_ACCEL_RANGE_8G
        16    |   12           | BMX160_ACCEL_RANGE_16G

        ex: bmx.accel_range = BMX160_ACCEL_RANGE_4G
        equivalent to: bmx.accel_range = 5
        """

        if rangeconst in BMX160_ACCEL_RANGE_CONSTANTS:
            self._accel_range = rangeconst
            # read out the value to see if it changed successfully
            rangeconst = self._accel_range
            # convert to 0-3 range for indexing
            ind = rangeconst >> 2
            val = BMX160_ACCEL_RANGE_VALUES[ind]
            self.ACC_SCALAR = (val / 32768.0) / g_TO_METERS_PER_SECOND_SQUARED
        else:
            pass

    @property
    def accel_odr(self):
        return self._accel_odr

    @accel_odr.setter
    def accel_odr(self, odr):
        res = self.generic_setter(odr, BMX160_ACCEL_ODR_VALUES,
                                  BMX160_ACCEL_ODR_CONSTANTS,
                                  BMX160_ACCEL_CONFIG_ADDR,
                                  "accelerometer odr")
        if res != None:
            self._accel_odr = res[1]

    @property
    def accel_powermode(self):
        return self._accel_powermode

    @accel_powermode.setter
    def accel_powermode(self, powermode):
        """
        Set the power mode of the accelerometer. Unlike other setters, this one has to directly take the
        BMX160-const associated with the power mode. The possible modes are:
        `BMI160_ACCEL_NORMAL_MODE`
        `BMI160_ACCEL_LOWPOWER_MODE`
        `BMI160_ACCEL_SUSPEND_MODE`
        """
        if powermode not in BMX160_ACCEL_MODES:
            print("Unknown accelerometer power mode: " + str(powermode))
            return

        self.write_u8(BMX160_COMMAND_REG_ADDR, powermode)
        if int(self.query_error) == 0:
            self._accel_powermode = powermode
        else:
            settingswarning("accelerometer power mode")

        # NOTE: this delay is a worst case. If we need repeated switching
        # we can choose the delay on a case-by-case basis.
        time.sleep(BMX160_ACCEL_DELAY)

    ############## MAGENTOMETER SETTINGS  ##############

    def init_mag(self):
        # see pg 25 of: https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMX160-DS000.pdf
        self.write_u8(BMX160_COMMAND_REG_ADDR, BMX160_MAG_NORMAL_MODE)
        time.sleep(0.00065) # datasheet says wait for 650microsec
        self.write_u8(BMX160_MAG_IF_0_ADDR, 0x80)
        # put mag into sleep mode
        self.write_u8(BMX160_MAG_IF_3_ADDR, 0x01)
        self.write_u8(BMX160_MAG_IF_2_ADDR, 0x4B)
        # set x-y to regular power preset
        self.write_u8(BMX160_MAG_IF_3_ADDR, 0x04)
        self.write_u8(BMX160_MAG_IF_2_ADDR, 0x51)
        # set z to regular preset
        self.write_u8(BMX160_MAG_IF_3_ADDR, 0x0E)
        self.write_u8(BMX160_MAG_IF_2_ADDR, 0x52)
        # prepare MAG_IF[1-3] for mag_if data mode
        self.write_u8(BMX160_MAG_IF_3_ADDR, 0x02)
        self.write_u8(BMX160_MAG_IF_2_ADDR, 0x4C)
        self.write_u8(BMX160_MAG_IF_1_ADDR, 0x42)
        # Set ODR to 25 Hz
        self.write_u8(BMX160_MAG_ODR_ADDR, BMX160_MAG_ODR_25HZ)
        self.write_u8(BMX160_MAG_IF_0_ADDR, 0x00)
        # put in low power mode.
        self.write_u8(BMX160_COMMAND_REG_ADDR, BMX160_MAG_LOWPOWER_MODE)
        time.sleep(0.1) # takes this long to warm up (empirically)


    ## UTILS:
    def generic_setter(self, desired, possible_values, bmx_constants, config_addr, warning_interp = ""):
        i = find_nearest_valid(desired, possible_values)
        rounded = possible_values[i]
        bmxconst = bmx_constants[i]
        self.write_u8(config_addr, bmxconst)
        e = self.error_code

        if e == BMX160_OK:
            return (i, rounded)
        else:
            settingswarning(warning_interp)


class BMX160_I2C(BMX160):
    """Driver for the BMX160 connect over I2C."""

    def __init__(self, i2c):

        try:
            self.i2c_device = I2CDevice(i2c, BMX160_I2C_ADDR)
        except:
            self.i2c_device = I2CDevice(i2c, BMX160_I2C_ALT_ADDR)

        super().__init__()

    def read_u8(self, address):
        with self.i2c_device as i2c:
            self._BUFFER[0] = address & 0xFF
            i2c.write_then_readinto(self._BUFFER, self._BUFFER, out_end=1, in_start=1, in_end=2)
        return self._BUFFER[1]

    def read_bytes(self, address, count, buf):
        with self.i2c_device as i2c:
            buf[0] = address & 0xFF
            i2c.write_then_readinto(buf, buf, out_end=1, in_end=count)
        return buf

    def write_u8(self, address, val):
        with self.i2c_device as i2c:
            self._BUFFER[0] = address & 0xFF
            self._BUFFER[1] = val & 0xFF
            i2c.write(self._BUFFER, end=2, stop=True)


class BMX160_SPI(BMX160):
    """Driver for the BMX160 connect over SPI."""
    def __init__(self, spi, cs):
        self.i2c_device = SPIDevice(spi, cs)
        super().__init__()

    def read_u8(self, address):
        with self.i2c_device as spi:
            self._BUFFER[0] = (address | 0x80) & 0xFF
            spi.write(self._BUFFER, end=1)
            spi.readinto(self._BUFFER, end=1)
        return self._BUFFER[0]

    def read_bytes(self, address, count, buf):
        with self.i2c_device as spi:
            buf[0] = (address | 0x80) & 0xFF
            spi.write(buf, end=1)
            spi.readinto(buf, end=count)
        return buf

    def write_u8(self, address, val):
        with self.i2c_device as spi:
            self._BUFFER[0] = (address & 0x7F) & 0xFF
            self._BUFFER[1] = val & 0xFF
            spi.write(self._BUFFER, end=2)


# GENERIC UTILS:

def find_nearest_valid(desired, possible_values):
    # NOTE: assumes `possible_values` is sorted in decreasing order

    # This line finds the first value less than or equal to the desired value and returns its index.
    # If no such value exists (desired is lower than all possible), the line throws a StopIteration
    # Exception. In that case we return -1 as the index to use (i.e. the last/smallest value in the list)
    try:
        return next(filter(lambda x: (desired >= x[1]), enumerate(possible_values)))[0]
    except:
        return -1

def settingswarning(interp = ""):
    if interp != "":
            interp = " --"  + interp + " -- "
    print("BMX160 error occurred during " + interp +
         "setting change. \nSetting not successfully changed and BMX160 may be in error state.")


def format_binary(b):
    return '{:0>8}'.format(bin(b)[2:])