# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
MODIFIED VERSION of adafruit_rfm9x CircuitPython Library for PyCubed Use
See https://github.com/adafruit/Adafruit_CircuitPython_RFM9x

CircuitPython Version: 7.0.0 alpha
Library Repo: https://github.com/pycubed/library_pycubed.py
* Edits by: Max Holliday
"""
import time
from random import random
import digitalio
from micropython import const
import adafruit_bus_device.spi_device as spidev

# pylint: disable=bad-whitespace
# Internal constants:
# Register names (FSK Mode even though we use LoRa instead, from table 85)
_RH_RF95_REG_00_FIFO = const(0x00)
_RH_RF95_REG_01_OP_MODE = const(0x01)
_RH_RF95_REG_06_FRF_MSB = const(0x06)
_RH_RF95_REG_07_FRF_MID = const(0x07)
_RH_RF95_REG_08_FRF_LSB = const(0x08)
_RH_RF95_REG_09_PA_CONFIG = const(0x09)
_RH_RF95_REG_0A_PA_RAMP = const(0x0A)
_RH_RF95_REG_0B_OCP = const(0x0B)
_RH_RF95_REG_0C_LNA = const(0x0C)
_RH_RF95_REG_0D_FIFO_ADDR_PTR = const(0x0D)
_RH_RF95_REG_0E_FIFO_TX_BASE_ADDR = const(0x0E)
_RH_RF95_REG_0F_FIFO_RX_BASE_ADDR = const(0x0F)
_RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR = const(0x10)
_RH_RF95_REG_11_IRQ_FLAGS_MASK = const(0x11)
_RH_RF95_REG_12_IRQ_FLAGS = const(0x12)
_RH_RF95_REG_13_RX_NB_BYTES = const(0x13)
_RH_RF95_REG_14_RX_HEADER_CNT_VALUE_MSB = const(0x14)
_RH_RF95_REG_15_RX_HEADER_CNT_VALUE_LSB = const(0x15)
_RH_RF95_REG_16_RX_PACKET_CNT_VALUE_MSB = const(0x16)
_RH_RF95_REG_17_RX_PACKET_CNT_VALUE_LSB = const(0x17)
_RH_RF95_REG_18_MODEM_STAT = const(0x18)
_RH_RF95_REG_19_PKT_SNR_VALUE = const(0x19)
_RH_RF95_REG_1A_PKT_RSSI_VALUE = const(0x1A)
_RH_RF95_REG_1B_RSSI_VALUE = const(0x1B)
_RH_RF95_REG_1C_HOP_CHANNEL = const(0x1C)
_RH_RF95_REG_1D_MODEM_CONFIG1 = const(0x1D)
_RH_RF95_REG_1E_MODEM_CONFIG2 = const(0x1E)
_RH_RF95_REG_1F_SYMB_TIMEOUT_LSB = const(0x1F)
_RH_RF95_REG_20_PREAMBLE_MSB = const(0x20)
_RH_RF95_REG_21_PREAMBLE_LSB = const(0x21)
_RH_RF95_REG_22_PAYLOAD_LENGTH = const(0x22)
_RH_RF95_REG_23_MAX_PAYLOAD_LENGTH = const(0x23)
_RH_RF95_REG_24_HOP_PERIOD = const(0x24)
_RH_RF95_REG_25_FIFO_RX_BYTE_ADDR = const(0x25)
_RH_RF95_REG_26_MODEM_CONFIG3 = const(0x26)

_RH_RF95_REG_40_DIO_MAPPING1 = const(0x40)
_RH_RF95_REG_41_DIO_MAPPING2 = const(0x41)
_RH_RF95_REG_42_VERSION = const(0x42)

_RH_RF95_REG_4B_TCXO = const(0x4B)
_RH_RF95_REG_4D_PA_DAC = const(0x4D)
_RH_RF95_REG_5B_FORMER_TEMP = const(0x5B)
_RH_RF95_REG_61_AGC_REF = const(0x61)
_RH_RF95_REG_62_AGC_THRESH1 = const(0x62)
_RH_RF95_REG_63_AGC_THRESH2 = const(0x63)
_RH_RF95_REG_64_AGC_THRESH3 = const(0x64)

_RH_RF95_DETECTION_OPTIMIZE = const(0x31)
_RH_RF95_DETECTION_THRESHOLD = const(0x37)

_RH_RF95_PA_DAC_DISABLE = const(0x04)
_RH_RF95_PA_DAC_ENABLE = const(0x07)

# The Frequency Synthesizer step = RH_RF95_FXOSC / 2^^19
_RH_RF95_FSTEP = 32000000 / 524288

# RadioHead specific compatibility constants.
_RH_BROADCAST_ADDRESS = const(0xFF)

# The acknowledgement bit in the FLAGS
# The top 4 bits of the flags are reserved for RadioHead. The lower 4 bits are reserved
# for application layer use.
_RH_FLAGS_ACK = const(0x80)
_RH_FLAGS_RETRY = const(0x40)

# User facing constants:
SLEEP_MODE  = const(0)#0b000
STANDBY_MODE= const(1)#0b001
FS_TX_MODE  = const(2)#0b010
TX_MODE     = const(3)#0b011
FS_RX_MODE  = const(4)#0b100
RX_MODE     = const(5)#0b101
# pylint: enable=bad-whitespace

# gap =bytes([0xFF])
# sgap=bytes([0xFF,0xFF,0xFF])
# dot =bytes([0])
# dash=bytes([0,0,0])
# # ...- .-. ...-- -..-
# VR3X = (gap+(dot+gap)*3)+dash+sgap+\
# (dot+gap)+dash+gap+dot+sgap+\
# ((dot+gap)*3)+dash+gap+dash+sgap+\
# dash+gap+((dot+gap)*2)+dash+gap
VR3X=b'\xff\x00\xff\x00\xff\x00\xff\x00\x00\x00\xff\xff\xff\x00\xff\x00\x00\x00\xff\x00\xff\xff\xff\x00\xff\x00\xff\x00\xff\x00\x00\x00\xff\x00\x00\x00\xff\xff\xff\x00\x00\x00\xff\x00\xff\x00\xff\x00\x00\x00\xff'


# Disable the too many instance members warning.  Pylint has no knowledge
# of the context and is merely guessing at the proper amount of members.  This
# is a complex chip which requires exposing many attributes and state.  Disable
# the warning to work around the error.
# pylint: disable=too-many-instance-attributes

_bigbuffer=bytearray(256)
bw_bins = (7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000)
class RFM9x:
    """Interface to a RFM95/6/7/8 LoRa radio module.  Allows sending and
    receivng bytes of data in long range LoRa mode at a support board frequency
    (433/915mhz).

    You must specify the following parameters:
    - spi: The SPI bus connected to the radio.
    - cs: The CS pin DigitalInOut connected to the radio.
    - reset: The reset/RST pin DigialInOut connected to the radio.
    - frequency: The frequency (in mhz) of the radio module (433/915mhz typically).

    You can optionally specify:
    - preamble_length: The length in bytes of the packet preamble (default 8).
    - high_power: Boolean to indicate a high power board (RFM95, etc.).  Default
    is True for high power.
    - baudrate: Baud rate of the SPI connection, default is 10mhz but you might
    choose to lower to 1mhz if using long wires or a breadboard.

    Remember this library makes a best effort at receiving packets with pure
    Python code.  Trying to receive packets too quickly will result in lost data
    so limit yourself to simple scenarios of sending and receiving single
    packets at a time.

    Also note this library tries to be compatible with raw RadioHead Arduino
    library communication. This means the library sets up the radio modulation
    to match RadioHead's defaults and assumes that each packet contains a
    4 byte header compatible with RadioHead's implementation.
    Advanced RadioHead features like address/node specific packets
    or "reliable datagram" delivery are supported however due to the
    limitations noted, "reliable datagram" is still subject to missed packets but with it,
    sender is notified if a packet has potentially been missed.
    """

    # Global buffer for SPI commands
    _BUFFER = bytearray(4)
    DEBUG_HEADER=False
    valid_ids  = (58,59,60,255)
    class _RegisterBits:
        # Class to simplify access to the many configuration bits avaialable
        # on the chip's registers.  This is a subclass here instead of using
        # a higher level module to increase the efficiency of memory usage
        # (all of the instances of this bit class will share the same buffer
        # used by the parent RFM69 class instance vs. each having their own
        # buffer and taking too much memory).

        # Quirk of pylint that it requires public methods for a class.  This
        # is a decorator class in Python and by design it has no public methods.
        # Instead it uses dunder accessors like get and set below.  For some
        # reason pylint can't figure this out so disable the check.
        # pylint: disable=too-few-public-methods

        # Again pylint fails to see the true intent of this code and warns
        # against private access by calling the write and read functions below.
        # This is by design as this is an internally used class.  Disable the
        # check from pylint.
        # pylint: disable=protected-access

        def __init__(self, address, *, offset=0, bits=1):
            assert 0 <= offset <= 7
            assert 1 <= bits <= 8
            assert (offset + bits) <= 8
            self._address = address
            self._mask = 0
            for _ in range(bits):
                self._mask <<= 1
                self._mask |= 1
            self._mask <<= offset
            self._offset = offset

        def __get__(self, obj, objtype):
            reg_value = obj._read_u8(self._address)
            return (reg_value & self._mask) >> self._offset

        def __set__(self, obj, val):
            reg_value = obj._read_u8(self._address)
            reg_value &= ~self._mask
            reg_value |= (val & 0xFF) << self._offset
            obj._write_u8(self._address, reg_value)

    operation_mode = _RegisterBits(_RH_RF95_REG_01_OP_MODE, bits=3)

    low_frequency_mode = _RegisterBits(_RH_RF95_REG_01_OP_MODE, offset=3, bits=1)

    osc_calibration   = _RegisterBits(_RH_RF95_REG_24_HOP_PERIOD, offset=3, bits=1)

    modulation_type = _RegisterBits(_RH_RF95_REG_01_OP_MODE, offset=5, bits=2)

    # Long range/LoRa mode can only be set in sleep mode!
    long_range_mode = _RegisterBits(_RH_RF95_REG_01_OP_MODE, offset=7, bits=1)


    lna_boost = _RegisterBits(_RH_RF95_REG_0C_LNA, bits=2)

    output_power = _RegisterBits(_RH_RF95_REG_09_PA_CONFIG, bits=4)

    modulation_shaping = _RegisterBits(_RH_RF95_REG_0A_PA_RAMP, bits=2)

    pa_ramp = _RegisterBits(_RH_RF95_REG_0A_PA_RAMP, bits=4)

    max_power = _RegisterBits(_RH_RF95_REG_09_PA_CONFIG, offset=4, bits=3)

    pa_select = _RegisterBits(_RH_RF95_REG_09_PA_CONFIG, offset=7, bits=1)

    pa_dac = _RegisterBits(_RH_RF95_REG_4D_PA_DAC, bits=3)

    dio0_mapping = _RegisterBits(_RH_RF95_REG_40_DIO_MAPPING1, offset=6, bits=2)

    low_datarate_optimize = _RegisterBits(_RH_RF95_REG_26_MODEM_CONFIG3, offset=3, bits=1)
    auto_agc  = _RegisterBits(_RH_RF95_REG_26_MODEM_CONFIG3, offset=2, bits=1)

    debug=False
    buffview = memoryview(_bigbuffer)
    def __init__(
        self,
        spi,
        cs,
        reset,
        frequency,
        *,
        preamble_length=8,
        code_rate=5,
        high_power=True,
        baudrate=5000000,
        max_output=False
    ):
        self.high_power = high_power
        self.max_output=max_output
        self.dio0=False
        # Device support SPI mode 0 (polarity & phase = 0) up to a max of 10mhz.
        # Set Default Baudrate to 5MHz to avoid problems
        self._device = spidev.SPIDevice(spi, cs, baudrate=baudrate, polarity=0, phase=0)
        # Setup reset as a digital input (default state for reset line according
        # to the datasheet).  This line is pulled low as an output quickly to
        # trigger a reset.  Note that reset MUST be done like this and set as
        # a high impedence input or else the chip cannot change modes (trust me!).
        self._reset = reset
        self._reset.switch_to_input(pull=digitalio.Pull.UP)
        self.reset()
        # No device type check!  Catch an error from the very first request and
        # throw a nicer message to indicate possible wiring problems.
        version = self._read_u8(_RH_RF95_REG_42_VERSION)
        if version != 18:
            raise RuntimeError(
                "Failed to find rfm9x with expected version -- check wiring"
            )

        # Set sleep mode, wait 10ms and confirm in sleep mode (basic device check).
        # Also set long range mode (LoRa mode) as it can only be done in sleep.
        self.idle()
        time.sleep(0.01)
        self.osc_calibration=True
        time.sleep(1)

        self.sleep()
        time.sleep(0.01)
        self.long_range_mode = True
        if self.operation_mode != SLEEP_MODE or not self.long_range_mode:
            raise RuntimeError("Failed to configure radio for LoRa mode, check wiring!")
        # clear default setting for access to LF registers if frequency > 525MHz
        if frequency > 525:
            self.low_frequency_mode = 0
        # Setup entire 256 byte FIFO
        self._write_u8(_RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0x00)
        self._write_u8(_RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0x00)
        # Disable Freq Hop
        self._write_u8(_RH_RF95_REG_24_HOP_PERIOD, 0x00)
        # Set mode idle
        self.idle()


        # Set frequency
        self.frequency_mhz = frequency
        # Set preamble length (default 8 bytes to match radiohead).
        self.preamble_length = preamble_length
        # Defaults set modem config to RadioHead compatible Bw125Cr45Sf128 mode.
        self.signal_bandwidth = 125000
        self.coding_rate = code_rate
        self.spreading_factor = 7
        # Default to disable CRC checking on incoming packets.
        self.enable_crc = False
        # Note no sync word is set for LoRa mode either!
        self._write_u8(_RH_RF95_REG_26_MODEM_CONFIG3, 0x00)
        # Set transmit power to 13 dBm, a safe value any module supports.
        self.tx_power = 13
        # initialize last RSSI reading
        self.last_rssi = 0.0
        """The RSSI of the last received packet. Stored when the packet was received.
           This instantaneous RSSI value may not be accurate once the
           operating mode has been changed.
        """
        # initialize timeouts and delays delays
        self.ack_wait = 0.5
        """The delay time before attempting a retry after not receiving an ACK"""
        self.receive_timeout = 0.5
        """The amount of time to poll for a received packet.
           If no packet is received, the returned packet will be None
        """
        self.xmit_timeout = 2.0
        """The amount of time to wait for the HW to transmit the packet.
           This is mainly used to prevent a hang due to a HW issue
        """
        self.ack_retries = 5
        """The number of ACK retries before reporting a failure."""
        self.ack_delay = None
        """The delay time before attemting to send an ACK.
           If ACKs are being missed try setting this to .1 or .2.
        """
        # initialize sequence number counter for reliabe datagram mode
        self.sequence_number = 0
        # create seen Ids list
        self.seen_ids = bytearray(256)
        # initialize packet header
        # node address - default is broadcast
        self.node = _RH_BROADCAST_ADDRESS
        """The default address of this Node. (0-255).
           If not 255 (0xff) then only packets address to this node will be accepted.
           First byte of the RadioHead header.
        """
        # destination address - default is broadcast
        self.destination = _RH_BROADCAST_ADDRESS
        """The default destination address for packet transmissions. (0-255).
           If 255 (0xff) then any receiving node should accept the packet.
           Second byte of the RadioHead header.
        """
        # ID - contains seq count for reliable datagram mode
        self.identifier = 0
        """Automatically set to the sequence number when send_with_ack() used.
           Third byte of the RadioHead header.
        """
        # flags - identifies ack/reetry packet for reliable datagram mode
        self.flags = 0
        """Upper 4 bits reserved for use by Reliable Datagram Mode.
           Lower 4 bits may be used to pass information.
           Fourth byte of the RadioHead header.
        """
        self.crc_error_count = 0

        self.auto_agc=True
        self.pa_ramp=0   # mode agnostic
        self.lna_boost=3 # mode agnostic

    def cw(self,msg=None):
        success=False
        if msg is None:
            msg = VR3X

        cache=[]
        if self.long_range_mode:
            # cache LoRa params
            cache = [self.spreading_factor,
                    self.signal_bandwidth,
                    self.coding_rate,
                    self.preamble_length,
                    self.enable_crc]

        self.operation_mode = SLEEP_MODE
        time.sleep(0.01)
        self.long_range_mode=False # FSK/OOK Mode
        self.modulation_type=1 # OOK
        self.modulation_shaping = 2
        self._write_u8(0x25,0x00) # no preamble
        self._write_u8(0x26,0x00) # no preamble
        self._write_u8(0x27,0x00) # no sync word
        self._write_u8(0x3f,10)   # clear FIFO
        self._write_u8(0x02,0xFF) # BitRate(15:8)
        self._write_u8(0x03,0xFF) # BitRate(15:8)
        self._write_u8(0x05,11)   # Freq deviation Lsb 600 Hz
        self.idle()
        # Set payload length VR3X Morse length = 51
        self._write_u8(0x35,len(msg)-1)
        self._write_from(_RH_RF95_REG_00_FIFO, bytearray(msg))

        _t=time.monotonic() + 10
        self.operation_mode = TX_MODE
        while time.monotonic() < _t:
            a=self._read_u8(0x3f)
            # print(a,end=' ')
            if (a>>6)&1:
                time.sleep(0.01)
                success=True
                break
        if not (a>>6)&1:
            print('cw timeout')
        self.idle()
        if cache:
            self.operation_mode = SLEEP_MODE
            time.sleep(0.01)
            self.long_range_mode = True
            self._write_u8(_RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0x00)
            self._write_u8(_RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0x00)
            self._write_u8(_RH_RF95_REG_24_HOP_PERIOD, 0x00)
            self.idle()
            self.spreading_factor = cache[0]
            self.signal_bandwidth = cache[1]
            self.coding_rate      = cache[2]
            self.preamble_length  = cache[3]
            self.enable_crc       = cache[4]
            self.auto_agc = True
            self.low_datarate_optimize = True
        return success

    # pylint: disable=no-member
    # Reconsider pylint: disable when this can be tested
    def _read_into(self, address, buf, length=None):
        # Read a number of bytes from the specified address into the provided
        # buffer.  If length is not specified (the default) the entire buffer
        # will be filled.
        if length is None:
            length = len(buf)
        with self._device as device:
            self._BUFFER[0] = address & 0x7F  # Strip out top bit to set 0
            # value (read).
            device.write(self._BUFFER, end=1)
            device.readinto(buf, end=length)

    def _read_u8(self, address):
        # Read a single byte from the provided address and return it.
        self._read_into(address, self._BUFFER, length=1)
        return self._BUFFER[0]

    def _write_from(self, address, buf, length=None):
        # Write a number of bytes to the provided address and taken from the
        # provided buffer.  If no length is specified (the default) the entire
        # buffer is written.
        if length is None:
            length = len(buf)
        with self._device as device:
            self._BUFFER[0] = (address | 0x80) & 0xFF  # Set top bit to 1 to
            # indicate a write.
            device.write(self._BUFFER, end=1)
            device.write(buf, end=length)

    def _write_u8(self, address, val):
        # Write a byte register to the chip.  Specify the 7-bit address and the
        # 8-bit value to write to that address.
        with self._device as device:
            self._BUFFER[0] = (address | 0x80) & 0xFF  # Set top bit to 1 to
            # indicate a write.
            self._BUFFER[1] = val & 0xFF
            device.write(self._BUFFER, end=2)

    def reset(self):
        """Perform a reset of the chip."""
        # See section 7.2.2 of the datasheet for reset description.
        self._reset.switch_to_output(value=False)
        time.sleep(0.0001)  # 100 us
        self._reset.switch_to_input(pull=digitalio.Pull.UP)
        time.sleep(0.005)  # 5 ms

    def idle(self):
        """Enter idle standby mode."""
        self.operation_mode = STANDBY_MODE

    def sleep(self):
        """Enter sleep mode."""
        self.operation_mode = SLEEP_MODE

    def listen(self):
        """Listen for packets to be received by the chip.  Use :py:func:`receive`
        to listen, wait and retrieve packets as they're available.
        """
        self.operation_mode = RX_MODE
        self.dio0_mapping = 0b00  # Interrupt on rx done.

    def transmit(self):
        """Transmit a packet which is queued in the FIFO.  This is a low level
        function for entering transmit mode and more.  For generating and
        transmitting a packet of data use :py:func:`send` instead.
        """
        self.operation_mode = TX_MODE
        self.dio0_mapping = 0b01  # Interrupt on tx done.

    @property
    def preamble_length(self):
        """The length of the preamble for sent and received packets, an unsigned
        16-bit value.  Received packets must match this length or they are
        ignored! Set to 8 to match the RadioHead RFM95 library.
        """
        msb = self._read_u8(_RH_RF95_REG_20_PREAMBLE_MSB)
        lsb = self._read_u8(_RH_RF95_REG_21_PREAMBLE_LSB)
        return ((msb << 8) | lsb) & 0xFFFF

    @preamble_length.setter
    def preamble_length(self, val):
        assert 0 <= val <= 65535
        self._write_u8(_RH_RF95_REG_20_PREAMBLE_MSB, (val >> 8) & 0xFF)
        self._write_u8(_RH_RF95_REG_21_PREAMBLE_LSB, val & 0xFF)

    @property
    def frequency_mhz(self):
        """The frequency of the radio in Megahertz. Only the allowed values for
        your radio must be specified (i.e. 433 vs. 915 mhz)!
        """
        msb = self._read_u8(_RH_RF95_REG_06_FRF_MSB)
        mid = self._read_u8(_RH_RF95_REG_07_FRF_MID)
        lsb = self._read_u8(_RH_RF95_REG_08_FRF_LSB)
        frf = ((msb << 16) | (mid << 8) | lsb) & 0xFFFFFF
        frequency = (frf * _RH_RF95_FSTEP) / 1000000.0
        return frequency

    @frequency_mhz.setter
    def frequency_mhz(self, val):
        if val < 240 or val > 960:
            raise RuntimeError("frequency_mhz must be between 240 and 960")
        # Calculate FRF register 24-bit value.
        frf = int((val * 1000000.0) / _RH_RF95_FSTEP) & 0xFFFFFF
        # Extract byte values and update registers.
        msb = frf >> 16
        mid = (frf >> 8) & 0xFF
        lsb = frf & 0xFF
        self._write_u8(_RH_RF95_REG_06_FRF_MSB, msb)
        self._write_u8(_RH_RF95_REG_07_FRF_MID, mid)
        self._write_u8(_RH_RF95_REG_08_FRF_LSB, lsb)

    @property
    def tx_power(self):
        """The transmit power in dBm. Can be set to a value from 5 to 23 for
        high power devices (RFM95/96/97/98, high_power=True) or -1 to 14 for low
        power devices. Only integer power levels are actually set (i.e. 12.5
        will result in a value of 12 dBm).
        The actual maximum setting for high_power=True is 20dBm but for values > 20
        the PA_BOOST will be enabled resulting in an additional gain of 3dBm.
        The actual setting is reduced by 3dBm.
        The reported value will reflect the reduced setting.
        """
        if self.high_power:
            return self.output_power + 5
        return self.output_power - 1

    @tx_power.setter
    def tx_power(self, val):
        val = int(val)
        if self.max_output is True:
            print('RFM9X Max Output Power Enabled')
            self._write_u8(_RH_RF95_REG_0B_OCP,0x3F) # set Ocp to 240mA
            self.pa_dac = _RH_RF95_PA_DAC_ENABLE
            self.pa_select = True
            self.max_power = 0b111
            self.output_power=0x0F
            return

        if self.high_power:
            if val < 5 or val > 23:
                raise RuntimeError("tx_power must be between 5 and 23")
            # Enable power amp DAC if power is above 20 dB.
            # Lower setting by 3db when PA_BOOST enabled - see Data Sheet  Section 6.4
            if val > 20:
                self.pa_dac = _RH_RF95_PA_DAC_ENABLE
                val -= 3
            else:
                self.pa_dac = _RH_RF95_PA_DAC_DISABLE
            self.pa_select = True
            self.output_power = (val - 5) & 0x0F
        else:
            assert -1 <= val <= 14
            self.pa_select = False
            self.max_power = 0b111  # Allow max power output.
            self.output_power = (val + 1) & 0x0F

    # ADDED FOR PYCUBED
    @property
    def packet_status(self):
        return (self.rssi,self._read_u8(_RH_RF95_REG_19_PKT_SNR_VALUE)/4)

    @property
    def pll_timeout(self):
        return (self._read_u8(_RH_RF95_REG_1C_HOP_CHANNEL))

    def rssi(self,raw=False):
        """The received strength indicator (in dBm) of the last received message."""
        # Read RSSI register and convert to value using formula in datasheet.
        # Remember in LoRa mode the payload register changes function to RSSI!
        if raw:
            return self._read_u8(_RH_RF95_REG_1A_PKT_RSSI_VALUE)
        return self._read_u8(_RH_RF95_REG_1A_PKT_RSSI_VALUE) - 137

    @property
    def signal_bandwidth(self):
        """The signal bandwidth used by the radio (try setting to a higher
        value to increase throughput or to a lower value to increase the
        likelihood of successfully received payloads).  Valid values are
        listed in RFM9x.bw_bins."""
        bw_id = (self._read_u8(_RH_RF95_REG_1D_MODEM_CONFIG1) & 0xF0) >> 4
        if bw_id >= len(bw_bins):
            current_bandwidth = 500000
        else:
            current_bandwidth = bw_bins[bw_id]
        return current_bandwidth

    @signal_bandwidth.setter
    def signal_bandwidth(self, val):
        # Set signal bandwidth (set to 125000 to match RadioHead Bw125).
        for bw_id, cutoff in enumerate(bw_bins):
            if val <= cutoff:
                break
        else:
            bw_id = 9
        self._write_u8(
            _RH_RF95_REG_1D_MODEM_CONFIG1,
            (self._read_u8(_RH_RF95_REG_1D_MODEM_CONFIG1) & 0x0F) | (bw_id << 4))
        if val >= 500000:
            # see Semtech SX1276 errata note 2.1
            self._write_u8(0x36,0x02)
            self._write_u8(0x3a,0x64)
        else:
            if val == 7800:
                self._write_u8(0x2F,0x48)
            elif val >= 62500:
                # see Semtech SX1276 errata note 2.3
                self._write_u8(0x2F,0x40)
            else:
                self._write_u8(0x2F,0x44)
            self._write_u8(0x30,0)


    @property
    def coding_rate(self):
        """The coding rate used by the radio to control forward error
        correction (try setting to a higher value to increase tolerance of
        short bursts of interference or to a lower value to increase bit
        rate).  Valid values are limited to 5, 6, 7, or 8."""
        cr_id = (self._read_u8(_RH_RF95_REG_1D_MODEM_CONFIG1) & 0x0E) >> 1
        denominator = cr_id + 4
        return denominator

    @coding_rate.setter
    def coding_rate(self, val):
        # Set coding rate (set to 5 to match RadioHead Cr45).
        denominator = min(max(val, 5), 8)
        cr_id = denominator - 4
        self._write_u8(
            _RH_RF95_REG_1D_MODEM_CONFIG1,
            (self._read_u8(_RH_RF95_REG_1D_MODEM_CONFIG1) & 0xF1) | (cr_id << 1),
        )

    @property
    def spreading_factor(self):
        """The spreading factor used by the radio (try setting to a higher
        value to increase the receiver's ability to distinguish signal from
        noise or to a lower value to increase the data transmission rate).
        Valid values are limited to 6, 7, 8, 9, 10, 11, or 12."""
        sf_id = (self._read_u8(_RH_RF95_REG_1E_MODEM_CONFIG2) & 0xF0) >> 4
        return sf_id

    @spreading_factor.setter
    def spreading_factor(self, val):
        # Set spreading factor (set to 7 to match RadioHead Sf128).
        val = min(max(val, 6), 12)
        self._write_u8(_RH_RF95_DETECTION_OPTIMIZE, 0xC5 if val == 6 else 0xC3)

        if self.signal_bandwidth >= 5000000:
            self._write_u8(_RH_RF95_DETECTION_OPTIMIZE, 0xC5 if val == 6 else 0xC3)
        else:
            # see Semtech SX1276 errata note 2.3
            self._write_u8(_RH_RF95_DETECTION_OPTIMIZE, 0x45 if val == 6 else 0x43)

        self._write_u8(_RH_RF95_DETECTION_THRESHOLD, 0x0C if val == 6 else 0x0A)
        self._write_u8(
            _RH_RF95_REG_1E_MODEM_CONFIG2,
            (
                (self._read_u8(_RH_RF95_REG_1E_MODEM_CONFIG2) & 0x0F)
                | ((val << 4) & 0xF0)
            ),
        )


    @property
    def enable_crc(self):
        """Set to True to enable hardware CRC checking of incoming packets.
        Incoming packets that fail the CRC check are not processed.  Set to
        False to disable CRC checking and process all incoming packets."""
        return (self._read_u8(_RH_RF95_REG_1E_MODEM_CONFIG2) & 0x04) == 0x04

    @enable_crc.setter
    def enable_crc(self, val):
        # Optionally enable CRC checking on incoming packets.
        if val:
            self._write_u8(
                _RH_RF95_REG_1E_MODEM_CONFIG2,
                self._read_u8(_RH_RF95_REG_1E_MODEM_CONFIG2) | 0x04,
            )
        else:
            self._write_u8(
                _RH_RF95_REG_1E_MODEM_CONFIG2,
                self._read_u8(_RH_RF95_REG_1E_MODEM_CONFIG2) & 0xFB,
            )

    def tx_done(self):
        """Transmit status"""
        # if self.dio0:
        #     print('TxDIO0: {}, {}'.format(self.dio0.value,hex((self._read_u8(_RH_RF95_REG_12_IRQ_FLAGS) & 0x8) >> 3)))
        #     return self.dio0.value
        return (self._read_u8(_RH_RF95_REG_12_IRQ_FLAGS) & 0x8) >> 3

    def rx_done(self):
        """Receive status"""
        if self.dio0:
            # print('RxDIO0: {}, {}'.format(self.dio0.value,hex((self._read_u8(_RH_RF95_REG_12_IRQ_FLAGS) & 0x40) >> 6)))
            return self.dio0.value
        else:
            return (self._read_u8(_RH_RF95_REG_12_IRQ_FLAGS) & 0x40) >> 6

    async def await_rx(self,timeout=60):
        _t=time.monotonic()+timeout
        while not self.rx_done():
            if time.monotonic() < _t:
                yield
            else:
                # Timed out
                return False
        # Received something
        return True

    def crc_error(self):
        """crc status"""
        return (self._read_u8(_RH_RF95_REG_12_IRQ_FLAGS) & 0x20) >> 5

    def send(
        self,
        data,
        *,
        keep_listening=False,
        destination=None,
        node=None,
        identifier=None,
        flags=None
    ):
        """Send a string of data using the transmitter.
           You can only send 252 bytes at a time
           (limited by chip's FIFO size and appended headers).
           This appends a 4 byte header to be compatible with the RadioHead library.
           The header defaults to using the initialized attributes:
           (destination,node,identifier,flags)
           It may be temporarily overidden via the kwargs - destination,node,identifier,flags.
           Values passed via kwargs do not alter the attribute settings.
           The keep_listening argument should be set to True if you want to start listening
           automatically after the packet is sent. The default setting is False.

           Returns: True if success or False if the send timed out.
        """
        # Disable pylint warning to not use length as a check for zero.
        # This is a puzzling warning as the below code is clearly the most
        # efficient and proper way to ensure a precondition that the provided
        # buffer be within an expected range of bounds. Disable this check.
        # pylint: disable=len-as-condition

        if hasattr(self,'txrx'):  # TX
            self.txrx[0].value=True
            self.txrx[1].value=False

        l=len(data)
        assert 0 < l <= 252
        # pylint: enable=len-as-condition
        self.idle()  # Stop receiving to clear FIFO and keep it clear.
        l+=4
        # Fill the FIFO with a packet to send.
        self._write_u8(_RH_RF95_REG_0D_FIFO_ADDR_PTR, 0x00)  # FIFO starts at 0.

        # Combine header and data to form payload
        if data == b'!':
            payload = bytearray(5)
        else:
            payload = self.buffview[:l]

        if destination is None:  # use attribute
            payload[0] = self.destination
        else:  # use kwarg
            payload[0] = destination
        if node is None:  # use attribute
            payload[1] = self.node
        else:  # use kwarg
            payload[1] = node
        if identifier is None:  # use attribute
            payload[2] = self.identifier
        else:  # use kwarg
            payload[2] = identifier
        if flags is None:  # use attribute
            payload[3] = self.flags
        else:  # use kwarg
            payload[3] = flags
        if self.DEBUG_HEADER: print('[header] - {}'.format([hex(i) for i in payload]))
        # payload = payload + data
        try:
            if isinstance(data,(bytes,bytearray,memoryview)):
                payload[4:]=data[:]
            else:
                payload[4:]=data.encode()
        except Exception as e:
            print('payload encoding error:',e)
            payload = bytearray(payload[:4])+data

        # Write payload.
        self._write_from(_RH_RF95_REG_00_FIFO, payload)
        # Write payload and header length.
        self._write_u8(_RH_RF95_REG_22_PAYLOAD_LENGTH, l)
        # Turn on transmit mode to send out the packet.
        self.transmit()
        # Wait for tx done interrupt with explicit polling (not ideal but
        # best that can be done right now without interrupts).
        start = time.monotonic()
        timed_out = False
        while not timed_out and not self.tx_done():
            if (time.monotonic() - start) >= self.xmit_timeout:
                timed_out = True

        if hasattr(self,'txrx'): # RX
            self.txrx[0].value=False
            self.txrx[1].value=True

        # Listen again if necessary and return the result packet.
        if keep_listening:
            self.listen()
        else:
            # Enter idle mode to stop receiving other packets.
            self.idle()
        # Clear interrupt.
        self._write_u8(_RH_RF95_REG_12_IRQ_FLAGS, 0xFF)
        return not timed_out

    def send_with_ack(self, data):
        """Reliable Datagram mode:
           Send a packet with data and wait for an ACK response.
           The packet header is automatically generated.
           If enabled, the packet transmission will be retried on failure
        """
        if self.ack_retries:
            retries_remaining = self.ack_retries
        else:
            retries_remaining = 1
        got_ack = False
        self.retry_counter=0 # ADDED FOR PYCUBED
        self.sequence_number = (self.sequence_number + 1) & 0xFF
        while not got_ack and retries_remaining:
            self.identifier = self.sequence_number
            self.send(data, keep_listening=True)
            # Don't look for ACK from Broadcast message
            if self.destination == _RH_BROADCAST_ADDRESS:
                print('uhf destination=RHbroadcast address (dont look for ack)')
                got_ack = True
            else:
                # wait for a packet from our destination
                ack_packet = self.receive(timeout=self.ack_wait, with_header=True)
                if ack_packet is not None:
                    if ack_packet[3] & _RH_FLAGS_ACK:
                        # check the ID
                        if ack_packet[2] == self.identifier:
                            got_ack = True
                            break
            # pause before next retry -- random delay
            if not got_ack:
                self.retry_counter+=1 # ADDED FOR PYCUBED
                print('no uhf ack, sending again...')
                # delay by random amount before next try
                time.sleep(self.ack_wait + self.ack_wait * random())
            retries_remaining = retries_remaining - 1
            # set retry flag in packet header
            self.flags |= _RH_FLAGS_RETRY
        self.flags = 0  # clear flags
        return got_ack

    # pylint: disable=too-many-branches
    def receive(
        self, *, keep_listening=True, with_header=False, with_ack=False, timeout=None, debug=False, view=False):
        """Wait to receive a packet from the receiver. If a packet is found the payload bytes
           are returned, otherwise None is returned (which indicates the timeout elapsed with no
           reception).
           If keep_listening is True (the default) the chip will immediately enter listening mode
           after reception of a packet, otherwise it will fall back to idle mode and ignore any
           future reception.
           All packets must have a 4-byte header for compatibilty with the
           RadioHead library.
           The header consists of 4 bytes (To,From,ID,Flags). The default setting will  strip
           the header before returning the packet to the caller.
           If with_header is True then the 4 byte header will be returned with the packet.
           The payload then begins at packet[4].
           If with_ack is True, send an ACK after receipt (Reliable Datagram mode)
        """
        if hasattr(self,'txrx'): # RX
            self.txrx[0].value=False
            self.txrx[1].value=True

        timed_out = False
        if timeout is None:
            timeout = self.receive_timeout
        if timeout is not None:
            # Wait for the payload_ready signal.  This is not ideal and will
            # surely miss or overflow the FIFO when packets aren't read fast
            # enough, however it's the best that can be done from Python without
            # interrupt supports.
            # Make sure we are listening for packets.
            self.listen()
            start = time.monotonic()
            timed_out = False
            while not timed_out and not self.rx_done():
                if (time.monotonic() - start) >= timeout:
                    timed_out = True
        # Payload ready is set, a packet is in the FIFO.
        packet = None
        # save last RSSI reading
        self.last_rssi = self.rssi(raw=True)
        # Enter idle mode to stop receiving other packets.
        self.idle()
        if not timed_out:
            if self.enable_crc and self.crc_error():
                self.crc_error_count += 1
                print('crc error')
                if hasattr(self,'crc_errs'):
                    self.crc_errs+=1
            else:
                # Read the data from the FIFO.
                # Read the length of the FIFO.
                fifo_length = self._read_u8(_RH_RF95_REG_13_RX_NB_BYTES)
                # Handle if the received packet is too small to include the 4 byte
                # RadioHead header and at least one byte of data --reject this packet and ignore it.
                if fifo_length > 0:  # read and clear the FIFO if anything in it
                    current_addr = self._read_u8(_RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR)
                    self._write_u8(_RH_RF95_REG_0D_FIFO_ADDR_PTR, current_addr)
                    # packet = bytearray(fifo_length)
                    packet = self.buffview[:fifo_length]
                    # Read the packet.
                    self._read_into(_RH_RF95_REG_00_FIFO, packet)
                # Clear interrupt.
                self._write_u8(_RH_RF95_REG_12_IRQ_FLAGS, 0xFF)
                if fifo_length < 5:
                    print('missing pckt header')
                    packet = None
                else:
                    if (
                        self.node != _RH_BROADCAST_ADDRESS
                        and packet[0] != _RH_BROADCAST_ADDRESS
                        and packet[0] != self.node
                    ):
                        packet = None
                    # send ACK unless this was an ACK or a broadcast
                    elif (
                        with_ack
                        and ((packet[3] & _RH_FLAGS_ACK) == 0)
                        and (packet[0] != _RH_BROADCAST_ADDRESS)
                    ):
                        # delay before sending Ack to give receiver a chance to get ready
                        if self.ack_delay is not None:
                            time.sleep(self.ack_delay)
                        # send ACK packet to sender (data is b'!')
                        self.send(
                            b"!",
                            keep_listening=keep_listening,
                            destination=packet[1],
                            node=packet[0],
                            identifier=packet[2],
                            flags=(packet[3] | _RH_FLAGS_ACK),
                        )
                        if debug: print('Sent Ack to {}'.format(packet[1]))
                        if debug: print('\t{}'.format(packet))

                        # # reject Retries if we have seen this idetifier from this source before
                        # if (self.seen_ids[packet[1]] == packet[2]) and (
                        #     packet[3] & _RH_FLAGS_RETRY
                        # ):
                        #     print('duplicate identifier from this source. rejecting...')
                        #     packet = None
                        # else:  # save the packet identifier for this source
                        self.seen_ids[packet[1]] = packet[2]
                    if (
                        not with_header and packet is not None
                    ):  # skip the header if not wanted
                        packet = packet[4:]

        if hasattr(self,'txrx'): # RX
            self.txrx[0].value=False
            self.txrx[1].value=True

        # Listen again if necessary and return the result packet.
        if keep_listening:
            self.listen()
        else:
            # Enter idle mode to stop receiving other packets.
            self.idle()
        # Clear interrupt.
        self._write_u8(_RH_RF95_REG_12_IRQ_FLAGS, 0xFF)
        if view:
            return packet
        elif packet is not None:
            return bytes(packet)
        return packet

    def receive_all(self, only_for_me=True,debug=False):
        # msg=[]
        l=0
        fifo_length=0
        self.idle()
        if self.enable_crc and self.crc_error():
            self.crc_error_count += 1
            print('crc error')
            if hasattr(self,'crc_errs'):
                self.crc_errs+=1
        else:
            fifo_length = self._read_u8(_RH_RF95_REG_13_RX_NB_BYTES)

        if fifo_length > 0:
            current_addr = self._read_u8(_RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR)
            self._write_u8(_RH_RF95_REG_0D_FIFO_ADDR_PTR, _RH_RF95_REG_00_FIFO)
            self._read_into(_RH_RF95_REG_00_FIFO,_bigbuffer)
            for i in range(4):
                self._write_from(_RH_RF95_REG_00_FIFO, bytes(64))
            self._write_u8(_RH_RF95_REG_0D_FIFO_ADDR_PTR,_RH_RF95_REG_00_FIFO)
            self.listen()
            # Clear interrupt.
            self._write_u8(_RH_RF95_REG_12_IRQ_FLAGS, 0xFF)
            packetindex=[]
            i=0
            while i < 253: # 256-4 = 252
                # check first
                if self.buffview[i] in self.valid_ids:
                    # check second
                    if self.buffview[i+1] in self.valid_ids:
                        # make sure not the same
                        if self.buffview[i]!=self.buffview[i+1]:
                            # append first
                            packetindex.append(i)
                            i=i+4
                            l+=1
                            continue
                i+=1
            for i in range(l-1):
                # assume packets are back-to-back (read till index of next one)
                # if (packetindex[i+1]-packetindex[i]) <= 10:
                    # msg.append(bytes(self.buffview[packetindex[i]:packetindex[i+1]]))
                yield self.buffview[packetindex[i]:packetindex[i+1]]
            # last packet so read until the end of our fifo_length
            if packetindex:
                # if (packetindex[-1]-(current_addr+fifo_length)) <= 10:
                    # print('{},{},{}'.format(packetindex[-1],current_addr,fifo_length))
                    # msg.append(bytes(self.buffview[packetindex[-1]:current_addr+fifo_length]))
                yield self.buffview[packetindex[-1]:current_addr+fifo_length]
        else:
            self.listen()
            # Clear interrupt.
            self._write_u8(_RH_RF95_REG_12_IRQ_FLAGS, 0xFF)

        # if only_for_me:
        #     return [i for i in msg if msg[0] is self.node]
        # else:
        #     return msg

    def send_fast(self,data,l):
        self.idle()
        self._write_u8(_RH_RF95_REG_0D_FIFO_ADDR_PTR, 0x00) # set fifo position
        # Write payload.
        self._write_from(_RH_RF95_REG_00_FIFO, data)
        # Write payload and header length.
        self._write_u8(_RH_RF95_REG_22_PAYLOAD_LENGTH, l)
        # Turn on transmit mode to send out the packet.
        self.transmit()
        # Wait for tx done interrupt with explicit polling (not ideal but
        # best that can be done right now without interrupts).
        _t = time.monotonic() + 5
        while time.monotonic() < _t and not self.tx_done():
            pass
        self.idle()
        # Clear interrupt.
        self._write_u8(_RH_RF95_REG_12_IRQ_FLAGS, 0xFF)
        return
