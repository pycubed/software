from .managed_resource import ManagedResource
import tasko

class ManagedSpi:
    def __init__(self, spi_bus, loop=tasko.get_loop()):
        """
        Vends access to an SPI bus via chip select leases.
        """
        self._resource = ManagedResource(spi_bus, on_acquire=self._acquire_spi, on_release=self._release_spi, loop=loop)
        self._handles = {}
    
    def _acquire_spi(self, chip_select):
        chip_select.value = False

    def _release_spi(self, chip_select):
        chip_select.value = True
    
    def cs_handle(self, chip_select):
        """
        pass in a digitalio.DigitalInOut chip select.
        This will be pulled low when a SpiHandle acquires the bus.

        Store 1 handle for each chip select you want to manage with a shared SPI.

        You can read or write batches of data from an sd card while sending updates to a display
        between read batches, while reading sensor data from 3 different sensors on a timer on a single
        spi bus - without coordinating between them. Only await the hande for each task's turn with the bus.

        You need to:
          * configure the bus to work with your devices
          * configure the chip select pins as outputs
        
        You get:
          * non-blocking, awaitable access to an SPI
        """
        chip_select.value = True
        spi_handle = self._resource.handle(chip_select=chip_select)
        return spi_handle
