from unittest import TestCase

from tasko.managed_spi import ManagedSpi
from tasko import Loop


# This is a terrible pattern, used only for tests
class YieldOne:
    def __await__(self):
        yield


class FakeDigitalIO:
    def __init__(self, id):
        self.id = id
        self.value = False


class TestManagedSpi(TestCase):
    def test_acquire(self):
        loop = Loop()

        # Synchronize access to an SPI allowing other tasks to work while waiting
        spi_bus = 'board.SPI'
        managed_spi = ManagedSpi(spi_bus, loop=loop)

        # Configure 3 pins for selecting different chip selects on the shared SPI bus
        sdcard_spi = managed_spi.cs_handle(FakeDigitalIO('D1'))
        screen_spi = managed_spi.cs_handle(FakeDigitalIO('D2'))
        sensor_spi = managed_spi.cs_handle(FakeDigitalIO('D3'))

        did_read = did_screen = did_sensor = False

        # Define 3 full while True app loops dependent on 1 shared SPI

        async def read_sdcard():
            nonlocal did_read
            while True:
                async with sdcard_spi as spi:
                    # do_something_with(spi)
                    for _ in range(2):
                        self.assertTrue(sdcard_spi.active)
                        self.assertFalse(screen_spi.active)
                        self.assertFalse(sensor_spi.active)
                        await YieldOne()
                did_read = True
                await YieldOne()
        
        async def update_screen():
            nonlocal did_screen
            while True:
                # await do_other_work
                async with screen_spi as spi:
                    for _ in range(2):
                        self.assertFalse(sdcard_spi.active)
                        self.assertTrue(screen_spi.active)
                        self.assertFalse(sensor_spi.active)
                        await YieldOne()
                did_screen = True
                await YieldOne()
        
        async def read_sensor():
            nonlocal did_sensor
            while True:
                async with sensor_spi as spi:
                    for _ in range(2):
                        self.assertFalse(sdcard_spi.active)
                        self.assertFalse(screen_spi.active)
                        self.assertTrue(sensor_spi.active)
                        await YieldOne()
                did_sensor = True
                await YieldOne()

        # Add the top level application coroutines
        loop.add_task(read_sdcard())
        loop.add_task(read_sensor())
        loop.add_task(update_screen())

        # would just use tasko.add_task() and tasko.run() but for test let's manually step it through
        # loop.run()

        # They didn't run yet
        self.assertFalse(sdcard_spi.active)
        self.assertFalse(screen_spi.active)
        self.assertFalse(sensor_spi.active)

        loop._step()
        self.assertTrue(sdcard_spi.active) # sdcard was the first to queue up
        loop._step()
        loop._step()

        loop._step()
        self.assertTrue(sensor_spi.active) # sensor was the second to queue up
        loop._step()
        loop._step()

        loop._step()
        self.assertTrue(screen_spi.active) # screen was the last to queue up
        loop._step()
        loop._step()

        self.assertTrue(did_sensor)
        self.assertTrue(did_screen)
        self.assertTrue(did_read)
