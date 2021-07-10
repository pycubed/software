from unittest import TestCase

from tasko.managed_resource import ManagedResource
from tasko import Loop


class Resource:
    def __init__(self):
        self.active_cs = None

    def acquire(self, chip_select):
        assert self.active_cs is None, 'cannot acquire owned resource.  Owned cs: ' + self.active_cs + ', requested cs: ' + chip_select
        self.active_cs = chip_select
        return self

    def release(self, chip_select):
        assert self.active_cs is not None, 'cannot release unowned resource.  Owned cs: ' + self.active_cs + ', requested cs: ' + chip_select
        self.active_cs = None


class YieldOne:
    def __await__(self):
        yield


class TestManagedResource(TestCase):
    def test_acquire(self):
        loop = Loop()
        spi = Resource()
        managed_spi = ManagedResource(spi, spi.acquire, spi.release, loop=loop)
        handle_cs1 = managed_spi.handle(chip_select=1)
        handle_cs2 = managed_spi.handle(chip_select=2)
        self.assertIsNone(spi.active_cs)

        async def test_fn(managed_resource):     # Stop points
            await YieldOne()                     # Enter fn
            async with managed_resource as spi:  # acquire/suspend
                await YieldOne()                 # work
                self.assertTrue(spi.active_cs is not None)
            await YieldOne()                     # after context

        loop.add_task(test_fn(handle_cs1))
        loop.add_task(test_fn(handle_cs2))

        loop._step()  # 1 Enter fn      2 Enter fn
        loop._step()  # 1 acquire-work  2 suspend
        # 1 is working with spi on cs1, 2 is suspended waiting.
        self.assertIs(spi.active_cs, 1)
        self.assertEqual(len(loop._tasks), 1)  # 2 is suspended, not eligible to be run next step

        loop._step()  # 1 after context
        self.assertIsNone(spi.active_cs)
        self.assertEqual(len(loop._tasks), 2)  # 1 is unfinished, 2 is unsuspended by the ManagedResource

        loop._step()  # 1 end           2 work
        self.assertIs(spi.active_cs, 2)
        self.assertEqual(len(loop._tasks), 1)  # 1 is finished, 2 is working with spi on cs2

        loop._step()  #                 2 after context
        self.assertIsNone(spi.active_cs)
        self.assertEqual(len(loop._tasks), 1)  # 2 is unfinished

        loop._step()  # 2 end
        self.assertEqual(loop._tasks, [])  # 2 is finished
