from tasko.loop import _yield_once, set_time_provider
import time
from unittest import TestCase

from tasko import Loop


class TestLoop(TestCase):
    def test_add_task(self):
        loop = Loop()
        ran = False

        async def foo():
            nonlocal ran
            ran = True
        loop.add_task(foo())
        loop._step()
        self.assertTrue(ran)

    def test_sleep(self):
        loop = Loop()
        complete = False

        async def foo():
            nonlocal complete
            await loop.sleep(0.1)
            complete = True
        loop.add_task(foo())
        start = time.monotonic()
        while not complete and time.monotonic() - start < 1:
            loop._step()
        self.assertTrue(complete)

    def test_reschedule(self):
        now = 0
        def nanos():
            nonlocal now
            return now

        set_time_provider(nanos)
        try:
            loop = Loop()
            run_count = 0

            async def foo():
                nonlocal run_count
                run_count += 1
            scheduled_task = loop.schedule(1000000000, foo)

            now = 2
            self.assertEqual(0, run_count, 'did not run before step')
            loop._step()
            self.assertEqual(1, run_count, 'ran only once during step')

            now = 4
            scheduled_task.stop()
            loop._step()
            self.assertEqual(1, run_count, 'does not run again while stopped')

            now = 6
            scheduled_task.start()
            loop._step()
            self.assertEqual(2, run_count, 'runs again after restarting')

            now = 7
            scheduled_task.change_rate(1000000000 / 10)
            loop._step()
            self.assertEqual(3, run_count, 'this run was already scheduled. the next one will be at 10-step')

            now = 16
            loop._step()
            self.assertEqual(3, run_count, 'expect to run after 10 has passed')

            now = 17
            loop._step()
            self.assertEqual(4, run_count, 'new schedule rate ran')
        finally:
            set_time_provider(time.monotonic_ns)

    def test_schedule_rate(self):
        # Checks a bunch of scheduled tasks to make sure they hit their target fixed rate schedule.
        # Pathological scheduling sees these tasks barge in front of others all the time. Many run
        # at a fairly high frequency.
        #
        # 98 tasks => 15khz throughput tested.
        #   Works reliably on macbook pro; microcontroller throughput will be.. somewhat less.

        loop = Loop(debug=False)
        duration = 1  # Seconds to run the scheduler. (try with 10s if you suspect scheduler drift)
        tasks = 96    # How many tasks (higher indexes count faster. 96th task => 293hz)

        counters = []
        for i in range(tasks):
            async def f(_i):
                counters[_i] += 1

            counters.append(0)

            loop.schedule(3 * (i + 1) + 5, f, i)

        start = time.monotonic()
        while time.monotonic() - start < duration:
            loop._step()

        expected_tps = 0
        actual_tps = 0
        # Assert that all the tasks hit their scheduled count, at least within +-5 iterations.
        for i in range(len(counters)):
            self.assertAlmostEqual(duration * (3*(i+1) + 5), counters[i], delta=5)
            expected_tps += (3*(i+1) + 5)
            actual_tps += counters[i]
        actual_tps /= duration
        print('expected tps:', expected_tps, 'actual:', actual_tps)
    
    def test_schedule_later(self):
        control_ticks = 0
        deferred_ticks = 0
        deferred_ticked = False
        loop = Loop(debug=False)

        async def deferred_task():
            nonlocal deferred_ticked, deferred_ticks
            deferred_ticks = deferred_ticks + 1
            deferred_ticked = True

        async def control_ticker():
            nonlocal control_ticks
            control_ticks = control_ticks + 1

        loop.schedule(100, control_ticker)
        loop.schedule_later(10, deferred_task)

        while True:
            loop._step()
            if deferred_ticked:
                break

        self.assertEqual(deferred_ticks, 1)
        self.assertAlmostEqual(control_ticks, 10, delta=2)
    
    def test_run_later(self):
        loop = Loop()
        count = 0

        async def run_later():
            nonlocal count
            while True:
                count = count + 1
                await _yield_once()  # For testing

        loop.run_later(seconds_to_delay=0.1, awaitable_task=run_later())

        self.assertEqual(0, count, 'count should not increment upon coroutine instantiation')
        loop._step()
        self.assertEqual(0, count, 'count should not increment before waiting long enough')
        
        time.sleep(0.1)  # Make sure enough time has passed for step to pick up the task
        loop._step()
        self.assertEqual(1, count, 'count should increment once per step')
