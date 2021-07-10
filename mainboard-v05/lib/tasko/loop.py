import time

_monotonic_ns = time.monotonic_ns


def set_time_provider(monotonic_ns):
    global _monotonic_ns
    _monotonic_ns = monotonic_ns


def _yield_once():
    """await the return value of this function to yield the processor"""

    class _CallMeNextTime:
        def __await__(self):
            # This is inside the scheduler where we know generator yield is the
            #   implementation of task switching in CircuitPython.  This throws
            #   control back out through user code and up to the scheduler's
            #   __iter__ stack which will see that we've suspended _current.
            # Don't yield in async methods; only await unless you're making a library.
            yield

    return _CallMeNextTime()


def _get_future_nanos(seconds_in_future):
    return _monotonic_ns() + int(seconds_in_future * 1000000000)


class Sleeper:
    def __init__(self, resume_nanos, task):
        self.task = task
        self._resume_nanos = resume_nanos

    def resume_nanos(self):
        return self._resume_nanos

    def priority_sort(self):
        return self.task.priority

    def __repr__(self):
        return "{{Sleeper remaining: {:.2f}, task: {} }}".format(
            (self.resume_nanos() - _monotonic_ns()) , self.task
        )

    __str__ = __repr__


class Task:
    def __init__(self, coroutine, priority):
        # Added a priority level
        self.coroutine = coroutine
        self.priority = priority

    def priority_sort(self):
        return self.priority

    def __repr__(self):
        return "{{Task {}, Priority {}}}".format(self.coroutine, self.priority)

    __str__ = __repr__


class ScheduledTask:
    def change_rate(self, hz):
        ### Update the task rate to a new frequency ###
        self._nanoseconds_per_invocation = (1 / hz) * 1000000000

    def stop(self):
        ### Stop the task (does not interrupt a currently running task) ###
        self._stop = True

    def start(self):
        ### Schedule the task (if it's not already scheduled) ###
        self._stop = False
        if not self._scheduled_to_run:
            # Don't double-up the task if it's still in the run list!
            # print("Added task to loop._task")
            self._loop.add_task(self._run_at_fixed_rate(), self._priority)

    def __init__(
        self, loop, hz, forward_async_fn, priority, forward_args, forward_kwargs
    ):
        self._loop = loop
        self._forward_async_fn = forward_async_fn
        self._forward_args = forward_args
        self._forward_kwargs = forward_kwargs
        self._nanoseconds_per_invocation = (1 / hz) * 1000000000
        self._stop = False
        self._running = False
        self._scheduled_to_run = False
        self._priority = priority

    async def _run_at_fixed_rate(self):
        self._scheduled_to_run = True
        try:
            target_run_nanos = _monotonic_ns()
            while True:
                if self._stop:
                    return  # Check before running

                iteration = self._forward_async_fn(
                    *self._forward_args, **self._forward_kwargs
                )
                self._loop._debug("  iteration ", iteration)

                self._running = True
                try:
                    await iteration
                finally:
                    self._running = False

                if self._stop:
                    return  # Check before waiting

                # Try to reschedule for the next window without skew. If we're falling behind,
                # just go as fast as possible & schedule to run "now." If we catch back up again
                # we'll return to seconds_per_invocation without doing a bunch of catchup runs.
                target_run_nanos = target_run_nanos + self._nanoseconds_per_invocation
                # print('target_run_nanos is ', target_run_nanos)
                now_nanos = _monotonic_ns()
                if now_nanos <= target_run_nanos:
                    # print("Going to put to sleep")
                    await self._loop._sleep_until_nanos(target_run_nanos)
                else:
                    target_run_nanos = now_nanos
                    # Allow other tasks a chance to run if this task is too slow.
                    await _yield_once()
        finally:
            self._scheduled_to_run = False

    def __repr__(self):
        hz = 1 / (self._nanoseconds_per_invocation / 1000000000)
        state = "running" if self._running else "waiting"
        return "{{ScheduledTask {} rate: {}hz, fn: {}}}".format(
            state, hz, self._forward_async_fn
        )

    __str__ = __repr__


class TaskCanceledException(Exception):
    pass


class Loop:
    """
    It's your task host.  You run() it and it manages your main application loop.
    """

    def __init__(self, debug=False):
        self._tasks = []
        self._sleeping = []
        self._ready = []
        self._current = None
        self.debug=debug
        if debug:
            self._debug = print
        else:
            self._debug = lambda *arg, **kwargs: None

    def add_task(self, awaitable_task, priority):
        """
        Add a concurrent task (known as a coroutine, implemented as a generator in CircuitPython)
        Use:
          scheduler.add_task( my_async_method() )
        :param awaitable_task:  The coroutine to be concurrently driven to completion.
        """
        self._debug("adding task ", awaitable_task)
        # Added a priority parameter
        self._tasks.append(Task(awaitable_task, priority))

    async def sleep(self, seconds):
        """
        From within a coroutine, this suspends your call stack for some amount of time.

        NOTE:  Always `await` this!  You will have a bad time if you do not.

        :param seconds: Floating point; will wait at least this long to call your task again.
        """
        await self._sleep_until_nanos(_get_future_nanos(seconds))

    def run_later(self, seconds_to_delay, awaitable_task, priority):
        """
        Add a concurrent task, delayed by some seconds.
        Use:
          tasko.run_later( seconds_to_delay=1.2, my_async_method() )
        :param seconds_to_delay: How long until the task should be kicked off?
        :param awaitable_task:   The coroutine to be concurrently driven to completion.
        """
        # Make sure we don't wait unnecessarily if there are lots of tasks to kick off
        start_nanos = _get_future_nanos(seconds_to_delay)

        async def _run_later():
            await self._sleep_until_nanos(start_nanos)
            await awaitable_task

        # Added a priority parameter
        self.add_task(_run_later(), priority)

    def suspend(self):
        """
        For making library functions that suspend and then resume later on some condition
        E.g., a scope manager for SPI

        To use this you will stash the resumer somewhere to call from another coroutine, AND
        you will `await suspender` to pause this stack at the spot you choose.

        :returns (async_suspender, resumer)
        """
        assert (
            self._current is not None
        ), "You can only suspend the current task if you are running the event loop."
        suspended = self._current

        def resume():
            self._tasks.append(suspended)

        self._current = None
        return _yield_once(), resume

    def schedule(self, hz: float, coroutine_function, priority, *args, **kwargs):
        """
        Describe how often a method should be called.

        Your event loop will call this coroutine on the hz schedule.
        Only up to 1 instance of your method will be alive at a time.

        This will use sleep() internally when there's nothing to do
        and scheduled, waiting functions consume no cpu so you should
        feel pretty good about using scheduled async functions.

        usage:
          async def main_loop:
            await your_code()
          scheduled_task = get_loop().schedule(hz=100, coroutine_function=main_loop)
          get_loop().run()

        :param hz: How many times per second should the function run?
        :param coroutine_function: the async def function you want invoked on your schedule
        :param event_loop: An event loop that can .sleep() and .add_task.  Like BudgetEventLoop.
        """
        assert coroutine_function is not None, "coroutine function must not be none"
        task = ScheduledTask(self, hz, coroutine_function, priority, args, kwargs)
        task.start()
        return task

    def schedule_later(self, hz: float, coroutine_function, priority, *args, **kwargs):
        """
        Like schedule, but invokes the coroutine_function after the first hz interval.

        See schedule api for parameters.
        """
        ran_once = False

        async def call_later():
            nonlocal ran_once
            if ran_once:
                await coroutine_function(*args, **kwargs)
            else:
                await _yield_once()
                ran_once = True

        return self.schedule(hz, call_later, priority)

    def run(self):
        """
        Use:
            async def application_loop():
              pass

            def run():
              main_loop = Loop()
              loop.schedule(100, application_loop)
              loop.run()

            if __name__ == '__main__':
              run()
        The crucial StopIteration exception signifies the end of a coroutine in CircuitPython.
        Other Exceptions that reach the runner break out, stopping your app and showing a stack trace.
        """

        assert (
            self._current is None
        ), "Loop can only be advanced by 1 stack frame at a time."
        self._loopnum = 0
        while self._tasks or self._sleeping:
            self._debug(
                "[{}] ---- sleeping: {}, active: {}".format(
                    self._loopnum, len(self._sleeping), len(self._tasks)
                )
            )
            self._step()
            self._debug("\n")
            self._loopnum += 1
        # while self._tasks or self._sleeping:
        # self._step()
        self._debug("Loop completed", self._tasks, self._sleeping)

    def _step(self):
        self._debug("  stepping over ", len(self._tasks), " tasks")

        # Sort tasks by priority
        self._tasks.sort(key=Task.priority_sort)

        for _ in range(len(self._tasks)):
            task = self._tasks.pop(0)
            self._run_task(task)

        if self.debug:
            self._debug("  sleeping list (unsorted):")
            for i in self._sleeping:
                self._debug("    {}".format(i))

        #Create the ready list based on whichever tasks are ready to be executed
        self._ready = [x for x in self._sleeping if x.resume_nanos() <= _monotonic_ns()]
        #Sort the ready tasks based on priority
        self._ready.sort(key=lambda x: x.task.priority)

        if self.debug:
            self._debug("  ready list (sorted)")
            for i in self._ready:
                self._debug("    {}".format(i))


        #Run the ready tasks, and simultaneously remove them from the _sleeping and _ready lists
        for i in range(len(self._ready)):
            ready_task = self._ready.pop(0)
            self._sleeping.remove(ready_task)
            self._run_task(ready_task.task)

        if len(self._tasks) == 0 and len(self._sleeping) > 0:

            #Sort the sleeper list only when we need to, i.e, only
            #when there are no active tasks, and the system needs to
            #ACTUALLY sleep.

            self._sleeping.sort(key=Sleeper.resume_nanos)
            next_sleeper = self._sleeping[0]
            sleep_nanos = next_sleeper.resume_nanos() - _monotonic_ns()

            if sleep_nanos > 0:
                # Give control to the system, there's nothing to be done right now,
                # and nothing else is scheduled to run for this long.
                # This is the real sleep. If/when interrupts are implemented this will likely need to change.
                sleep_seconds = sleep_nanos / 1000000000.0
                self._debug(
                    "  No active tasks.  Sleeping for ",
                    sleep_seconds,
                    "s. \n",
                    self._sleeping,
                )

                time.sleep(sleep_seconds)

    def _run_task(self, task: Task):
        """
        Runs a task and re-queues for the next loop if it is both (1) not complete and (2) not sleeping.
        """

        self._current = task
        try:

            task.coroutine.send(None)
            self._debug("  current", self._current)
            # Sleep gate here, in case the current task suspended.
            # If a sleeping task re-suspends it will have already put itself in the sleeping queue.
            if self._current is not None:
                self._tasks.append(task)
        except StopIteration:
            # This task is all done.
            self._debug("  task complete")
            pass
        finally:
            self._current = None

    async def _sleep_until_nanos(self, target_run_nanos):
        """
        From within a coroutine, sleeps until the target time.monotonic_ns
        Returns the thing to await
        """
        assert self._current is not None, "You can only sleep from within a task"
        self._sleeping.append(Sleeper(target_run_nanos, self._current))
        self._debug("  sleeping ", self._current)
        self._current = None
        # Pretty subtle here.  This yields once, then it continues next time the task scheduler executes it.
        # The async function is parked at this point.
        await _yield_once()

