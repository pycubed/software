import tasko


class ManagedResource:
    """
    Manages a singleton resource with your functions that initialize a resource and clean it up between uses.

    This class vends access to `resource` via a fair queue.  Intended use is with something like a busio.SPI
    with on_acquire setting a chip select pin and on_release resetting that pin.

    A ManagedResource instance should be shared among all users of `resource`.
    """
    def __init__(self, resource, on_acquire=lambda *args, **kwargs: None, on_release=lambda *args, **kwargs: None, loop=tasko.get_loop()):
        """
        :param resource: The resource you want to manage access to (e.g., a busio.SPI)
        :param on_acquire: function(*args, **kwargs) => void  acquires your singleton resource (CS pin low or something)
        :param on_release: function(*args, **kwargs) => void  releases your singleton resource (CS pin high or something)
        """
        self._resource = resource
        self._on_acquire = on_acquire
        self._on_release = on_release
        self._loop = loop
        self._ownership_queue = []
        self._owned = False

    def handle(self, *args, **kwargs):
        """
        returns a reusable, reentrant handle to the managed resource.
        args and kwargs are passed to on_acquire and on_release functions you provided with the resource.
        """
        return Handle(self, args, kwargs)

    async def _aenter(self, args, kwargs):
        if self._owned:
            # queue up for access to the resource later
            await_handle, resume_fn = self._loop.suspend()
            self._ownership_queue.append(resume_fn)
            # This leverages the suspend() feature in tasko; this current coroutine is not considered again until
            # the owning job is complete and __aexit__s below.  This keeps waiting handles as cheap as possible.
            await await_handle
        self._owned = True
        self._on_acquire(*args, **kwargs)
        return self._resource

    async def _aexit(self, args, kwargs):
        assert self._owned, 'Exited from a context where a managed resource was not owned'
        self._on_release(*args, **kwargs)
        if len(self._ownership_queue) > 0:
            resume_fn = self._ownership_queue.pop(0)
            # Note that the awaiter has already passed the ownership check.
            # By not resetting to unowned here we avoid unfair resource starvation in certain code constructs.
            resume_fn()
        else:
            self._owned = False

class Handle:
    """
    For binding resource initialization/teardown args to a resource.
    """
    def __init__(self, managed_resource, args, kwargs):
        self._managed_resource = managed_resource
        self._args = args
        self._kwargs = kwargs
        self.active = False

    async def __aenter__(self):
        resource = await self._managed_resource._aenter(self._args, self._kwargs)
        self.active = True
        return resource

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        resource = await self._managed_resource._aexit(self._args, self._kwargs)
        self.active = False
        return resource
