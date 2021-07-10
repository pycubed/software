from .loop import Loop

# Enable logging by setting builtins.tasko_logging = True before importing the first time.
#
# import builtins
# builtins.tasko_logging = True
# import tasko

__global_event_loop = None

try:
    global tasko_logging
    if tasko_logging:
        print('Enabling tasko instrumentation')
except NameError:
    # Set False by default to skip debug logging
    tasko_logging = False


def get_loop(debug=tasko_logging):
    """Returns the singleton event loop"""
    global __global_event_loop
    if __global_event_loop is None:
        __global_event_loop = Loop(debug=debug)
    return __global_event_loop


add_task = get_loop().add_task
run_later = get_loop().run_later
schedule = get_loop().schedule
schedule_later = get_loop().schedule_later
sleep = get_loop().sleep
suspend = get_loop().suspend

run = get_loop().run
