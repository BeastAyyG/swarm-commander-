import collections
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping
    collections.Mapping = collections.abc.Mapping
    collections.Sequence = collections.abc.Sequence
    collections.Iterable = collections.abc.Iterable
    collections.Callable = collections.abc.Callable

from dronekit import connect
import time

print("Connecting...")
try:
    vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True, timeout=30)
    print(f"Connected! Mode: {vehicle.mode.name}")
    print(f"Location: {vehicle.location.global_relative_frame}")
    print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
    vehicle.close()
except Exception as e:
    print(f"Error: {e}")
