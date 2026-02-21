#!/usr/bin/env python3
"""
test_sitl_connection.py — Verify SITL connectivity
====================================================
Quick test to ensure ArduCopter SITL is running and DroneKit can connect.
"""

import collections
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping
    collections.Mapping = collections.abc.Mapping
    collections.Sequence = collections.abc.Sequence
    collections.Iterable = collections.abc.Iterable
    collections.Callable = collections.abc.Callable

import sys
import time


def test_connection(port=5760, timeout=30):
    """Test connection to a single SITL instance."""
    from dronekit import connect
    
    conn_str = f"tcp:127.0.0.1:{port}"
    print(f"Testing connection to {conn_str}...")
    
    try:
        vehicle = connect(conn_str, wait_ready=True, timeout=timeout)
        print(f"  ✓ Connected!")
        print(f"    Firmware: {vehicle.version}")
        print(f"    Mode: {vehicle.mode.name}")
        print(f"    Armed: {vehicle.armed}")
        print(f"    GPS: {vehicle.gps_0}")
        loc = vehicle.location.global_relative_frame
        print(f"    Position: ({loc.lat:.6f}, {loc.lon:.6f}, {loc.alt:.1f}m)")
        vehicle.close()
        return True
    except Exception as e:
        print(f"  ✗ Failed: {e}")
        return False


def main():
    ports = [5760, 5770, 5780, 5790, 5800]
    num = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    
    print(f"Testing {num} SITL instances...\n")
    
    results = []
    for i in range(num):
        ok = test_connection(ports[i])
        results.append(ok)
        print()
    
    passed = sum(results)
    print(f"Results: {passed}/{num} connected successfully")
    sys.exit(0 if passed == num else 1)


if __name__ == "__main__":
    main()
