#!/usr/bin/env python3
"""Dependency/import smoke test: python tests/import_smoke.py."""
import sys, os

# Add RaspberryPi to path if not already there
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ['PYTHONIOENCODING'] = 'utf-8'

errors = []

def test(name, module):
    try:
        __import__(module)
        print(f'  {name:30s} OK')
    except Exception as e:
        print(f'  {name:30s} FAIL: {e}')
        errors.append(name)

# Dependencies
print('=== Dependencies ===')
try:
    import serial; print(f'  pyserial {serial.__version__:20s} OK')
except Exception as e:
    print(f'  pyserial {"":20s} FAIL: {e}'); errors.append('pyserial')
try:
    import numpy; print(f'  numpy {numpy.__version__:22s} OK')
except Exception as e:
    print(f'  numpy {"":22s} FAIL: {e}'); errors.append('numpy')
try:
    import cv2; print(f'  opencv {cv2.__version__:21s} OK')
except Exception as e:
    print(f'  opencv {"":21s} FAIL: {e}'); errors.append('opencv')
try:
    # Importing pynput opens the X backend on Linux and fails in a headless SSH
    # session even when the package is installed. Check package metadata here;
    # control.keyboard_control is imported below for the real code-path check.
    from importlib.metadata import version
    print(f'  pynput {version("pynput"):21s} OK')
except Exception as e:
    print(f'  pynput {"":21s} FAIL: {e}'); errors.append('pynput')

# Utils
print('\n=== Utils ===')
test('utils.crc16', 'utils.crc16')

# Protocol
print('\n=== Protocol ===')
test('protocol.commands', 'protocol.commands')
test('protocol.transport', 'protocol.transport')

# Control
print('\n=== Control ===')
test('control.chassis', 'control.chassis')
test('control.servo', 'control.servo')
test('control.stepper', 'control.stepper')
test('control.actions', 'control.actions')
test('control.keyboard_control', 'control.keyboard_control')

# Vision
print('\n=== Vision ===')
test('vision.cube_detector', 'vision.cube_detector')
test('vision.field_localizer', 'vision.field_localizer')

# Competition architecture
print('\n=== Competition ===')
test('robot', 'robot')
test('Strategy.task0', 'Strategy.task0')
test('Strategy.competition', 'Strategy.competition')

# Main syntax
print('\n=== Main ===')
try:
    with open(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'main.py'), encoding='utf-8') as f:
        compile(f.read(), 'main.py', 'exec')
    print('  main.py syntax                OK')
except Exception as e:
    print(f'  main.py syntax                FAIL: {e}')
    errors.append('main.py')

print()
if errors:
    print(f'FAILED: {errors}')
else:
    print('=== ALL MODULES LOADED SUCCESSFULLY ===')
