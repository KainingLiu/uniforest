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

# Utils
print('\n=== Utils ===')
test('utils.crc16', 'utils.crc16')
test('utils.run_logs', 'utils.run_logs')
test('utils.diagnostics', 'utils.diagnostics')

# Protocol
print('\n=== Protocol ===')
test('protocol.commands', 'protocol.commands')
test('protocol.transport', 'protocol.transport')
test('protocol.extensions', 'protocol.extensions')

# Control
print('\n=== Control ===')
test('control.chassis', 'control.chassis')
test('control.servo', 'control.servo')
test('control.stepper', 'control.stepper')
test('control.actions', 'control.actions')
test('control.trajectory', 'control.trajectory')
test('control.carried_cube_inspection', 'control.carried_cube_inspection')

# Vision
print('\n=== Vision ===')
test('vision.cube_detector', 'vision.cube_detector')
test('vision.orange_lookahead', 'vision.opencv.orange_lookahead')
test('vision.field_localizer', 'vision.field_localizer')
test('vision.opencv.camera_tuner', 'vision.opencv.camera_tuner')
test('vision.yolo.collector', 'vision.yolo.collector')

# Competition architecture
print('\n=== Competition ===')
test('robot', 'robot')
test('Strategy.plans', 'Strategy.plans')
test('Strategy.runner', 'Strategy.runner')
test('Strategy.controllers','Strategy.controllers')
test('Strategy.flows.factory','Strategy.flows.factory')
test('Strategy.execution', 'Strategy.execution')
test('Strategy.execution.blind', 'Strategy.execution.blind')
test('Strategy.execution.pickup_motion', 'Strategy.execution.pickup_motion')
test('Strategy.flows', 'Strategy.flows')
test('Strategy.flows.transitions', 'Strategy.flows.transitions')
test('Strategy.flows.curves', 'Strategy.flows.curves')
test('Strategy.transition_config', 'Strategy.transition_config')
test('Strategy.transition_switches', 'Strategy.transition_switches')
test('Strategy.cli', 'Strategy.cli')
test('Strategy.navigation', 'Strategy.navigation')
test('navigation.motion', 'Strategy.navigation.motion')
test('navigation.tracking', 'Strategy.navigation.tracking')
test('Strategy.motion_planning', 'Strategy.optimizations.motion_planning')
test('Strategy.local_routes', 'Strategy.optimizations.local_routes')
test('Strategy.recipe_navigation', 'Strategy.navigation.recipe')
test('Strategy.route_odometry', 'Strategy.navigation.odometry')
test('navigation.robot_adapter', 'Strategy.navigation.robot_adapter')
test('navigation.competition', 'Strategy.navigation.competition')
test('Strategy.fast_alignment', 'Strategy.optimizations.fast_alignment')
test('Strategy.adaptive_blind', 'Strategy.optimizations.adaptive_blind')

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

sys.exit(1 if errors else 0)
