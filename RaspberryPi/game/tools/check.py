#!/usr/bin/env python3
"""Check syntax, imports and wire formats; does not validate robot behaviour."""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def run(label, command, cwd=ROOT):
    try:
        result = subprocess.run(command, cwd=cwd, capture_output=True, text=True,
                                encoding='utf-8', errors='replace')
    except OSError as exc:
        print(f'{label}: {exc}', file=sys.stderr)
        return False
    ok = result.returncode == 0
    print(f'{label}: {"PASS" if ok else "FAIL"}', flush=True)
    if not ok:
        print(result.stdout + result.stderr, file=sys.stderr)
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--firmware', action='store_true',
                        help='also configure/build A-board Debug; requires cmake, Ninja and ARM GCC')
    args = parser.parse_args()
    results = {}
    files = sorted(ROOT.glob('*.py'))
    for directory in ('control', 'protocol', 'Strategy', 'simulation', 'vision', 'sensors', 'utils', 'tools', 'tests'):
        files.extend(sorted(path for path in (ROOT / directory).rglob('*.py')
                            if not any(part in ('.venv', 'venv', '__pycache__')
                                       for part in path.relative_to(ROOT).parts)))
    results['syntax'] = True
    for path in files:
        try:
            compile(path.read_bytes(), str(path), 'exec')
        except (SyntaxError, OSError) as exc:
            print(exc, file=sys.stderr)
            results['syntax'] = False
    print(f'Python syntax: {"PASS" if results["syntax"] else "FAIL"} ({len(files)} files)', flush=True)
    results['imports'] = run('Imports', [sys.executable, 'tests/import_smoke.py'])
    results['run_logs'] = run('Per-run log archives', [
        sys.executable, '-m', 'unittest', 'tests.test_run_logs', '-q'])
    results['protocol'] = run('Protocol formats', [
        sys.executable, '-m', 'unittest', 'tests.test_protocol_schema',
        'tests.test_stepper_dual3', '-q'])
    results['collection'] = run('Dataset collection', [
        sys.executable, '-m', 'unittest', 'tests.test_cube_collection', '-q'])
    results['additive'] = run('Additive firmware and rollback compatibility', [
        sys.executable, '-m', 'unittest', 'tests.test_protocol_extensions',
        'tests.test_additive_firmware', 'tests.test_firmware_execution', '-q'])
    results['route'] = run('Route arrival control', [
        sys.executable, '-m', 'unittest', 'tests.test_chassis_route', '-q'])
    results['tag6'] = run('Tag6 completion control', [
        sys.executable, '-m', 'unittest', 'tests.test_tag6_completion', '-q'])
    results['strategy'] = run('Strategy composition', [
        sys.executable, '-m', 'unittest', 'tests.test_strategy_composition',
        'tests.test_plan_b', 'tests.test_functional_operations',
        'tests.test_route_speed_profiles', 'tests.test_task5', '-q'])
    results['execution'] = run('Execution and camera contracts', [
        sys.executable, '-m', 'unittest', 'tests.test_execution',
        'tests.test_action_sessions', 'tests.test_camera_pose_gate',
        'tests.test_blind_transition', '-q'])
    results['transitions'] = run('Registered pickup and inspection transitions', [
        sys.executable, '-m', 'unittest', 'tests.test_pickup_motion', 'tests.test_pickup_window_regressions', 'tests.test_managed_task',
        'tests.test_inspection_session', 'tests.test_registered_pickup_transitions',
        'tests.test_registered_inspection_transitions', 'tests.test_transition_config',
        'tests.test_transition_switches', '-q'])
    results['alignment'] = run('Pluggable fast pickup alignment', [
        sys.executable, '-m', 'unittest', 'tests.test_fast_alignment',
        'tests.test_adaptive_blind', 'tests.test_orange_lookahead',
        'tests.test_pickup_origin_regressions', '-q'])
    results['curves'] = run('Continuous trajectories and route integration', [
        sys.executable, '-m', 'unittest', 'tests.test_continuous_trajectory',
        'tests.test_curve_routes', '-q'])
    results['simulation'] = run('Offline motion and CAD geometry', [
        sys.executable, '-m', 'unittest', 'tests.test_motion_simulation',
        'tests.test_simulation_geometry', 'tests.test_simulation_model',
        'tests.test_match_simulation', 'tests.test_match_contract',
        'tests.test_match_curves', 'tests.test_match_curve_contract', '-q'])
    results['visual_fallback'] = run('Search recovery and visual fallback', [
        sys.executable, '-m', 'unittest', 'tests.test_visual_fallback', '-q'])
    results['navigation'] = run('Field navigation and moving visual localization', [
        sys.executable, '-m', 'unittest', 'tests.test_field_navigation',
        'tests.test_navigation_motion','tests.test_motion_planning_plugin','tests.test_position_tracking',
        'tests.test_competition_navigation','tests.test_recipe_navigation',
        'tests.test_local_routes','tests.test_enabled_route_regression', '-q'])
    firmware = (ROOT.parent.parent / 'Uniforest_A' / 'game'
                if ROOT.name == 'game' else ROOT.parent / 'Uniforest_A')
    if args.firmware:
        configured = run('Firmware configure', ['cmake', '--preset', 'Debug'], firmware)
        results['firmware'] = configured and run(
            'Firmware build', ['cmake', '--build', 'build/Debug'], firmware)
    print('These checks do not validate motion, vision accuracy or field calibration.', flush=True)
    return 0 if all(results.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
