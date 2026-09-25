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
    for directory in ('control', 'protocol', 'Strategy', 'vision', 'sensors', 'utils', 'tools', 'tests'):
        files.extend(sorted((ROOT / directory).rglob('*.py')))
    results['syntax'] = True
    for path in files:
        try:
            compile(path.read_bytes(), str(path), 'exec')
        except (SyntaxError, OSError) as exc:
            print(exc, file=sys.stderr)
            results['syntax'] = False
    print(f'Python syntax: {"PASS" if results["syntax"] else "FAIL"} ({len(files)} files)', flush=True)
    results['imports'] = run('Imports', [sys.executable, 'tests/import_smoke.py'])
    results['protocol'] = run('Protocol formats', [
        sys.executable, '-m', 'unittest', 'tests.test_protocol_schema',
        'tests.test_stepper_dual3', '-q'])
    results['collection'] = run('Dataset collection', [
        sys.executable, '-m', 'unittest', 'tests.test_cube_collection', '-q'])
    firmware = ROOT.parent / 'Uniforest_A'
    if args.firmware:
        configured = run('Firmware configure', ['cmake', '--preset', 'Debug'], firmware)
        results['firmware'] = configured and run(
            'Firmware build', ['cmake', '--build', 'build/Debug'], firmware)
    print('These checks do not validate motion, vision accuracy or field calibration.', flush=True)
    return 0 if all(results.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
