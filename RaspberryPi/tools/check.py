#!/usr/bin/env python3
"""Run repeatable offline checks without opening robot hardware."""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def run(label, command, cwd=ROOT):
    print(f'\n=== {label} ===', flush=True)
    try:
        result = subprocess.run(command, cwd=cwd)
    except OSError as exc:
        print(f'{label}: {exc}', file=sys.stderr)
        return False
    return result.returncode == 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--module', action='append', default=[],
                        help='unittest module/name; repeat to select checks (default: full suite)')
    parser.add_argument('--firmware', action='store_true',
                        help='also configure/build A-board Debug; requires cmake, Ninja and ARM GCC')
    parser.add_argument('--native-actions', action='store_true',
                        help='also run C action tests; requires native cc or CC')
    args = parser.parse_args()
    results = {}
    files = sorted(ROOT.glob('*.py'))
    for directory in ('control', 'protocol', 'Strategy', 'vision', 'sensors', 'utils', 'tools', 'tests'):
        files.extend(sorted((ROOT / directory).rglob('*.py')))
    print('=== Python syntax ===', flush=True)
    results['syntax'] = True
    for path in files:
        try:
            compile(path.read_bytes(), str(path), 'exec')
        except (SyntaxError, OSError) as exc:
            print(exc, file=sys.stderr)
            results['syntax'] = False
    results['imports'] = run('Imports', [sys.executable, 'tests/import_smoke.py'])
    tests = ([*args.module, '-b', '-v'] if args.module else
             ['discover', '-s', 'tests', '-t', '.', '-b', '-v'])
    results['tests'] = run('Python tests', [sys.executable, '-m', 'unittest', *tests])
    firmware = ROOT.parent / 'Uniforest_A'
    if args.firmware:
        configured = run('Firmware configure', ['cmake', '--preset', 'Debug'], firmware)
        results['firmware'] = configured and run(
            'Firmware build', ['cmake', '--build', 'build/Debug'], firmware)
    if args.native_actions:
        results['native_actions'] = run(
            'C action tests', [sys.executable, str(firmware / 'tests/run_actions_host.py')])
    print('\n' + ', '.join(f'{name}={"PASS" if ok else "FAIL"}'
                           for name, ok in results.items()), flush=True)
    return 0 if all(results.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
