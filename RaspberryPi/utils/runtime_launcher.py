"""Select an isolated runtime before importing any hardware-facing module."""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
ENTRIES = {'main': 'main', 'task2_main': 'main', 'robot': 'debug_main'}


def split_runtime(arguments):
    """Consume only the exact new option; leave all legacy arguments intact."""
    remaining = []
    runtime = None
    index = 0
    while index < len(arguments):
        item = arguments[index]
        if item == '--':
            remaining.extend(arguments[index:])
            break
        if item == '--runtime' or item.startswith('--runtime='):
            if runtime is not None:
                raise ValueError('--runtime may be specified only once')
            if item == '--runtime':
                index += 1
                if index == len(arguments):
                    raise ValueError('--runtime requires main or game')
                runtime = arguments[index]
            else:
                runtime = item.partition('=')[2]
            if runtime not in ('main', 'game'):
                raise ValueError('--runtime requires main or game')
        else:
            remaining.append(item)
        index += 1
    return runtime or 'main', remaining


def launch(entry):
    if entry not in ENTRIES:
        raise ValueError('unknown runtime entry')
    try:
        runtime, arguments = split_runtime(sys.argv[1:])
    except ValueError as exc:
        print(f'{Path(sys.argv[0]).name}: error: {exc}', file=sys.stderr)
        return 2
    if runtime == 'game':
        target = ROOT / 'game' / (entry + '.py')
        if not target.is_file():
            print(f'game runtime unavailable: {target}', file=sys.stderr)
            return 2
        # Replace this process on Linux: signals go straight to the selected
        # controller. Its script directory wins over all main imports. Keep
        # the caller's cwd, environment, interpreter, and relative arguments.
        command = [sys.executable, '-u', '-B', str(target), *arguments]
        try:
            os.execv(sys.executable, command)
        except OSError as exc:
            print(f'cannot start game runtime: {exc}', file=sys.stderr)
            return 1
    sys.argv[1:] = arguments
    from utils.run_logs import run_entry
    return run_entry(entry, ENTRIES[entry], require_archive=False)
