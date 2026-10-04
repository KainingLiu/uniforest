#!/usr/bin/env python3
"""Manage plan, set and collect-build desktop launchers; never connect to hardware."""
import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
# Keep existing plan file IDs so desktop shortcuts are updated in place.
ENTRIES = {
    'uniforest-all.desktop': ('PlanA', 'Uniforest PlanA 原策略'),
    'uniforest-planb.desktop': ('PlanB', 'Uniforest PlanB 采集投放策略'),
    'uniforest-pland.desktop': ('PlanD', 'Uniforest PlanD 按底座层数搭建'),
    'uniforest-set1.desktop': ('set1', 'Uniforest set1 任务组合'),
    'uniforest-set2.desktop': ('set2', 'Uniforest set2 任务组合'),
    'uniforest-collect-build-1.desktop': ('collect-build-1', 'Uniforest collect-build-1 采集搭建'),
    'uniforest-collect-build-2.desktop': ('collect-build-2', 'Uniforest collect-build-2 采集搭建'),
}
# Exact names previously managed by this installer; unrelated shortcuts stay.
LEGACY_ENTRIES = tuple(f'uniforest-{selection}.desktop' for selection in (
    'round1', 'round2', 'task0',
    'task0-2', 'task1-1', 'task1-2', 'task2-1', 'task2-2',
    'task3-1', 'task3-2', 'task4-1', 'task4-2'))


def desktop_text(selection, name):
    launcher = str(ROOT / 'tools' / 'desktop_task.sh')
    # Desktop Exec uses double-quoted arguments, not shell single quotes.
    launcher = launcher.replace('\\', '\\\\').replace('"', '\\"').replace('`', '\\`').replace('$', '\\$').replace('%', '%%')
    comment = f'运行 {selection}；会驱动机器人，Ctrl+C 停止'
    return ('[Desktop Entry]\nVersion=1.0\nType=Application\n'
            f'Name={name}\nComment={comment}\n'
            f'Exec=/bin/bash "{launcher}" {selection}\nPath={ROOT}\n'
            'Icon=media-playback-start\nTerminal=true\nStartupNotify=false\n'
            'Categories=Utility;\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--desktop-dir', type=Path)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    desktop = args.desktop_dir
    if desktop is None:
        desktop = Path(subprocess.check_output(['xdg-user-dir', 'DESKTOP'], text=True).strip())
    entries = {str(desktop / name): desktop_text(*entry) for name, entry in ENTRIES.items()}
    obsolete = [desktop / name for name in LEGACY_ENTRIES
                if (desktop / name).exists() or (desktop / name).is_symlink()]
    if args.dry_run:
        print(json.dumps({'entries': entries, 'remove': [str(p) for p in obsolete]},
                         ensure_ascii=False, indent=2))
        return
    desktop.mkdir(parents=True, exist_ok=True)
    for filename, content in entries.items():
        target = Path(filename)
        target.write_text(content, encoding='utf-8')
        target.chmod(0o755)
    for target in obsolete:
        target.unlink()
    print(f'Updated {len(entries)} desktop launchers; removed {len(obsolete)} old entries: {desktop}')


if __name__ == '__main__':
    main()
