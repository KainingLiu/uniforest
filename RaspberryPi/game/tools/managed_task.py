#!/usr/bin/env python3
"""Run main.py under the Pi's user service manager, independent of SSH."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
UNIT = 'uniforest-task.service'


def start_command(arguments, root=ROOT):
    if not arguments:
        raise ValueError('pass an explicit main.py selection after --')
    if not any(a in ('--strategy','--flow','--program') or
               a.startswith(('--strategy=','--flow=','--program=')) for a in arguments):
        raise ValueError('an explicit --strategy, --flow or --program is required')
    uid = os.getuid()
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{uid}'))
    if not runtime.is_dir():
        raise RuntimeError('user runtime directory unavailable')
    return ['systemd-run','--user','--collect','--unit='+UNIT,
            '--property=Type=exec','--property=Restart=no',
            '--property=KillSignal=SIGINT','--property=KillMode=control-group',
            '--property=TimeoutStopSec=8s','--property=StandardOutput=journal',
            '--property=StandardError=journal','--working-directory='+str(root),
            '--','/usr/bin/flock','-n','-F',str(runtime/f'uniforest-desktop-{uid}.lock'),
            str(root/'.venv/bin/python'),'-u','-B',str(root/'main.py'),*arguments]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('start','stop','status','logs'))
    parser.add_argument('arguments',nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    arguments = args.arguments
    if arguments[:1] == ['--']:
        arguments = arguments[1:]
    if sys.platform != 'linux':
        parser.error('run this tool on the Raspberry Pi')
    try:
        if args.action == 'start':
            command = start_command(arguments)
            # Without lingering, the user manager may exit with the last SSH
            # login, defeating the purpose of this launcher.
            linger = subprocess.run(['loginctl','show-user',str(os.getuid()),
                '--property=Linger','--value'],capture_output=True,text=True,check=True)
            if linger.stdout.strip() != 'yes':
                raise RuntimeError('enable user lingering first: loginctl enable-linger '+str(os.getuid()))
        else:
            if arguments:
                raise ValueError('only start accepts main.py arguments')
            command = (['journalctl','_SYSTEMD_USER_UNIT='+UNIT,
                        '_UID='+str(os.getuid()),'-f','-n','60']
                       if args.action == 'logs' else
                       ['systemctl','--user',args.action,UNIT,'--no-pager'])
        return subprocess.call(command)
    except (OSError,ValueError,RuntimeError,subprocess.CalledProcessError) as exc:
        print(str(exc),file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
