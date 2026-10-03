"""Render the actual transfer recipe/compiler through the no-hardware replay."""
import argparse
import contextlib
import io
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tests.test_refill_planning import TransferReplay


def point(p):
    return [round(p.x_mm, 1), round(p.y_mm, 1), round(p.yaw_deg, 1)]


def world(start, local):
    a = math.radians(start.yaw_deg)
    return [round(start.x_mm+math.cos(a)*local.x_mm-math.sin(a)*local.y_mm, 1),
            round(start.y_mm+math.sin(a)*local.x_mm+math.cos(a)*local.y_mm, 1),
            round(start.yaw_deg+local.yaw_deg, 1)]


def case(source, index, offset):
    with contextlib.redirect_stdout(io.StringIO()):
        old = TransferReplay(offset=offset, planned=False).run(source, index)
        new = TransferReplay(offset=offset).run(source, index)
    segments = []
    for e in new.events:
        if e['kind'] == 'trajectory':
            ramp = (not e['settings'].monotone_xy and len(e['points']) == 2
                    and abs(e['points'][-1].y_mm) < 1e-6
                    and e['settings'].max_speed_mm_s <= 1000.)
            segments.append({'kind': 'ramp' if ramp else 'curve',
                             'points': [world(e['start'], p) for p in e['samples']]})
        elif e['kind'] in ('move', 'turn'):
            a, b = point(e['start']), point(e['pose'])
            segments.append({'kind': e['kind'], 'points': [
                [round(x+(y-x)*u/20,1) for x,y in zip(a,b)] for u in range(21)]})
    return dict(source=source, round=index, offset=offset,
                classic=[[0,0,0]]+[point(e['pose']) for e in old.events if e['kind'] in ('move','turn')],
                segments=segments,
                anchors=[dict(kind=e['kind'], pose=point(e['pose']))
                         for e in new.events if e['kind'] in ('wall','tag')],
                old_tag=next((point(e['pose']) for e in old.events if e['kind']=='tag'), None),
                original_turns=sum(e['kind']=='turn' for e in old.events),
                planned_turns=sum(e['kind']=='turn' for e in new.events),
                end=point(new.pose))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    cases = [case(source,index,offset) for source in ('ground','highland')
             for index in (1,2,3) for offset in (0,450,900,1350,1800)]
    template = Path(__file__).with_name('refill_route_view.html').read_text(encoding='utf-8')
    output = template.replace('__REFILL_DATA__', json.dumps(cases, separators=(',', ':')))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output, encoding='utf-8')
    print(f'{len(cases)} route views; {len(output.encode("utf-8"))} bytes; {args.output}')


if __name__ == '__main__': main()
