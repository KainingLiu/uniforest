"""Reproducible dense endpoint-pair and perception-scenario verification."""
from dataclasses import asdict
from datetime import datetime, timezone
import itertools
import json
import random
from .navigation import HERE,load_planner,simulate,engine_revision
from Strategy.navigation import Location


def verify():
    revision=engine_revision(); print('ENGINE',revision,flush=True)
    p=load_planner(); nodes=[]
    for f in p.LABELS:
        for u in ((0,.5,1) if f in ('orange_ground','orange_highland','purple','building') else (.5,)):
            nodes.append(Location(f,u))
    cases=[(a,b,{}) for a,b in itertools.product(nodes,repeat=2)]
    rng=random.Random(20261002)
    for _ in range(48):
        a,b=(Location(rng.choice(list(p.LABELS)),rng.random(),rng.choice([0,20,60,120])) for _ in range(2))
        cases.append((a,b,{}))
    for vision in ('normal','dropout','outlier','missing','no_building'):
        cases.append((Location('orange_highland',.7),Location('building',.35),dict(vision=vision,drift_mm=30)))
    results=[]
    for index,(a,b,options) in enumerate(cases):
        try:
            r=simulate(a,b,planner=p,max_points=4,**options)
            expected='stopped' if options.get('vision') in ('missing','no_building') else 'arrived'
            fast=r['metrics']['high_speed_min_clearance_mm']
            ok=(r['status']==expected and r['metrics']['collisions']==0 and
                (expected=='stopped' or r['metrics']['position_error_mm']<(5 if b.family=='building' else p.motion.arrival_mm)) and
                (fast is None or fast>=100))
            results.append(dict(source=asdict(a),target=asdict(b),scenario=options,passed=ok,
                                status=r['status'],reason=r.get('reason'),metrics=r['metrics']))
        except Exception as exc:
            results.append(dict(source=asdict(a),target=asdict(b),scenario=options,passed=False,error=str(exc)))
        if not results[-1]['passed']: print('FAIL',json.dumps(results[-1]),flush=True)
        if (index+1)%30==0: print(f'{index+1}/{len(cases)} checked',flush=True)
    report=dict(generated_at=datetime.now(timezone.utc).isoformat(),simulation_only=True,
                field_validated=False,total=len(results),passed=sum(r['passed'] for r in results),
                node_count=len(nodes),clearance_threshold_mm=100,engine_revision=revision,results=results)
    (HERE/'output').mkdir(exist_ok=True)
    (HERE/'output/navigation-verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print('RESULT',report['passed'],'/',report['total'],flush=True)
    return report


if __name__=='__main__':
    result=verify()
    raise SystemExit(0 if result['passed']==result['total'] else 1)
