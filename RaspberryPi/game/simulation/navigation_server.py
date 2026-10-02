"""Local-only interactive navigation lab; no hardware construction or commands."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from dataclasses import replace
from pathlib import Path
from urllib.parse import urlsplit
from .navigation import HERE, load_planner, simulate, engine_revision
from Strategy.navigation import Location


class Handler(SimpleHTTPRequestHandler):
    def log_message(self,*args): pass

    def reply(self,status,value):
        data=json.dumps(value,ensure_ascii=False,allow_nan=False).encode('utf-8')
        self.send_response(status); self.send_header('Content-Type','application/json; charset=utf-8')
        self.send_header('Content-Length',str(len(data))); self.end_headers(); self.wfile.write(data)

    def do_GET(self):
        path=urlsplit(self.path).path
        if path=='/api/navigation':
            p=self.server.planner
            return self.reply(200,dict(field=p.field,robot=p.robot,catalog=p.catalog(),simulation_only=True,
                                       engine_revision=self.server.revision,motion_profile=p.motion.describe()))
        if path in ('/','/simulator.html','/navigation.html'):
            body=(HERE/'navigation.html').read_bytes()
            self.send_response(200); self.send_header('Content-Type','text/html; charset=utf-8')
            self.send_header('Content-Length',str(len(body))); self.end_headers(); self.wfile.write(body)
            return
        if path=='/verification.json':
            file=HERE/'output/navigation-verification.json'
            return self.reply(200,json.loads(file.read_text(encoding='utf-8'))) if file.exists() else self.reply(404,{'error':'验证报告尚未生成'})
        if path=='/linear-comparison.json':
            file=HERE/'output/linear-comparison.json'
            return self.reply(200,json.loads(file.read_text(encoding='utf-8'))) if file.exists() else self.reply(404,{'error':'对照尚未生成'})
        if path=='/match.html': return super().do_GET()
        self.reply(404,{'error':'not found'})

    def do_POST(self):
        if urlsplit(self.path).path!='/api/plan': return self.reply(404,{'error':'not found'})
        try:
            # This endpoint performs CPU-only simulation. Limit payloads and
            # expose no path, command, hardware, or arbitrary Python interface.
            length=int(self.headers.get('Content-Length','0'))
            if not 0<length<=4096: raise ValueError('invalid request length')
            data=json.loads(self.rfile.read(length))
            if set(data)-{'source','target','vision','delay_s','drift_mm','cruise_multiplier'}: raise ValueError('unknown request fields')
            planner=self.server.planner
            multiplier=data.get('cruise_multiplier',planner.motion.cruise_multiplier)
            if isinstance(multiplier,bool) or not isinstance(multiplier,(int,float)) or not .5<=multiplier<=1.2:
                raise ValueError('cruise multiplier must be in [0.5,1.2]')
            if multiplier!=planner.motion.cruise_multiplier:
                from Strategy.navigation import FieldPlanner
                planner=FieldPlanner(planner.field,planner.robot,motion=replace(planner.motion,cruise_multiplier=multiplier,wheel_rpm=None),tracking=planner.tracking)
            result=simulate(Location(**data['source']),Location(**data['target']),planner=planner,
                            vision=data.get('vision','normal'),delay_s=data.get('delay_s',.12),
                            drift_mm=data.get('drift_mm',0))
            result['engine_revision']=self.server.revision
            self.reply(200,result)
        except (ValueError,TypeError,KeyError,RuntimeError) as exc:
            self.reply(400,{'error':str(exc)})


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8766)
    args=parser.parse_args()
    server=ThreadingHTTPServer(('127.0.0.1',args.port),partial(Handler,directory=str(HERE/'output')))
    server.planner=load_planner()
    from Strategy.navigation.search import PoseSearch
    server.planner._search=PoseSearch(server.planner)
    server.revision=engine_revision()
    print(f'Navigation simulation: http://127.0.0.1:{args.port}/simulator.html',flush=True)
    server.serve_forever()


if __name__=='__main__': main()
