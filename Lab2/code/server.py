"""Lab 2 Wi-Fi service: Flask HTTP/JSON control, telemetry, camera, and map.
Run from project root: .venv/bin/python -m Lab2.code.server
"""
import argparse
import json
import math
import signal
import threading
import time
from pathlib import Path
from flask import Flask,Response,jsonify,request,send_from_directory
from shared.robot import Robot
from Lab1b.code.vision import Vision
from Lab1b.code.navigation import Navigator

ROOT=Path(__file__).resolve().parents[2]


def create_app(robot,vision,nav):
    app=Flask(__name__,static_folder=str(ROOT/'Lab2/electron/vendor'),static_url_path='/vendor')
    command_lock=threading.RLock()
    sequences={}

    @app.before_request
    def check_origin():
        if request.method=='POST':
            origin=request.headers.get('Origin')
            if origin and origin!=request.host_url.rstrip('/'):
                return jsonify(error='Cross-origin control is disabled'),403
            if not request.is_json:
                return jsonify(error='JSON required'),415

    @app.get('/')
    def index(): return send_from_directory(ROOT/'Lab2/electron','index.html')

    @app.get('/index.js')
    def javascript(): return send_from_directory(ROOT/'Lab2/electron','index.js')

    @app.get('/style.css')
    def css(): return send_from_directory(ROOT/'Lab2/electron','style.css')

    @app.get('/api/status')
    def status(): return jsonify(robot=robot.telemetry(),vision=vision.status(),navigation={'state':nav.state,'goal':nav.goal},config=robot.cfg)

    @app.get('/api/map')
    def map_data(): return jsonify(nav.snapshot())

    @app.get('/camera.jpg')
    def camera():
        with vision.lock: data=vision.raw_jpeg if request.args.get('raw')=='1' else vision.jpeg
        if data is None: return Response('Camera starting',status=503)
        return Response(data,mimetype='image/jpeg',headers={'Cache-Control':'no-store'})

    @app.post('/api/command')
    def command():
        data=request.get_json()
        if not isinstance(data,dict): return jsonify(error='Expected JSON object'),400
        try:
            with command_lock:
                name=data.get('command')
                client=str(data.get('client','api'))[:80]
                sequence=data.get('sequence')
                if sequence is not None:
                    if type(sequence) is not int: raise ValueError('sequence must be an integer')
                    if sequence<=sequences.get(client,-1): return jsonify(error='Stale command discarded'),409
                    sequences[client]=sequence
                if name=='stop':
                    robot.stop('Emergency/operator stop')
                    robot.mode='idle'
                elif name=='drive':
                    if nav.thread and nav.thread.is_alive(): raise RuntimeError('Stop navigation before manual driving')
                    direction=data.get('direction',1)
                    if type(direction) is not int or direction not in (-1,1): raise ValueError('direction must be -1 or 1')
                    steer=float(data.get('steer',0)); duration=float(data.get('duration',.35))
                    # Robot.drive checks finite values and limits before hardware access.
                    robot.cancel.clear(); robot.mode='manual'
                    robot.drive(direction,steer,duration)
                elif name=='scan': nav.start(scan_only=True)
                elif name=='navigate':
                    goal=data.get('goal')
                    if not isinstance(goal,list): raise ValueError('goal must be [forward_cm, left_cm]')
                    nav.start([float(v) for v in goal])
                elif name=='reset': nav.reset()
                else: raise ValueError('Unknown command')
            return jsonify(ok=True,robot=robot.telemetry())
        except (ValueError,TypeError,RuntimeError) as exc:
            return jsonify(error=str(exc)),400

    return app


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--host',default='0.0.0.0'); parser.add_argument('--port',type=int,default=5000)
    parser.add_argument('--resume',help='Development only: restore a stopped pose checkpoint')
    args=parser.parse_args()
    robot=Robot(); vision=Vision(robot); nav=Navigator(robot,vision)
    if args.resume:
        saved=json.loads(Path(args.resume).read_text())
        with robot.lock: robot.pose=list(saved['pose'])
        nav.history=saved.get('history',[])
    app=create_app(robot,vision,nav)
    def shutdown(*unused):
        robot.stop('Server shutting down')
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,shutdown); signal.signal(signal.SIGINT,shutdown)
    try:
        from werkzeug.serving import run_simple
        run_simple(args.host,args.port,app,threaded=True,use_reloader=False)
    finally:
        robot.stop('Server shutting down')
        if nav.thread: nav.thread.join(timeout=7)
        vision.close(); robot.close()

if __name__=='__main__': main()
