"""Repeated ultrasonic mapping, packaged A*, bounded driving, and vision rules."""
import json
import math
import threading
import time
from pathlib import Path
from Lab1b.code.mapping import Grid,hybrid_astar,advance,GoalBlocked,PlanningTimeout,approach_blocked_goal
from shared.kinematics import steering_angle

ROOT=Path(__file__).resolve().parents[2]


class Navigator:
    def __init__(self,robot,vision):
        self.robot=robot; self.vision=vision
        self.grid=Grid(robot.cfg)
        self.route=[]; self.goal=None; self.state='idle'
        self.thread=None; self.lock=threading.Lock()
        self.stop_latched=False; self.last_sign=0.; self.started_at=0.
        self.events=[]; self.run_dir=None; self.history=[]; self.recoveries=0

    def event(self,event,**details):
        item={'time':time.time(),'event':event,**details}
        with self.lock: self.events.append(item)
        print(json.dumps(item),flush=True)
        if self.run_dir:
            with (self.run_dir/'events.jsonl').open('a') as f: f.write(json.dumps(item)+'\n')

    def snapshot(self):
        with self.lock:
            return {'state':self.state,'goal':self.goal,'route':self.route,'map':self.grid.export(),'events':self.events[-15:]}

    def start(self,goal=None,scan_only=False):
        if self.thread and self.thread.is_alive(): raise RuntimeError('Navigation or scan already running')
        if not scan_only and not self.robot.cfg['calibrated']: raise RuntimeError('Complete steering/pan calibration before autonomous driving')
        if goal is not None and (len(goal)!=2 or not all(math.isfinite(float(v)) for v in goal)):
            raise ValueError('Goal must contain two finite coordinates')
        self.robot.stop('Preparing scan')
        self.robot.cancel.clear()
        self.goal=goal; self.route=[]; self.stop_latched=False; self.last_sign=0.; self.recoveries=0
        self.started_at=time.monotonic()
        self.run_dir=ROOT/'Lab1b/evidence'/time.strftime('run-%Y%m%d-%H%M%S')
        self.run_dir.mkdir(parents=True,exist_ok=True)
        self.thread=threading.Thread(target=self._run,args=(scan_only,),daemon=True); self.thread.start()

    def reset(self):
        if self.thread and self.thread.is_alive(): raise RuntimeError('Wait for navigation to stop')
        self.robot.reset_pose()
        with self.lock:
            self.grid=Grid(self.robot.cfg); self.route=[]; self.goal=None; self.state='idle'; self.history=[]

    def _sign_observation(self,state,now):
        signs=[obj for obj in state['detections'] if obj['label']=='stop sign']
        observed_at=state['front_at'] if signs else 0.
        cached=state.get('stop_sign')
        if cached and cached['at']>=self.started_at and 0<=now-cached['at']<=15 and cached['at']>observed_at:
            signs=cached['detections'];observed_at=cached['at']
        return signs,observed_at

    def _vision_clear(self):
        """Wait for a fresh forward-facing frame, persons to leave, and sign hold."""
        deadline=time.monotonic()+90
        sign_until=0.
        clear_frame=None
        while not self.robot.cancel.is_set():
            now=time.monotonic()
            if now>deadline: raise RuntimeError('Vision wait timed out')
            state=self.vision.status()
            if state['error']: raise RuntimeError('Vision unavailable: '+state['error'])
            fresh=state['front_at']>self.robot.pan_centered_at+.2 and now-state['front_at']<1.2
            if not fresh:
                clear_frame=None
                self.robot.stop('Waiting for fresh forward camera frame',cancel=False)
                self.robot.cancel.wait(.05); continue
            labels={d['label'] for d in state['detections']}
            signs,observed_at=self._sign_observation(state,now)
            if signs:
                self.last_sign=max(self.last_sign,observed_at)
                if not self.stop_latched:
                    self.stop_latched=True; sign_until=now+self.robot.cfg['stop_seconds']
                    self.event('stop_sign',detections=signs,observed_at=observed_at)
            elif now-self.last_sign>15:
                self.stop_latched=False
            if 'person' in labels:
                clear_frame=None
                self.robot.stop('Waiting for person to leave',cancel=False)
            elif now<sign_until:
                clear_frame=None
                self.robot.stop('Stop sign: three-second hold',cancel=False)
            else:
                if clear_frame is None:clear_frame=state['front_at']
                elif state['front_at']-clear_frame>=1.:return True
            self.robot.cancel.wait(.05)
        return False

    def _backtrack(self):
        if not self.history or self.recoveries>=4:
            return False
        last=self.history[-1]
        if time.monotonic()-last['at']>90:
            return False
        pose=tuple(self.robot.pose)
        distance=min(last['distance'],10.)
        samples=[advance(pose,-distance*i/8,steering_angle(last['steer'],self.robot.cfg),self.robot.cfg['wheelbase_cm']) for i in range(1,9)]
        if distance<1 or not all(self.grid.free(p) for p in samples):
            return False
        # Reversal is limited to the immediately preceding traversed arc. This
        # is not permission to drive into unobserved space behind the car.
        predicted=samples[-1]
        if math.hypot(predicted[0]-last['start'][0],predicted[1]-last['start'][1])>4:
            return False
        self.history.pop();self.recoveries+=1;self.state='retracing last clear segment'
        self.event('backtrack',distance=distance,steer=last['steer'],pose=pose)
        self.robot.drive(-1,last['steer'],self.robot.duration_for_distance(distance,last['steer']))
        self.robot.cancel.wait(self.robot.duration_for_distance(distance,last['steer']))
        self.robot.stop('Recovery complete; rescanning',cancel=False)
        return True

    def _run(self,scan_only):
        self.robot.mode='scan' if scan_only else 'navigation'
        self.state='scanning'; begin=time.monotonic();blocked_goal_scans=0
        try:
            for cycle in range(90):
                if self.robot.cancel.is_set(): break
                if time.monotonic()-begin>600: raise RuntimeError('Ten-minute navigation limit')
                self.state='scanning'
                readings=self.robot.scan()
                if self.robot.cancel.is_set(): break
                with self.robot.lock: pose=tuple(self.robot.pose)
                with self.lock: self.grid.update(pose,readings)
                self.event('scan',pose=pose,readings=readings)
                (self.run_dir/f'map-{cycle:03d}.json').write_text(json.dumps(self.grid.export()))
                if scan_only:
                    self.state='scan complete'; return
                if math.hypot(pose[0]-self.goal[0],pose[1]-self.goal[1])<=self.robot.cfg['goal_tolerance_cm']:
                    self.state='goal reached (estimated)'; self.event('goal_reached',pose=pose,goal=self.goal); return
                self.state='planning'
                try:
                    route=hybrid_astar(self.grid,pose,self.goal)
                    blocked_goal_scans=0
                except GoalBlocked:
                    approach=approach_blocked_goal(self.grid,pose,self.goal)
                    if approach:
                        waypoint,route=approach
                        blocked_goal_scans=0
                        self.event('approach_waypoint',waypoint=waypoint,goal=self.goal)
                    else:
                        blocked_goal_scans+=1
                        if blocked_goal_scans>self.robot.cfg.get('obstacle_memory_scans',4):raise
                        self.event('goal_blocked_rescan',attempt=blocked_goal_scans)
                        self.robot.stop('Goal occupied in map; stationary rescan',cancel=False)
                        continue
                except PlanningTimeout:
                    raise
                except RuntimeError:
                    if self._backtrack(): continue
                    raise
                with self.lock: self.route=route
                self.event('plan',goal=self.goal,route=route)
                if not route: continue
                self.state='checking vision'
                if not self._vision_clear(): break
                action=route[0]; duration=self.robot.duration_for_distance(min(action['distance'],self.robot.cfg['segment_cm']),action['steer'])
                self.state='driving'
                previous_total=self.robot.total_cm
                self.robot.drive(1,action['steer'],duration)
                end=time.monotonic()+duration
                while time.monotonic()<end and not self.robot.cancel.is_set():
                    state=self.vision.status()
                    if state['error'] or time.monotonic()-state['front_at']>1.2:
                        self.event('motion_interrupted',reason='Vision stale')
                        self.robot.stop('Vision stale',cancel=False); break
                    labels={o['label'] for o in state['detections']}
                    signs,_=self._sign_observation(state,time.monotonic())
                    if 'person' in labels or (signs and not self.stop_latched):
                        self.event('motion_interrupted',reason='Camera detected traffic hazard',detections=state['detections'])
                        self.robot.stop('Camera detected traffic hazard',cancel=False); break
                    self.robot.cancel.wait(.02)
                self.robot.stop('Segment complete; rescanning',cancel=False)
                traveled=self.robot.total_cm-previous_total
                if traveled>.5:
                    self.history.append({'start':pose,'steer':action['steer'],'distance':traveled,'at':time.monotonic()})
            self.state='canceled' if self.robot.cancel.is_set() else 'segment limit reached'
        except Exception as exc:
            self.state='stopped: '+str(exc); self.event('error',message=str(exc))
        finally:
            self.robot.stop(self.state,cancel=False); self.robot.mode='idle'
            (self.run_dir/'result.json').write_text(json.dumps({'state':self.state,'telemetry':self.robot.telemetry()},indent=2))
