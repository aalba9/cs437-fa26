"""Single hardware owner, bounded commands, sensor telemetry, calibrated odometry.
Coordinates: x forward, y left, heading counterclockwise (radians).
Speed/distance are estimates, never claimed to be encoder measurements.
"""
import atexit
import fcntl
import getpass
import json
import math
import os
import statistics
import threading
import time
from pathlib import Path
from shared.ranging import RangeReader
from shared.kinematics import steering_angle,travel_speed

ROOT = Path(__file__).resolve().parents[1]


def load_config():
    return json.loads((ROOT / 'shared/config.json').read_text(encoding='utf-8-sig'))


class Robot:
    def __init__(self, config=None):
        self.cfg = config or load_config()
        self._file = open('/tmp/cs437-car.lock', 'w')
        fcntl.flock(self._file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        os.getlogin = getpass.getuser  # SSH/services have no controlling terminal.
        from picarx import Picarx
        self.px = Picarx()
        self.ranger = RangeReader(self.px.ultrasonic)
        self.px.stop()
        self.px.dir_cali_val += self.cfg.get('steering_trim_deg',0.)
        self.px.set_dir_servo_angle(0)
        self.lock = threading.RLock()
        self.closed = threading.Event()
        self.cancel = threading.Event()
        self.pose = [0.0, 0.0, 0.0]
        self.total_cm = 0.0
        self.velocity = 0.0
        self.motion_started_at = 0.0
        self.steer = 0.0
        self.pan = 0.0
        self.pan_centered_at = time.monotonic()
        self.deadline = 0.0
        self.distance = None
        self.range_state = 'invalid'
        self.sensor_at = 0.0
        self.grayscale = []
        self.mode = 'idle'
        self.reason = 'Ready; motors stopped'
        self._last_tick = time.monotonic()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        atexit.register(self.close)

    def _integrate(self, now):
        dt = max(0, now-max(self._last_tick,self.motion_started_at))
        self._last_tick = now
        ds = self.velocity*dt
        turn = ds*math.tan(math.radians(steering_angle(self.steer,self.cfg)))/self.cfg['wheelbase_cm']
        heading = self.pose[2]
        if abs(turn) > 1e-8:
            radius = ds/turn
            self.pose[0] += radius*(math.sin(heading+turn)-math.sin(heading))
            self.pose[1] -= radius*(math.cos(heading+turn)-math.cos(heading))
        else:
            self.pose[0] += ds*math.cos(heading)
            self.pose[1] += ds*math.sin(heading)
        self.pose[2] = (heading+turn+math.pi)%(2*math.pi)-math.pi
        self.total_cm += abs(ds)

    def _stop(self, reason):
        self._integrate(time.monotonic())
        self.px.stop()
        self.velocity = 0.0
        self.deadline = 0.0
        self.reason = reason

    def stop(self, reason='Stopped', cancel=True):
        if cancel:
            self.cancel.set()
        with self.lock:
            self._stop(reason)

    def drive(self, direction, steer=0.0, duration=.35):
        if direction not in (-1, 1):
            raise ValueError('direction must be -1 or 1')
        if not all(math.isfinite(float(v)) for v in (steer, duration)):
            raise ValueError('non-finite command')
        if not -30 <= steer <= 30 or not 0 < duration <= 1.2:
            raise ValueError('command outside limits')
        with self.lock:
            if self.cancel.is_set() or self.closed.is_set():
                raise RuntimeError('Motion canceled; start a new command')
            if self.pan != 0:
                raise RuntimeError('Cannot drive during a scan')
            if direction > 0 and (not self._range_clear() or time.monotonic()-self.sensor_at > .7):
                self._stop('Front obstacle or unavailable range reading')
                raise RuntimeError(self.reason)
            self._integrate(time.monotonic())
            self.steer = float(steer)
            try:
                self.px.set_dir_servo_angle(steer*self.cfg['steering_sign'])
                time.sleep(.10)
                self._integrate(time.monotonic())  # Settling while stopped is not travel.
                if self.cancel.is_set(): raise RuntimeError('Motion canceled')
                (self.px.forward if direction > 0 else self.px.backward)(self.cfg['speed'])
            except Exception:
                self._stop('Motion command failed or canceled')
                raise
            if not self.velocity:
                self.motion_started_at = time.monotonic()+self.cfg.get('startup_delay_s',0.)
            self.velocity = direction*travel_speed(steer,self.cfg)
            self.deadline = time.monotonic()+duration
            self.reason = 'Driving with command timeout'

    def duration_for_distance(self,distance,steer=0.):
        return abs(distance)/travel_speed(steer,self.cfg)+self.cfg.get('startup_delay_s',0.)

    def _range_clear(self):
        return self.range_state == 'beyond_range' or (self.distance is not None and self.distance >= self.cfg['stop_distance_cm'])

    def set_pan(self, angle):
        with self.lock:
            if self.velocity:
                raise RuntimeError('Stop before panning')
            self.px.set_cam_pan_angle(angle*self.cfg['pan_sign'])
            self.pan = angle
            if angle == 0:
                self.pan_centered_at = time.monotonic()

    def scan(self):
        self.stop('Scanning', cancel=False)
        observations = []
        try:
            for angle in range(-75, 76, 10):
                if self.cancel.is_set():
                    break
                self.set_pan(angle)
                if self.cancel.wait(.14):
                    break
                values = []
                for _ in range(3):
                    with self.lock:
                        value,kind = self.ranger.read()
                    if kind == 'beyond_range': values.append(400.)
                    elif value is not None: values.append(float(value))
                observations.append({'angle': angle, 'distance': statistics.median(values) if values else None})
        finally:
            self.set_pan(0)
            time.sleep(.16)
            with self.lock:
                self.distance,self.range_state = self.ranger.read()
                self.sensor_at = time.monotonic()
        return observations

    def reset_pose(self):
        with self.lock:
            if self.velocity or self.mode in ('navigation', 'scan'):
                raise RuntimeError('Stop navigation before resetting the origin')
            self.pose = [0., 0., 0.]
            self.total_cm = 0.

    def telemetry(self):
        try:
            temp = float(Path('/sys/class/thermal/thermal_zone0/temp').read_text())/1000
        except (OSError, ValueError):
            temp = None
        with self.lock:
            return {'time': time.time(), 'mode': self.mode, 'reason': self.reason,
                    'distance_cm': self.distance, 'range_state': self.range_state, 'range_age_s': round(time.monotonic()-self.sensor_at, 3),
                    'temperature_c': temp, 'grayscale': self.grayscale,
                    'speed_cm_s_estimated': round(self.velocity, 2),
                    'total_distance_cm_estimated': round(self.total_cm, 2),
                    'pose_estimated': [round(v, 3) for v in self.pose],
                    'steering_deg': self.steer, 'pan_deg': self.pan,
                    'calibrated': self.cfg['calibrated']}

    def _loop(self):
        next_sensor = 0.0
        try:
            while not self.closed.wait(.015):
                with self.lock:
                    now = time.monotonic()
                    self._integrate(now)
                    if self.velocity and (now >= self.deadline or self.cancel.is_set()):
                        self._stop('Command expired' if not self.cancel.is_set() else 'Canceled')
                    if now >= next_sensor and self.pan == 0:
                        self.distance,self.range_state = self.ranger.read()
                        self.sensor_at = time.monotonic()
                        self.grayscale = self.px.get_grayscale_data()
                        next_sensor = time.monotonic()+.09
                        if self.velocity > 0 and not self._range_clear():
                            self._stop('Front obstacle or invalid ultrasonic reading')
        except Exception as exc:
            self.cancel.set()
            with self.lock:
                self._stop('Hardware error: '+str(exc))
            self.closed.set()

    def close(self):
        if self.closed.is_set():
            with self.lock:
                self.px.stop()
            return
        self.closed.set()
        self.cancel.set()
        self.thread.join(timeout=2)
        with self.lock:
            self._stop('Shutdown')
            self.px.set_dir_servo_angle(0)
            self.px.set_cam_pan_angle(0)
        self._file.close()
