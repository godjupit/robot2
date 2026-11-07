#!/usr/bin/env python3
"""LAN hand tracking receiver. Default mode never contacts the robot."""
import argparse
import hmac
import json
import math
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def validate(data):
    if type(data.get('enabled')) is not bool:
        raise ValueError('enabled must be boolean')
    if not data['enabled']:
        return None
    xyz = data.get('xyz')
    if not isinstance(xyz, list) or len(xyz) != 3:
        raise ValueError('xyz requires three meters values')
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in xyz):
        raise ValueError('invalid coordinates')
    if not .12 <= xyz[2] <= 1.0 or any(abs(x) > 1 for x in xyz[:2]):
        raise ValueError('hand outside capture range')
    return xyz


def mapped(origin, hand_origin, hand, scale):
    # Camera +X right, +Y down, +Z away; robot +X forward, +Y left, +Z up.
    d = [hand[i] - hand_origin[i] for i in range(3)]
    return [origin[0] + scale*d[2], origin[1] - scale*d[0], origin[2] - scale*d[1]]


class Robot:
    def __init__(self, url, robot_id):
        import requests
        import numpy as np
        from robot_model import Model, FeedbackClient
        self.np = np
        self.session = requests.Session()
        self.session.trust_env = False
        self.url, self.robot_id = url.rstrip('/'), robot_id
        status = self.call('/status', get=True).get('robot_status', [])
        if robot_id >= len(status) or not status[robot_id].get('device_name'):
            raise RuntimeError('SO100 not connected')
        config = self.call('/robot/config')
        if config.get('servo_ids') != [1, 2, 3, 4, 5, 6] or not config.get('config'):
            raise RuntimeError('Expected calibrated SO100 IDs 1..6')
        self.feedback = FeedbackClient(self, config['config'])
        self.model = Model()
        self.previous = None

    def call(self, endpoint, body=None, get=False):
        r = self.session.request('GET' if get else 'POST', self.url+endpoint,
                                params=None if get else {'robot_id': self.robot_id},
                                json=None if get else (body or {}), timeout=.4)
        r.raise_for_status()
        data = r.json()
        if data.get('status') == 'error':
            raise RuntimeError(str(data))
        return data

    def read(self):
        angles = self.np.deg2rad(self.feedback.read())
        self.model.check(angles[:5], angles[5])
        return angles

    def write(self, q):
        # Prefix of five joints: the gripper is never part of a write request.
        self.call('/joints/write', {'unit': 'rad', 'angles': q.tolist()})

    def hold(self):
        angles = self.np.deg2rad(self.feedback.read())
        self.write(angles[:5])
        self.previous = None

    def step(self, target, fresh):
        np = self.np
        angles = self.read()
        q = angles[:5]
        if self.previous is not None and np.max(np.abs(q-self.previous)) > math.radians(8):
            raise RuntimeError('Tracking error exceeds 8 degrees')
        current = self.model.position(q)
        delta = np.asarray(target)-current
        goal = current+delta*min(1., .002/max(np.linalg.norm(delta), 1e-9))
        solved = self.model.solve_position(goal, q)
        if np.linalg.norm(self.model.position(solved)-goal) > .005:
            raise RuntimeError('IK target unreachable')
        step = solved-q
        command = q+step*min(1., math.radians(.5)/max(np.max(np.abs(step)), 1e-9))
        # Check asynchronous joint arrival combinations, not just endpoints.
        import itertools
        for mask in itertools.product((0, 1), repeat=5):
            self.model.check(np.where(mask, command, q), angles[5])
        if fresh():
            self.write(command)
            self.previous = command


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.hand = None
        self.received = 0.
        self.generation = 0
        self.result = {'mode': 'preview', 'active': False}

    def put(self, hand):
        with self.lock:
            if hand is None:
                self.generation += 1
            self.hand, self.received = hand, time.monotonic()

    def snapshot(self):
        with self.lock:
            return self.hand, self.received, self.generation


def control(state, stop, args):
    robot = None
    origin = hand_origin = None
    last_generation = -1
    active = False
    fault = None
    try:
        if args.execute:
            robot = Robot(args.robot_url, args.robot_id)
        while not stop.wait(.1):
            hand, received, generation = state.snapshot()
            fresh = hand is not None and time.monotonic()-received < .3
            if not fresh or generation != last_generation:
                if active and robot:
                    robot.hold()
                origin = hand_origin = None
                active = False
            last_generation = generation
            if fresh:
                if origin is None:
                    origin = robot.model.position(robot.read()[:5]).tolist() if robot else [.25, 0., .20]
                    hand_origin = hand[:]
                target = mapped(origin, hand_origin, hand, args.scale)
                if math.dist(origin, target) > .08 or target[2] < .05:
                    raise RuntimeError('Target exceeds 8 cm relative workspace or 5 cm minimum height')
                active = True
                if robot:
                    def still_fresh():
                        h, t, g = state.snapshot()
                        return h is not None and g == generation and time.monotonic()-t < .3 and not stop.is_set()
                    robot.step(target, still_fresh)
                with state.lock:
                    state.result = {'mode': 'execute' if robot else 'preview', 'active': True, 'target_m': target}
            else:
                with state.lock:
                    state.result = {'mode': 'execute' if robot else 'preview', 'active': False}
    except Exception as exc:
        fault = str(exc)
        print('STOP:', fault, flush=True)
    finally:
        if robot and active:
            try:
                robot.hold()
            except Exception as exc:
                print('Hold request failed:', exc, flush=True)
        with state.lock:
            state.result = {'active': False, 'fault': fault or 'server stopped; restart required'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--token', default=None)
    parser.add_argument('--scale', type=float, default=.4)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--robot-url', default='http://127.0.0.1:80')
    parser.add_argument('--robot-id', type=int, default=0)
    args = parser.parse_args()
    if not math.isfinite(args.scale) or not 0 < args.scale <= 1 or args.robot_id < 0:
        parser.error('scale must be in (0,1], robot-id >= 0')
    token = args.token or secrets.token_urlsafe(18)
    state, stop = State(), threading.Event()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *unused):
            pass

        def do_POST(self):
            self.connection.settimeout(1)
            if self.path != '/hand':
                self.send_error(404)
                return
            if not hmac.compare_digest(self.headers.get('Authorization', ''), 'Bearer '+token):
                self.send_error(401)
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 2048:
                    raise ValueError('invalid payload length')
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError('object required')
                state.put(validate(data))
                with state.lock:
                    payload = json.dumps(state.result).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            except (ValueError, TypeError, OSError) as exc:
                state.put(None)
                self.send_error(400, str(exc))

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    worker = threading.Thread(target=control, args=(state, stop, args), daemon=True)
    worker.start()
    print(f'Mode: {"EXECUTE" if args.execute else "PREVIEW (no robot connection)"}\nPort: {args.port}\nToken: {token}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        worker.join(timeout=5)


if __name__ == '__main__':
    main()
