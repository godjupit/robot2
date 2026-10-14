#!/usr/bin/env python3
"""通过 Phosphobot HTTP 控制 SO100 六关节，无需断开服务或直接打开串口。"""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET
from zoneinfo import ZoneInfo
import requests

NAMES = ['底座', '肩', '肘', '腕俯仰', '腕旋转', '夹爪']
DEFAULT_URDF = Path(__file__).resolve().parent/'models/so-100.urdf'


def forward_xyz(angles_degrees, urdf=DEFAULT_URDF):
    """实际关节角度 -> 模型末端 link frame，底座模型参考系，单位 cm。"""
    import numpy as np
    if len(angles_degrees) != 6 or any(not math.isfinite(v) for v in angles_degrees):
        raise ValueError('需要六个有限关节角度')
    joints = ET.parse(urdf).getroot().findall('joint')[:5]
    expected = ['Rotation', 'Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll']
    if [j.get('name') for j in joints] != expected:
        raise ValueError('URDF 必须是当前 Phospho SO100 五轴运动链模型')

    def axis_rotation(axis, angle):
        axis = np.asarray(axis, dtype=float)
        axis = axis / np.linalg.norm(axis)
        x, y, z = axis
        skew = np.array([[0,-z,y], [z,0,-x], [-y,x,0]])
        return np.eye(3)+math.sin(angle)*skew+(1-math.cos(angle))*(skew@skew)

    transform = np.eye(4)
    # 与 Phosphobot SO100 的 AXIS_ORIENTATION=[0,0,1,1] 相同。
    transform[:3,:3] = axis_rotation([0,0,1], math.pi/2)
    for joint, angle in zip(joints, angles_degrees):
        origin = joint.find('origin')
        r, p, y = map(float, origin.get('rpy', '0 0 0').split())
        local = np.eye(4)
        local[:3,:3] = (axis_rotation([0,0,1],y) @ axis_rotation([0,1,0],p)
                         @ axis_rotation([1,0,0],r))
        local[:3,3] = list(map(float, origin.get('xyz', '0 0 0').split()))
        transform = transform @ local
        motion = np.eye(4)
        motion[:3,:3] = axis_rotation(list(map(float, joint.find('axis').get('xyz').split())), math.radians(angle))
        transform = transform @ motion
    return (transform[:3,3]*100).tolist()


def print_angles(start, goal, actual=None, title='六轴角度规划'):
    print(title, flush=True)
    print('ID   关节       起始角度    规划目标    最后实测    目标误差', flush=True)
    for i, (a, b) in enumerate(zip(start, goal)):
        measured = '--' if actual is None else f'{actual[i]:.2f}°'
        error = '--' if actual is None else f'{abs(actual[i]-b):.2f}°'
        print(f'{i+1:<4} {NAMES[i]:<6} {a:>9.2f}° {b:>9.2f}° {measured:>10} {error:>10}', flush=True)


def finite(text):
    value = float(text)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError('参数必须是有限数字')
    return value


def expanded(values, count):
    if len(values) == 1:
        return values * count
    if len(values) != count:
        raise ValueError('角度数量必须为 1 或与 --ids 数量一致')
    return values


def trajectory(start, goal, speed, acc, dt):
    """所有关节共用梯形/三角速度曲线；speed: °/s，acc: °/s²。"""
    distance = max(abs(b-a) for a, b in zip(start, goal))
    if distance < 1e-9:
        return []
    ramp = min(speed / acc, math.sqrt(distance / acc))
    peak = acc * ramp
    cruise = max(0., (distance-acc*ramp*ramp) / peak)
    total = 2*ramp + cruise
    result = []
    for i in range(1, math.ceil(total/dt)+1):
        t = min(i*dt, total)
        if t <= ramp:
            travel = .5*acc*t*t
        elif t <= ramp+cruise:
            travel = .5*acc*ramp*ramp + peak*(t-ramp)
        else:
            travel = distance - .5*acc*(total-t)**2
        fraction = min(1., travel/distance)
        result.append([a+(b-a)*fraction for a, b in zip(start, goal)])
    result[-1] = list(goal)
    return result


class Client:
    def __init__(self, url, robot_id):
        self.url, self.robot_id = url.rstrip('/'), robot_id
        self.session = requests.Session()
        self.session.trust_env = False

    def call(self, endpoint, body=None, get=False, timeout=5):
        response = self.session.request('GET' if get else 'POST', self.url+endpoint,
            params=None if get else {'robot_id': self.robot_id},
            json=None if get else ({} if body is None else body), timeout=timeout)
        response.raise_for_status()
        data = response.json()
        if data.get('status') == 'error':
            raise RuntimeError(str(data))
        return data

    def precheck(self):
        status = self.call('/status', get=True)
        robots = status.get('robot_status', [])
        if self.robot_id >= len(robots) or not robots[self.robot_id].get('device_name'):
            raise RuntimeError('未发现已连接的真机；请先在 Phosphobot 连接机器人')
        data = self.call('/robot/config')
        if data.get('servo_ids') != [1, 2, 3, 4, 5, 6] or not data.get('config'):
            raise RuntimeError('需要已标定、ID 为 1..6 的 SO100')
        return data['config']

    def read(self):
        data = self.call('/joints/read', {'unit': 'rad', 'source': 'robot'})
        values = data.get('angles', [])
        if data.get('unit') != 'rad' or len(values) != 6 or any(v is None or not math.isfinite(v) for v in values):
            raise RuntimeError('真机关节反馈无效')
        return [math.degrees(v) for v in values]

    def write(self, angles):
        # 当前服务的部分 joints_ids 写入存在兼容问题，统一传完整六轴。
        # 非选中轴的目标保持本次运动开始前的真实读数，不使用零填充。
        self.call('/joints/write', {'unit': 'rad', 'angles': [math.radians(v) for v in angles]})

    def torque(self, on):
        self.call('/torque/toggle', {'torque_status': on})

    def initialize(self, timeout):
        return self.call('/move/init', {}, timeout=timeout)

    def write_base(self, angle):
        # 原生接口在 angles 长度为 1 且省略 joints_ids 时，仅写第一个舵机。
        self.call('/joints/write', {'unit': 'rad', 'angles': [math.radians(angle)]})

    def close(self):
        self.session.close()


def base_step_to(client, target=45., timeout=120., interval=.5):
    """仅底座：发送目标每次调整 1°，按真实反馈判断到位，补偿最多 5°。"""
    initial = client.read()
    command = initial[0]
    started = time.monotonic()
    stable = stalled = 0
    previous = initial[0]
    rows = []
    log_file = Path(__file__).resolve().parent/'垃圾文件夹/logs'/f'base-step-{datetime.now(ZoneInfo("Asia/Shanghai")):%Y%m%d-%H%M%S-%f}.json'
    log_file.parent.mkdir(parents=True, exist_ok=True)
    result = {'target_deg': target, 'step_deg': 1, 'initial_angles': initial,
              'samples': rows, 'status': 'running'}
    log_file.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(f'实验日志：{log_file}', flush=True)
    try:
        for index in range(240):
            if time.monotonic()-started >= timeout:
                raise RuntimeError('底座实验总超时')
            actual = client.read()
            error = target-actual[0]
            rows.append({'time_s': time.monotonic()-started, 'command_deg': command,
                         'actual_angles': actual.copy(), 'error_deg': error})
            print(f'{index:03d}: 最近发送 {command:.2f}°，实测 {actual[0]:.2f}°，距 {target:g}° 差 {error:+.2f}°', flush=True)
            if max(abs(a-b) for a,b in zip(actual[1:],initial[1:])) > 3:
                raise RuntimeError('其他关节偏移超过 3°，停止底座实验')
            stable = stable+1 if abs(error) <= .5 else 0
            if stable >= 3:
                result['status'] = 'completed'
                result['final_command_deg'] = command
                result['final_actual_deg'] = actual[0]
                print(f'底座实测连续三次接近 {target:g}°（误差≤0.5°）；保留最近目标 {command:.2f}°。', flush=True)
                return result
            if abs(error) > .5:
                stalled = stalled+1 if index and abs(actual[0]-previous) < .1 else 0
                if stalled >= 8:
                    raise RuntimeError('底座连续 8 次反馈几乎未变化，停止累加目标')
                proposed = command + (1 if error > 0 else -1)
                if not min(initial[0],target)-5 <= proposed <= max(initial[0],target)+5:
                    raise RuntimeError('目标补偿超过 5°，停止累加')
                if abs(proposed-actual[0]) > 5:
                    raise RuntimeError('发送角度与真实角度相差超过 5°，停止累加')
                client.write_base(proposed)
                command = proposed
            previous = actual[0]
            time.sleep(interval)
        raise RuntimeError('底座实验达到最大循环次数')
    except BaseException as exc:
        result['status'] = 'stopped'
        result['reason'] = str(exc) or '用户中断'
        try:
            hold = client.read()[0]
            client.write_base(hold)
            result['hold_requested_deg'] = hold
        except Exception as hold_error:
            result['hold_error'] = str(hold_error)
        raise
    finally:
        log_file.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')


def initialize_with_feedback(client, config, initial, timeout=60., request_timeout=30.,
                             tolerance=.5, max_compensation=15., interval=.5):
    """原生初始化后，以真实反馈为准补偿六轴零位；每次最多修正 1°。"""
    offsets, signs = config['servos_offsets'], config['servos_offsets_signs']
    if (len(offsets) != 6 or len(signs) != 6
            or any(not math.isfinite(v) or not 0 <= v <= 4095 for v in offsets)
            or any(v not in (-1, 1) for v in signs)):
        raise ValueError('标定参数无效，未开始初始化')
    started = time.monotonic()
    deadline = started+timeout
    command = [0.]*6
    log_file = Path(__file__).resolve().parent/'垃圾文件夹/logs'/f'init-{datetime.now(ZoneInfo("Asia/Shanghai")):%Y%m%d-%H%M%S-%f}.json'
    result = {'status': 'planned', 'unit': 'degrees', 'initial_angles': initial.copy(),
              'desired_angles': [0.]*6, 'tolerance_deg': tolerance,
              'max_compensation_deg': max_compensation, 'timeout_s': timeout,
              'samples': [], 'last_command_angles': None}
    log_file.parent.mkdir(parents=True, exist_ok=True)
    log_file.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(f'初始化补偿日志：{log_file}', flush=True)
    attempted = False
    try:
        attempted = True
        result['status'] = 'running'
        client.initialize(min(request_timeout, timeout))
        result['last_command_angles'] = command.copy()
        print('原生初始化已返回；开始读取真实反馈，最多每次修正 1°。', flush=True)
        stable = 0
        previous = None
        best = [float('inf')]*6
        stalled = [0]*6
        for iteration in range(512):
            if time.monotonic() >= deadline:
                raise RuntimeError('初始化补偿超时，未达到真实零位')
            actual = client.read()
            result['samples'].append({'time_s': time.monotonic()-started,
                                      'actual_angles': actual.copy(),
                                      'command_angles': command.copy()})
            result['last_actual_angles'] = actual.copy()
            errors = [abs(v) for v in actual]
            print(f'补偿 {iteration:03d}：实测 {[round(v,2) for v in actual]}°；'
                  f'最大零位误差 {max(errors):.2f}°', flush=True)
            stable = stable+1 if max(errors) <= tolerance else 0
            if stable >= 3:
                result['status'] = 'completed'
                print_angles(initial, [0.]*6, actual, '真实零位连续三次到位；保留补偿目标')
                print(f'最终发送目标（度）：{[round(v,3) for v in command]}', flush=True)
                return result
            proposed = command.copy()
            for i, error in enumerate(errors):
                best[i] = min(best[i], error)
                if error <= tolerance:
                    stalled[i] = 0
                    continue
                if error > best[i]+3:
                    raise RuntimeError(f'ID{i+1} 误差比此前最好值增大超过 3°，停止补偿')
                stalled[i] = (stalled[i]+1 if previous is not None
                              and abs(actual[i]-previous[i]) < .1 else 0)
                if stalled[i] >= 8:
                    raise RuntimeError(f'ID{i+1} 连续 8 次反馈几乎不变，停止补偿')
                proposed[i] += math.copysign(min(1., error), -actual[i])
                if abs(proposed[i]) > max_compensation+1e-9:
                    raise RuntimeError(f'ID{i+1} 需要的补偿超过 {max_compensation:g}°，停止补偿')
                raw = proposed[i]*signs[i]*4095/360+offsets[i]
                if not -180 <= proposed[i] <= 180 or not 0 <= raw <= 4095:
                    raise RuntimeError(f'ID{i+1} 补偿目标超出角度/编码器范围')
            # 先验证所有轴，再一次写入；不改标定或 PID。
            if proposed != command:
                client.write(proposed)
                command = proposed
                result['last_command_angles'] = command.copy()
            previous = actual.copy()
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise RuntimeError('初始化补偿超时，未达到真实零位')
            time.sleep(min(interval, remaining))
        raise RuntimeError('初始化补偿达到最大循环次数')
    except BaseException as exc:
        result['status'] = 'stopped'
        result['reason'] = str(exc) or '用户中断'
        if result.get('last_actual_angles') is not None:
            print_angles(initial, [0.]*6, result['last_actual_angles'], '补偿停止前最后反馈')
        if attempted:
            try:
                hold = client.read()
                client.write(hold)
                result['hold_requested_angles'] = hold.copy()
                print('已尝试发送当前反馈位置作为保持目标；未关闭力矩。', file=sys.stderr)
            except Exception as hold_error:
                result['hold_error'] = str(hold_error)
                print(f'保持请求失败：{hold_error}', file=sys.stderr)
        raise
    finally:
        result['elapsed_s'] = time.monotonic()-started
        try:
            log_file.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
        except OSError as exc:
            print(f'初始化日志保存失败：{exc}', file=sys.stderr)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:80')
    parser.add_argument('--robot-id', type=int, default=0)
    sub = parser.add_subparsers(dest='command', required=True)
    read = sub.add_parser('read', help='读取真机关节角度')
    read.add_argument('--ids', type=int, nargs='+', default=list(range(1, 7)))
    xyz = sub.add_parser('xyz', help='读取真机关节并计算当前模型末端 XYZ（cm）')
    xyz.add_argument('--urdf', type=Path, default=DEFAULT_URDF)
    init = sub.add_parser('init', help='原生初始化后，按真实反馈补偿六轴零位')
    init.add_argument('--execute', action='store_true')
    init.add_argument('--timeout', type=finite, default=60, help='初始化和补偿的总时间上限，秒')
    init.add_argument('--request-timeout', type=finite, default=30, help='原生初始化 HTTP 请求超时秒数')
    init.add_argument('--tolerance', type=finite, default=.5, help='真实零位误差，度，连续三次满足')
    init.add_argument('--max-compensation', type=finite, default=15, help='相对零位的最大补偿目标，度')
    init.add_argument('--native', action='store_true', help='仅调用原生 /move/init，不做反馈补偿')
    step = sub.add_parser('base-step', help='实验：底座目标每次调整 1°，实际角度到 45° 停止')
    step.add_argument('--execute', action='store_true')
    direct = sub.add_parser('direct', help='一次调用原版 /joints/write，不做 IK 或速度插值')
    direct.add_argument('--ids', type=int, nargs='+', required=True)
    direct_target = direct.add_mutually_exclusive_group(required=True)
    direct_target.add_argument('--angles', type=finite, nargs='+', help='绝对关节角度，度')
    direct_target.add_argument('--delta', type=finite, nargs='+', help='相对当前角度增量，度')
    direct.add_argument('--wait', type=finite, default=1, help='发送后等待几秒再读取反馈，默认 1')
    direct.add_argument('--execute', action='store_true')
    move = sub.add_parser('move', help='按关节角度移动；默认只预览')
    move.add_argument('--ids', type=int, nargs='+', required=True)
    target = move.add_mutually_exclusive_group(required=True)
    target.add_argument('--angles', type=finite, nargs='+')
    target.add_argument('--delta', type=finite, nargs='+')
    move.add_argument('--speed', type=finite, default=5, help='软件规划速度上限，°/秒')
    move.add_argument('--acc', type=finite, default=10, help='软件规划加速度，°/秒²')
    move.add_argument('--hz', type=finite, default=10, help='指令频率上限，默认 10 Hz')
    move.add_argument('--timeout', type=finite, default=30, help='执行总超时秒数')
    move.add_argument('--tolerance', type=finite, default=1, help='到位误差，单位度')
    move.add_argument('--tracking-error', type=finite, default=5, help='跟踪误差停止阈值，单位度')
    move.add_argument('--execute', action='store_true')
    move.add_argument('--log-file', type=Path, help='执行记录 JSON 路径，默认 robot/垃圾文件夹/logs/move-时间.json')
    torque = sub.add_parser('torque', help='整台机械臂六轴力矩开关')
    torque.add_argument('state', choices=['on', 'off'])
    torque.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    client = None
    moving = False
    report = None
    log_file = None
    started = None
    try:
        if args.robot_id < 0:
            raise ValueError('--robot-id 必须非负')
        if args.command == 'init':
            if args.timeout <= 0 or args.request_timeout <= 0:
                raise ValueError('timeout 和 request-timeout 必须大于 0')
            if not .1 <= args.tolerance <= 2 or not 0 <= args.max_compensation <= 20:
                raise ValueError('tolerance 范围 0.1..2°；max-compensation 范围 0..20°')
        if args.command in ('read', 'move', 'direct'):
            if len(set(args.ids)) != len(args.ids) or any(i not in range(1, 7) for i in args.ids):
                raise ValueError('--ids 必须是 1..6 内不重复的 ID')
        if args.command == 'move':
            if not .1 <= args.speed <= 90 or not .1 <= args.acc <= 150 or not 1 <= args.hz <= 30:
                raise ValueError('speed 范围 0.1..90；acc 范围 0.1..150；hz 范围 1..30')
            if args.timeout <= 0 or not .1 <= args.tolerance <= 5 or not .1 <= args.tracking_error <= 15:
                raise ValueError('timeout 必须大于 0；tolerance 范围 0.1..5；tracking-error 范围 0.1..15')
        if args.command == 'direct' and not 0 <= args.wait <= 30:
            raise ValueError('--wait 范围 0..30 秒')
        if args.command in ('move', 'direct'):
            values = expanded(args.angles if args.angles is not None else args.delta, len(args.ids))
        client = Client(args.url, args.robot_id)
        config = client.precheck()
        current = client.read()
        if args.command == 'base-step':
            print(f'底座当前 {current[0]:.2f}°；实际目标 45°；每次发送目标调整 1°；仅写 ID1。')
            if not args.execute:
                print('仅预览；加 --execute 才执行。')
                return 0
            time.sleep(.3)
            check = client.read()
            if max(abs(a-b) for a,b in zip(check,current)) > 1:
                raise RuntimeError('机械臂当前尚未稳定，未开始底座实验')
            base_step_to(client)
            return 0
        if args.command == 'init':
            print_angles(current, [0.]*6, title='Phosphobot 初始化：六轴标定零位，包含夹爪')
            if not args.native:
                print(f'原生初始化后按反馈补偿：每次最多 1°，补偿上限 ±{args.max_compensation:g}°，'
                      f'六轴误差≤{args.tolerance:g}° 连续三次才报告到位。')
            if not args.execute:
                print('仅预览；未发送初始化、运动或力矩指令。加 --execute 才执行。')
                return 0
            if not args.native:
                initialize_with_feedback(client, config, current, args.timeout,
                                         args.request_timeout, args.tolerance, args.max_compensation)
                return 0
            # 完全交给原生接口；不附加关节插值或键盘页面的 open=1 指令。
            try:
                result = client.initialize(min(args.request_timeout, args.timeout))
            except requests.RequestException:
                print('初始化请求失败或超时；服务端可能已开始运动，HTTP 超时不会取消运动。', file=sys.stderr)
                raise
            print(f'原生 /move/init 已返回：{result}；此响应不代表真机已到位。')
            actual = client.read()
            print_angles(current, [0.]*6, actual, '初始化接口返回后的真实角度反馈')
            return 0
        if args.command == 'xyz':
            x, y, z = forward_xyz(current, args.urdf)
            print('当前模型末端坐标（真实关节反馈 → 正向运动学）')
            print(f'X = {x:.3f} cm\nY = {y:.3f} cm\nZ = {z:.3f} cm')
            print('参考系：模型底座原点，轴向与 Phosphobot SO100 模型一致。')
            print('测量点：URDF 末端 link frame；尚未计入笔长和笔尖偏移。')
            print('此坐标不是 /move/absolute 使用的初始化位置相对偏移。')
            return 0
        if args.command == 'read':
            for sid in args.ids:
                print(f'ID{sid} {NAMES[sid-1]}: {current[sid-1]:.2f}°')
            return 0
        if args.command == 'torque':
            print(f'整台机器人六轴力矩 {args.state}，包含夹爪。关闭前请托住机械臂。')
            if args.execute:
                if args.state == 'on':
                    client.write(current)
                client.torque(args.state == 'on')
            else:
                print('仅预览；加 --execute 才写入。')
            return 0
        goal = current.copy()
        for sid, value in zip(args.ids, values):
            goal[sid-1] = value if args.angles is not None else current[sid-1]+value
        for i, angle in enumerate(goal):
            raw = angle*config['servos_offsets_signs'][i]*4095/360+config['servos_offsets'][i]
            if not math.isfinite(angle) or not -180 <= angle <= 180 or not 0 <= raw <= 4095:
                raise ValueError(f'ID{i+1} 目标超出角度/编码器范围')
        print_angles(current, goal)
        if args.command == 'direct':
            print('原版 /joints/write：一次发送完整六轴目标，不做 IK 或软件速度规划。')
            if not args.execute:
                print('仅预览；未发送运动或力矩指令。加 --execute 才执行。')
                return 0
            try:
                client.write(goal)
            except requests.RequestException:
                print('写入请求失败；服务端可能已收到目标，不能据此判断运动已取消。', file=sys.stderr)
                raise
            print('写入接口已返回；此响应不代表真机已到位。', flush=True)
            time.sleep(args.wait)
            actual = client.read()
            print_angles(current, goal, actual, f'写入后等待 {args.wait:g} 秒的实际角度反馈')
            return 0
        dt = 1/args.hz
        frames = trajectory(current, goal, args.speed, args.acc, dt)
        if len(frames)*dt >= args.timeout:
            raise ValueError('规划时长超过 timeout，请增大 --timeout')
        print(f'软件规划：速度≤{args.speed}°/s，加速度≤{args.acc}°/s²，{len(frames)} 个目标')
        if not args.execute:
            print('仅预览；未发送运动或力矩指令。加 --execute 才执行。')
            return 0
        recorded_at = datetime.now(ZoneInfo('Asia/Shanghai'))
        log_file = args.log_file or Path(__file__).resolve().parent/'垃圾文件夹'/'logs'/f'move-{recorded_at:%Y%m%d-%H%M%S-%f}.json'
        log_file.parent.mkdir(parents=True, exist_ok=True)
        report = {'recorded_at': recorded_at.isoformat(), 'unit': 'degrees',
                  'robot_id': args.robot_id, 'selected_ids': args.ids,
                  'start_angles': current.copy(), 'planned_goal_angles': goal.copy(),
                  'planned_frames': frames, 'speed_deg_s': args.speed, 'acc_deg_s2': args.acc,
                  'timeout_s': args.timeout, 'tolerance_deg': args.tolerance,
                  'status': 'planned', 'motion_requested': False,
                  'feedback_samples': [], 'last_feedback_before_stop': None,
                  'last_command_angles': None, 'hold_feedback_angles': None}
        # 先保存规划；日志路径无法写入时不开始运动。
        log_file.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(f'执行日志：{log_file.resolve()}', flush=True)
        started = time.monotonic()
        deadline = started+args.timeout
        def record_feedback(angles, phase):
            sample = {'time_s': time.monotonic()-started, 'phase': phase,
                      'angles': angles.copy(),
                      'goal_error_deg': [abs(a-b) for a, b in zip(angles, goal)]}
            report['feedback_samples'].append(sample)
            report['last_feedback_before_stop'] = sample
        record_feedback(current, 'start')
        moving = True
        report['status'] = 'running'
        last = current
        for frame in frames:
            begin = time.monotonic()
            if begin >= deadline:
                raise RuntimeError('执行总超时')
            actual = client.read()
            record_feedback(actual, 'trajectory')
            tracking_errors = [abs(a-b) for a, b in zip(actual, last)]
            if max(tracking_errors) > args.tracking_error:
                worst = max(range(6), key=tracking_errors.__getitem__)
                raise RuntimeError(
                    f'真实关节跟踪误差超限，停止后续轨迹：ID{worst+1} {NAMES[worst]}，'
                    f'上一小步目标 {last[worst]:.2f}°，反馈 {actual[worst]:.2f}°，'
                    f'跟踪误差 {tracking_errors[worst]:.2f}° > 阈值 {args.tracking_error:g}°'
                    '（该误差与最终目标误差不同）')
            report['motion_requested'] = True
            client.write(frame)
            report['last_command_angles'] = frame.copy()
            last = frame
            # 不追赶 HTTP 延迟；请求较慢时降低实际指令频率。
            time.sleep(max(0, dt-(time.monotonic()-begin)))
        stable = 0
        while time.monotonic() < deadline:
            actual = client.read()
            record_feedback(actual, 'settling')
            error = max(abs(a-b) for a, b in zip(actual, goal))
            stable = stable+1 if error <= args.tolerance else 0
            if stable >= 3:
                moving = False
                report['status'] = 'completed'
                print_angles(current, goal, actual, '到位反馈（度）')
                print(f'真实六轴反馈连续三次到位；最大误差 {error:.2f}°')
                return 0
            time.sleep(.1)
        raise RuntimeError(f'等待真实反馈到位超时（总时限 {args.timeout:g}s，'
                           f'要求六轴误差≤{args.tolerance:g}°）')
    except (Exception, KeyboardInterrupt) as exc:
        if report is not None:
            report['status'] = 'stopped'
            report['stop_reason'] = str(exc) or '用户中断'
            sample = report['last_feedback_before_stop']
            if sample:
                print_angles(report['start_angles'], report['planned_goal_angles'],
                             sample['angles'], f'停止前最后一次有效反馈（t={sample["time_s"]:.3f}s）')
        if moving and client:
            try:
                hold_angles = client.read()
                if report is not None:
                    report['hold_feedback_angles'] = hold_angles.copy()
                client.write(hold_angles)
                print('已尝试将六轴目标设为当前反馈位置，未关闭力矩。', file=sys.stderr)
            except Exception as hold_error:
                print(f'保持位置请求失败：{hold_error}', file=sys.stderr)
        print(f'停止：{exc or "用户中断"}', file=sys.stderr)
        return 1
    finally:
        if report is not None and log_file is not None:
            try:
                report['elapsed_s'] = None if started is None else time.monotonic()-started
                log_file.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
            except OSError as log_error:
                print(f'执行日志保存失败：{log_error}', file=sys.stderr)
        if client:
            client.close()


if __name__ == '__main__':
    sys.exit(main())
