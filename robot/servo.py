#!/usr/bin/env python3
"""通过 Phosphobot HTTP 控制 SO100 六关节，无需断开服务或直接打开串口。"""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time
from zoneinfo import ZoneInfo
import requests

NAMES = ['底座', '肩', '肘', '腕俯仰', '腕旋转', '夹爪']


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

    def call(self, endpoint, body=None, get=False):
        response = self.session.request('GET' if get else 'POST', self.url+endpoint,
            params=None if get else {'robot_id': self.robot_id},
            json=None if get else ({} if body is None else body), timeout=5)
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

    def close(self):
        self.session.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:80')
    parser.add_argument('--robot-id', type=int, default=0)
    sub = parser.add_subparsers(dest='command', required=True)
    read = sub.add_parser('read', help='读取真机关节角度')
    read.add_argument('--ids', type=int, nargs='+', default=list(range(1, 7)))
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
    move.add_argument('--log-file', type=Path, help='执行记录 JSON 路径，默认 robot/logs/move-时间.json')
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
        if args.command != 'torque':
            if len(set(args.ids)) != len(args.ids) or any(i not in range(1, 7) for i in args.ids):
                raise ValueError('--ids 必须是 1..6 内不重复的 ID')
        if args.command == 'move':
            if not .1 <= args.speed <= 90 or not .1 <= args.acc <= 150 or not 1 <= args.hz <= 30:
                raise ValueError('speed 范围 0.1..90；acc 范围 0.1..150；hz 范围 1..30')
            if args.timeout <= 0 or not .1 <= args.tolerance <= 5 or not .1 <= args.tracking_error <= 15:
                raise ValueError('timeout 必须大于 0；tolerance 范围 0.1..5；tracking-error 范围 0.1..15')
            values = expanded(args.angles if args.angles is not None else args.delta, len(args.ids))
        client = Client(args.url, args.robot_id)
        config = client.precheck()
        current = client.read()
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
        dt = 1/args.hz
        frames = trajectory(current, goal, args.speed, args.acc, dt)
        if len(frames)*dt >= args.timeout:
            raise ValueError('规划时长超过 timeout，请增大 --timeout')
        print(f'软件规划：速度≤{args.speed}°/s，加速度≤{args.acc}°/s²，{len(frames)} 个目标')
        if not args.execute:
            print('仅预览；未发送运动或力矩指令。加 --execute 才执行。')
            return 0
        recorded_at = datetime.now(ZoneInfo('Asia/Shanghai'))
        log_file = args.log_file or Path(__file__).resolve().parent/'logs'/f'move-{recorded_at:%Y%m%d-%H%M%S-%f}.json'
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
