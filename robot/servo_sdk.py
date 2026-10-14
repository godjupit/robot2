#!/usr/bin/env python3
"""SO100 六舵机基础控制：直接使用 Feetech SDK，无 HTTP、无 IK。"""
import argparse
import json
import math
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import time

NAMES = ['底座', '肩', '肘', '腕俯仰', '腕旋转', '夹爪']
DEFAULT_CONFIG = Path.home() / 'phosphobot/calibration/so-100_58FA082487_config.json'


def finite(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError('参数必须是有限数字')
    return number


def ticks_to_angle(ticks, servo_id, config):
    i = servo_id - 1
    return (ticks - config['servos_offsets'][i]) * config['servos_offsets_signs'][i] * 360 / 4095


def angle_to_ticks(angle, servo_id, config):
    i = servo_id - 1
    return int(angle * config['servos_offsets_signs'][i] * 4095 / 360 + config['servos_offsets'][i])


def expanded(values, count, name):
    if len(values) == 1:
        return values * count
    if len(values) != count:
        raise ValueError(f'{name} 需填 1 个值，或者与 --ids 数量一致的 {count} 个值')
    return values


class Bus:
    def __init__(self, device, baud):
        # 防止两个进程同时访问同一条 USB 串口。
        lsof = shutil.which('lsof')
        if lsof:
            occupied = subprocess.run([lsof, '-t', device], capture_output=True, text=True, timeout=5)
            if occupied.stdout.strip():
                raise RuntimeError(f'串口已被进程 {occupied.stdout.split()} 占用。请先断开 Phosphobot 的机器人连接。')
        from scservo_sdk import PortHandler, PacketHandler, GroupSyncWrite, COMM_SUCCESS
        self.success = COMM_SUCCESS
        self.port = PortHandler(device)
        self.packet = PacketHandler(0)
        self.group_type = GroupSyncWrite
        if not self.port.openPort():
            raise RuntimeError(f'无法打开串口 {device}')
        if not self.port.setBaudRate(baud):
            self.port.closePort()
            raise RuntimeError(f'无法设置波特率 {baud}')

    def check(self, result, error=0):
        if result != self.success:
            raise RuntimeError(self.packet.getTxRxResult(result))
        if error:
            raise RuntimeError(self.packet.getRxPacketError(error))

    def read(self, sid, address, size=2):
        method = self.packet.read1ByteTxRx if size == 1 else self.packet.read2ByteTxRx
        value, result, error = method(self.port, sid, address)
        self.check(result, error)
        return value

    def torque(self, sid, enabled):
        self.check(*self.packet.write1ByteTxRx(self.port, sid, 40, int(enabled)))

    def goals(self, rows, speed, acc):
        group = self.group_type(self.port, self.packet, 41, 7)
        try:
            for sid, target in rows:
                if not group.addParam(sid, list(struct.pack('<BHHH', acc, target, 0, speed))):
                    raise RuntimeError(f'无法构建 ID{sid} 指令')
            self.check(group.txPacket())
        finally:
            group.clearParam()

    def snapshot(self, ids):
        rows = []
        for sid in ids:
            mode = self.read(sid, 33, 1)
            if mode != 0:
                raise RuntimeError(f'ID{sid} 不是位置模式，停止操作')
            rows.append({'id': sid, 'position': self.read(sid, 56),
                         'lower': self.read(sid, 9), 'upper': self.read(sid, 11),
                         'torque': self.read(sid, 40, 1)})
        return rows

    def hold(self, ids, speed, acc):
        # 中断时尽力把目标更新为当前反馈，保留力矩，避免主动释放后掉落。
        self.goals([(sid, self.read(sid, 56)) for sid in ids], speed, acc)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/cu.usbmodem58FA0824871')
    parser.add_argument('--baudrate', type=int, default=1_000_000)
    parser.add_argument('--calibration', type=Path, default=DEFAULT_CONFIG)
    sub = parser.add_subparsers(dest='command', required=True)
    read = sub.add_parser('read', help='只读位置、角度、限位和力矩')
    read.add_argument('--ids', nargs='+', type=int, default=list(range(1, 7)))
    move = sub.add_parser('move', help='设置一个或多个舵机的角度；默认只预览')
    move.add_argument('--ids', nargs='+', type=int, required=True)
    target = move.add_mutually_exclusive_group(required=True)
    target.add_argument('--angles', nargs='+', type=finite, help='绝对关节角度，单位度')
    target.add_argument('--delta', nargs='+', type=finite, help='相对当前角度移动，单位度')
    move.add_argument('--speed', type=finite, default=5, help='速度上限，度/秒，默认 5，范围 0.1..90')
    move.add_argument('--acc', type=int, default=10, help='加速度原始参数 1..150，越小越缓慢')
    move.add_argument('--timeout', type=finite, default=30, help='等待到位秒数')
    move.add_argument('--tolerance', type=int, default=12, help='到位误差，编码器步数，默认 12')
    move.add_argument('--execute', action='store_true')
    torque = sub.add_parser('torque', help='开关力矩；默认只预览')
    torque.add_argument('state', choices=['on', 'off'])
    torque.add_argument('--ids', nargs='+', type=int, required=True)
    torque.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    bus = None
    moving = False
    try:
        if len(set(args.ids)) != len(args.ids) or any(i not in range(1, 7) for i in args.ids):
            raise ValueError('--ids 必须是 1..6 内不重复的舵机 ID')
        config = json.loads(args.calibration.read_text())
        for key in ('servos_offsets', 'servos_offsets_signs'):
            if len(config[key]) != 6 or any(not math.isfinite(v) for v in config[key]):
                raise ValueError('标定必须包含六个有限数值')
        if any(v not in (-1, 1) for v in config['servos_offsets_signs']):
            raise ValueError('标定方向必须为 -1 或 1')
        if args.command == 'move':
            if not .1 <= args.speed <= 90 or not 1 <= args.acc <= 150:
                raise ValueError('--speed 范围 0.1..90；--acc 范围 1..150')
            if args.timeout <= 0 or not 0 <= args.tolerance <= 100:
                raise ValueError('--timeout 必须大于零；--tolerance 范围 0..100')
            values = expanded(args.angles if args.angles is not None else args.delta, len(args.ids), '角度')
        bus = Bus(args.port, args.baudrate)
        rows = bus.snapshot(args.ids)
        if args.command == 'read':
            for row in rows:
                sid = row['id']
                print(f"ID{sid} {NAMES[sid-1]}: {ticks_to_angle(row['position'], sid, config):.2f}° "
                      f"ticks={row['position']} 限位=[{row['lower']},{row['upper']}] 力矩={row['torque']}")
            return 0
        if args.command == 'torque':
            print(f'ID {args.ids}: 力矩 {args.state}。关闭力矩前请托住机械臂。')
            if not args.execute:
                print('仅预览，未写入；添加 --execute 执行。')
                return 0
            if args.state == 'on':
                # 先清除失能舵机的旧目标，避免开启力矩后奔向历史目标。
                disabled = [(r['id'], r['position']) for r in rows if r['torque'] == 0]
                if disabled:
                    bus.goals(disabled, 57, 10)
            for row in rows:
                bus.torque(row['id'], args.state == 'on')
            print('力矩指令已发送。')
            return 0
        goals = []
        speed = max(1, round(args.speed * 4096 / 360))
        for row, value in zip(rows, values):
            sid = row['id']
            current = ticks_to_angle(row['position'], sid, config)
            angle = value if args.angles is not None else current + value
            target_ticks = angle_to_ticks(angle, sid, config)
            if not 0 <= target_ticks <= 4095 or not row['lower'] <= target_ticks <= row['upper']:
                raise ValueError(f'ID{sid}: 目标 {angle:.2f}° / {target_ticks} 超出舵机限位')
            goals.append((sid, target_ticks))
            print(f'ID{sid} {NAMES[sid-1]}: {current:.2f}° → {angle:.2f}° / ticks={target_ticks}')
        print(f'速度上限={args.speed}°/s（寄存器 {speed} 步/s），加速度原始值={args.acc}')
        if not args.execute:
            print('仅预览，未写入；添加 --execute 执行。')
            return 0
        if any(r['torque'] != 1 for r in rows):
            raise RuntimeError('所选舵机力矩未全部开启；请先使用 torque on 命令。未发送运动目标。')
        moving = True
        bus.goals(goals, speed, args.acc)
        deadline = time.monotonic() + args.timeout
        stable = 0
        while time.monotonic() < deadline:
            actual = [(sid, bus.read(sid, 56)) for sid, _ in goals]
            errors = [abs(value-target) for (_, value), (_, target) in zip(actual, goals)]
            stable = stable+1 if max(errors) <= args.tolerance else 0
            if stable >= 3:
                moving = False
                print('真实位置反馈连续三次在误差范围内：', actual)
                return 0
            time.sleep(.1)
        raise RuntimeError(f'等待到位超时，最后误差 ticks={errors}')
    except (Exception, KeyboardInterrupt) as exc:
        if moving and bus:
            try:
                bus.hold(args.ids, speed, args.acc)
                print('已尝试将目标设为当前反馈位置；力矩保持开启。', file=sys.stderr)
            except Exception as stop_error:
                print(f'保持当前位置失败：{stop_error}', file=sys.stderr)
        print(f'停止：{exc or "用户中断"}', file=sys.stderr)
        return 1
    finally:
        if bus:
            bus.port.closePort()


if __name__ == '__main__':
    sys.exit(main())
