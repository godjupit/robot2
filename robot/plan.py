#!/usr/bin/env python3
"""Phosphobot 真机关节反馈规划：Z=15 cm 的 10 cm 水平线，默认三个目标点。

当前 Phosphobot 0.3.134 没有独立的只规划/IK HTTP 接口。
默认只生成路径；--solve 用原始 SO100 模型独立求 IK 和检查模型；
--execute 才按真实反馈逐点补偿执行。每次单独保存规划和实际日志。
"""
import argparse
from datetime import datetime
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from zoneinfo import ZoneInfo

from servo import Client, forward_xyz, finite, NAMES

ROOT = Path(__file__).resolve().parent


class FeedbackClient:
    """通过原生接口逐舵机读取位置，避免全组读缓存参与到位判定。"""
    def __init__(self, client, config):
        self.client, self.config = client, config
        self.last_ticks = None

    def call(self, *args, **kwargs):
        return self.client.call(*args, **kwargs)

    def read(self):
        # 非默认顺序使当前 SO100 实现走单舵机读取分支。使用 motor_units
        # 避免服务对子集/重排后的数组使用错误的角度标定索引。
        ids = [6,5,4,3,2,1]
        data = self.call('/joints/read',{'unit':'motor_units','source':'robot','joints_ids':ids})
        values = data.get('angles',[])
        if (data.get('unit') != 'motor_units' or len(values) != 6
                or any(v is None or not math.isfinite(v) or not 0 <= v <= 4095 for v in values)):
            raise RuntimeError('逐舵机原始位置反馈无效，停止运动')
        ticks = dict(zip(ids,values))
        self.last_ticks = [ticks[i] for i in range(1,7)]
        return [(ticks[i]-self.config['servos_offsets'][i-1])
                *self.config['servos_offsets_signs'][i-1]*360/4095 for i in range(1,7)]


class Model:
    """Phosphobot 原始 SO100 URDF；独立 SciPy IK + 原始网格碰撞检查。"""
    def __init__(self, tcp_offset_cm=None):
        import numpy as np
        import trimesh
        from scipy.spatial.transform import Rotation
        from trimesh.collision import CollisionManager
        self.np, self.rotation = np, Rotation
        self.tcp_mode = tcp_offset_cm is not None
        self.tcp_offset = np.asarray([0.,0.,0.] if tcp_offset_cm is None else tcp_offset_cm,dtype=float)/100
        if self.tcp_offset.shape != (3,) or not np.isfinite(self.tcp_offset).all():
            raise ValueError('TCP 偏移必须为 Fixed_Jaw 局部坐标系中的三个有限数值（cm）')
        self.root = ET.parse(ROOT/'models/so-100.urdf').getroot()
        self.joints = self.root.findall('joint')
        self.lower = np.array([float(j.find('limit').get('lower')) for j in self.joints[:5]])
        self.upper = np.array([float(j.find('limit').get('upper')) for j in self.joints[:5]])
        self.origins = [self.origin(j.find('origin')) for j in self.joints]
        self.axes = [np.fromstring(j.find('axis').get('xyz'),sep=' ') for j in self.joints]
        self.manager = CollisionManager()
        self.geometry = []
        adjacent = {frozenset((j.find('parent').get('link'),j.find('child').get('link'))) for j in self.joints}
        assets = ROOT.parent/'phosphobot-main/phosphobot/resources/urdf'
        for link in self.root.findall('link'):
            for i, collision in enumerate(link.findall('collision')):
                mesh_info = collision.find('geometry/mesh')
                mesh = trimesh.load_mesh(assets/mesh_info.get('filename').replace('package://',''),process=False)
                mesh.apply_scale(np.fromstring(mesh_info.get('scale','1 1 1'),sep=' '))
                if not np.isfinite(mesh.vertices).all():
                    raise RuntimeError('碰撞网格包含非有限坐标')
                name = f'{link.get("name")}:{i}'
                self.manager.add_object(name,mesh)
                self.geometry.append((name,link.get('name'),self.origin(collision.find('origin')),mesh.vertices))
        self.pairs = [(a,b) for i,a in enumerate(self.manager._objs) for b in list(self.manager._objs)[i+1:]
                      if frozenset((a.split(':')[0],b.split(':')[0])) not in adjacent
                      and a.split(':')[0] != b.split(':')[0]]

    def origin(self, element):
        np = self.np
        t = np.eye(4)
        if element is not None:
            t[:3,:3] = self.rotation.from_euler('xyz',np.fromstring(element.get('rpy','0 0 0'),sep=' ')).as_matrix()
            t[:3,3] = np.fromstring(element.get('xyz','0 0 0'),sep=' ')
        return t

    def transforms(self, q, jaw):
        np = self.np
        base = np.eye(4);base[:3,:3] = self.rotation.from_quat([0,0,1,1]).as_matrix()
        transforms = {'Base':base}
        for j,origin,axis,angle in zip(self.joints,self.origins,self.axes,list(q)+[jaw]):
            motion = np.eye(4);motion[:3,:3] = self.rotation.from_rotvec(axis*angle).as_matrix()
            transforms[j.find('child').get('link')] = transforms[j.find('parent').get('link')]@origin@motion
        return transforms

    def position(self, q):
        t = self.transforms(q,0)['Fixed_Jaw']
        return t[:3,3]+t[:3,:3]@self.tcp_offset

    def solve_position(self, target, seed):
        """位置优先 TCP IK：XYZ 残差为 mm，不加入工具朝向约束。"""
        from scipy.optimize import least_squares
        np = self.np
        return least_squares(lambda angles: np.r_[(self.position(angles)-target)*1000,
                                                   (angles-seed)*.001],
                             seed,bounds=(self.lower,self.upper),max_nfev=200).x

    def check(self, q, jaw):
        import fcl
        np = self.np
        if not np.isfinite(q).all() or not math.isfinite(jaw):
            raise RuntimeError('碰撞检查关节角无效')
        if np.any(q < self.lower-1e-9) or np.any(q > self.upper+1e-9):
            raise RuntimeError('关节角超出 URDF 限位')
        transforms = self.transforms(q,jaw)
        for name,link,origin,vertices in self.geometry:
            t = transforms[link]@origin
            self.manager.set_transform(name,t)
            if link not in ('Base','Rotation_Pitch'):
                z = np.sum(vertices*t[2,:3],axis=1)+t[2,3]
                if not np.isfinite(z).all():
                    raise RuntimeError('碰撞变换产生非有限坐标')
                if z.min() < .005:
                    raise RuntimeError(f'{link} 网格距模型 Z=0 平面不足 5 mm')
        for a,b in self.pairs:
            objects = self.manager._objs
            if fcl.collide(objects[a]['obj'],objects[b]['obj'],fcl.CollisionRequest(),fcl.CollisionResult()):
                raise RuntimeError(f'模型自碰撞：{a} / {b}')


def solve_plan(plan, model=None):
    import numpy as np
    model = model if model is not None else Model(plan.get('tcp_offset_cm'))
    q = np.deg2rad(plan['source_joint_angles_deg'][:5])
    jaw = math.radians(plan['source_joint_angles_deg'][5])
    model.check(q,jaw)
    solutions = []
    targets = [('approach',p) for p in plan['approach_points_cm'][1:]]+ [('draw',p) for p in plan['drawing_points_cm'][1:]]
    if plan.get('return_home'):
        # 回原点指五个运动关节的零位，夹爪保持当前角度。
        home=np.zeros(5)
        home_xyz=(model.position(home)*100).tolist()
        for fraction in np.linspace(0,1,max(3,math.ceil(float(np.rad2deg(abs(q)).max())/.2)+1)):
            model.check(q*(1-fraction),jaw)
        solutions.append({'phase':'home','xyz_cm':home_xyz,'joint_angles_deg':[0.]*5,
                          'ik_error_mm':0.,'max_joint_step_deg':float(np.rad2deg(abs(q)).max())})
        q=home
        targets=[('approach',plan['drawing_points_cm'][0])]+[('draw',p) for p in plan['drawing_points_cm'][1:]]
    if model.tcp_mode:
        # 长工具从当前姿态接近起点时，细分 XYZ，逐点检查关节跳变和碰撞。
        expanded=[]
        preceding=(model.position(q)*100).tolist()
        approach=[preceding]
        for phase,point in targets:
            points=segment(preceding,point,1.)[1:] if phase=='approach' else [point]
            expanded.extend((phase,p) for p in points)
            if phase=='approach':
                approach.extend(points)
            preceding=point
        targets=expanded
        plan['approach_points_cm']=approach
    targets += [('lift',p) for p in plan.get('lift_points_cm',[])]
    if plan.get('stroke_targets'):
        targets=[(p['phase'],p['xyz_cm']) for p in plan['stroke_targets']]
    for index,(phase,point) in enumerate(targets):
        target = np.asarray(point)/100
        seed = q.copy()
        q = model.solve_position(target,seed)
        error = float(np.linalg.norm(model.position(q)-target)*1000)
        step_deg = float(np.rad2deg(np.abs(q-seed)).max())
        # 粗采样目标之间允许较大角差，执行层仍将发送角度拆成 0.5° 小步。
        if error > .5 or step_deg > 60:
            raise RuntimeError(f'第 {index+1} 点预检失败：位置误差 {error:.3f} mm，关节跳变 {step_deg:.3f}°')
        # 检查关节插值中间姿态，避免只检查端点。
        for fraction in np.linspace(0,1,max(3,math.ceil(step_deg/.2)+1)):
            model.check(seed+fraction*(q-seed),jaw)
        solutions.append({'phase':phase,'xyz_cm':point,'joint_angles_deg':np.rad2deg(q).tolist(),
                          'ik_error_mm':error,'max_joint_step_deg':step_deg})
        if plan.get('stroke_targets'):
            solutions[-1].update({k:v for k,v in plan['stroke_targets'][index].items() if k!='xyz_cm'})
    plan.update({'ik_validated':True,'collision_checked':True,'joint_limits_checked':True,
                 'solver':'Independent SciPy position-only TCP IK' if model.tcp_mode else 'Independent SciPy position-only end-frame IK',
                 'collision_scope':'Original SO100 meshes, nonadjacent self collisions and distal clearance above model Z=0. Rod volume and external objects are not collision modeled.',
                 'orientation':'Free orientation; XYZ residual in mm with weak 0.001 joint continuity regularization',
                 'solutions':solutions})
    return model


def choose_drawing_compensation(model, actual, command, error, goal_xyz, mask,
                                paper_z_cm, step_deg=.5, shoulder_step_deg=1., elbow_step_deg=1.,
                                z_below_tolerance_mm=0.):
    """每轮只增加一个轴的目标，预测各轴不同步时的 Z，优先安全的 XYZ 改善。"""
    from itertools import product
    np=model.np
    actual=np.asarray(actual[:5],dtype=float)
    command=np.asarray(command,dtype=float)
    current=model.position(np.deg2rad(actual))*100
    # 候选预测和实测保护使用同一个纸面下方阈值，不额外强制 Z>=纸面。
    floor=paper_z_cm-z_below_tolerance_mm/10
    candidates=[]
    for joint in np.flatnonzero(mask):
        limit=shoulder_step_deg if joint==1 else elbow_step_deg if joint==2 else step_deg
        delta=float(np.clip(error[joint],-limit,limit))
        for scale in (1.,.5,.25):
            proposed=command.copy();proposed[joint]+=delta*scale
            # 使用真实角度到所有待执行发送目标的组合，而非假设五轴同步。
            z_values=[]
            for bits in product((0,1),repeat=5):
                q=np.where(bits,proposed,actual)
                z_values.append(float(model.position(np.deg2rad(q))[2]*100))
            # 有限模型采样不代表真实运动保证。
            for fraction in np.linspace(0,1,11):
                q=actual+fraction*(proposed-actual)
                z_values.append(float(model.position(np.deg2rad(q))[2]*100))
            predicted=actual.copy();predicted[joint]+=delta*scale
            xyz=model.position(np.deg2rad(predicted))*100
            minimum=min(z_values)
            if minimum < floor-1e-9:
                continue
            improvement=float(np.linalg.norm(current-goal_xyz)-np.linalg.norm(xyz-goal_xyz))
            rise=float(xyz[2]-current[2])
            if improvement <= 1e-9 and rise <= 1e-9:
                continue
            rank=(rise,improvement) if current[2]-paper_z_cm < .2 else (improvement,rise)
            candidates.append((rank,proposed,joint,xyz,minimum))
            break
    if not candidates:
        raise RuntimeError('绘图补偿预检：没有可改善位置且满足 Z 阈值的单轴调整，停止整条轨迹')
    _,proposed,joint,xyz,minimum=max(candidates,key=lambda item:item[0])
    return proposed,{'joint_id':int(joint)+1,'joint_name':NAMES[joint],
                     'predicted_next_xyz_cm':xyz.tolist(),
                     'predicted_min_z_cm':minimum,'prediction_floor_z_cm':floor}


def move_point_with_compensation(client, model, point, command, write_arm, record,
                                 deadline, joint_tolerance=2., xyz_tolerance_mm=10.,
                                 point_timeout=60., max_compensation=15., step_deg=.5,
                                 interval=.25, paper_z_cm=None, z_below_tolerance_mm=5.,
                                 shoulder_step_deg=1., elbow_step_deg=1.):
    """逐点闭环；肩和肘每次最多 1°，其余运动关节每次最多 0.5°。"""
    np = model.np
    goal = np.asarray(point['joint_angles_deg'],dtype=float)
    goal_xyz = np.asarray(point['xyz_cm'],dtype=float)
    command = np.asarray(command,dtype=float).copy()
    first = np.asarray(client.read()[:5])
    lower_command = np.minimum(first,goal)-max_compensation
    upper_command = np.maximum(first,goal)+max_compensation
    started = time.monotonic()
    stable = unchanged = 0
    previous = None
    for iteration in range(1000):
        if time.monotonic() >= deadline or time.monotonic()-started >= point_timeout:
            raise RuntimeError('当前目标点反馈补偿超时')
        actual = client.read()
        q_actual = np.deg2rad(actual[:5])
        model.check(q_actual,math.radians(actual[5]))
        xyz = model.position(q_actual)*100
        error = goal-np.asarray(actual[:5])
        position_error_mm = float(np.linalg.norm(xyz-goal_xyz)*10)
        sample = {'actual_angles_deg':actual.copy(),'actual_xyz_cm':xyz.tolist(),
                  'goal_joint_angles_deg':goal.tolist(),'goal_xyz_cm':goal_xyz.tolist(),
                  'last_command_angles_deg':command.tolist(),
                  'joint_errors_deg':error.tolist(),'max_joint_error_deg':float(max(abs(error))),
                  'position_error_mm':position_error_mm}
        if getattr(client,'last_ticks',None) is not None:
            sample['motor_position_units']=client.last_ticks.copy()
        if model.tcp_mode and point['phase'] in ('draw','lower'):
            paper_z = goal_xyz[2] if paper_z_cm is None else paper_z_cm
            below_mm = (paper_z-float(xyz[2]))*10
            sample['below_paper_mm']=below_mm
        saved_sample=record(sample)
        # 绘图段高度保护先于到位判断，不受 XYZ 到位容差影响。
        if model.tcp_mode and point['phase'] in ('draw','lower'):
            if below_mm > z_below_tolerance_mm:
                raise RuntimeError(f'绘图 Z 高度保护：TCP Z={xyz[2]:.4f} cm，'
                                   f'低于纸面 Z={paper_z:g} cm {below_mm:.2f} mm，'
                                   f'超过允许 {z_below_tolerance_mm:g} mm，停止整条轨迹')
        if 'min_z_cm' in point and xyz[2] < point['min_z_cm']:
            raise RuntimeError(f'抬笔转移高度保护：TCP Z={xyz[2]:.4f} cm，低于要求 {point["min_z_cm"]:g} cm，停止整条轨迹')
        # TCP 绘图只要求尖端 XYZ；回零仍要求五轴角度到位。
        position_only = model.tcp_mode and point['phase'] != 'home'
        arrived = position_error_mm <= xyz_tolerance_mm and (position_only or max(abs(error)) <= joint_tolerance)
        stable = stable+1 if arrived else 0
        if stable >= 2:
            return command,sample
        if iteration % 10 == 0:
            worst = int(np.argmax(abs(error)))
            print(f'  补偿 {iteration:03d}: 最大误差关节 ID{worst+1}（{NAMES[worst]}）'
                  f' {abs(error[worst]):.2f}°，'
                  f'XYZ 误差 {position_error_mm:.2f} mm',flush=True)
        if arrived:
            # 不重新发送名义角度，保留已经有效的补偿目标。
            if iteration % 10 == 0:
                print('    已满足到位条件，等待连续反馈确认；本次不补偿。',flush=True)
            time.sleep(interval)
            continue
        unchanged = unchanged+1 if previous is not None and max(abs(np.asarray(actual[:5])-previous)) < .04 else 0
        previous = np.asarray(actual[:5])
        if unchanged >= 8:
            raise RuntimeError('连续 8 次反馈未变化且未到位，停止补偿')
        # 只根据实际残差调整，不把“发送成功”当成到位。
        mask = abs(error) > (.1 if position_error_mm > xyz_tolerance_mm else joint_tolerance)
        if model.tcp_mode and point['phase'] in ('draw','lower'):
            proposed,choice=choose_drawing_compensation(model,actual,command,error,goal_xyz,mask,paper_z,step_deg,shoulder_step_deg,elbow_step_deg,z_below_tolerance_mm)
            sample['compensation_choice']=choice
            if isinstance(saved_sample,dict):
                saved_sample['compensation_choice']=choice.copy()
            mask=abs(proposed-command)>1e-12
            if iteration % 10 == 0:
                print(f'    Z 预测选择 ID{choice["joint_id"]}（{choice["joint_name"]}）：'
                      f'当前 Z={xyz[2]:.4f} cm，预计单轴调整后 Z={choice["predicted_next_xyz_cm"][2]:.4f} cm，'
                      f'待执行角度组合最低 Z={choice["predicted_min_z_cm"]:.4f} cm，'
                      f'保护下限 Z={choice["prediction_floor_z_cm"]:.4f} cm',flush=True)
        else:
            limits=np.full(5,step_deg);limits[1]=shoulder_step_deg;limits[2]=elbow_step_deg
            proposed = command+np.where(mask,np.clip(error,-limits,limits),0.)
        if np.any(proposed < lower_command) or np.any(proposed > upper_command):
            raise RuntimeError('当前点补偿超出起始/目标范围外 15° 的上限')
        if max(abs(proposed-np.asarray(actual[:5]))) > max_compensation+3:
            raise RuntimeError('发送角度与实际角度偏差过大，停止补偿')
        if not np.allclose(proposed,command,atol=1e-12,rtol=0):
            write_arm(proposed)
            if iteration % 10 == 0:
                for joint in np.flatnonzero(mask):
                    print(f'    ID{joint+1}（{NAMES[joint]}）: '
                          f'实测 {actual[joint]:.2f}°，规划目标 {goal[joint]:.2f}°，'
                          f'本次发送目标调整 {proposed[joint]-command[joint]:+.2f}°，'
                          f'发送目标 {command[joint]:.2f}° → {proposed[joint]:.2f}°',flush=True)
            command = proposed
        time.sleep(interval)
    raise RuntimeError('当前点达到最大补偿循环次数')


def execute_plan(client, model, plan, config, destination, joint_tolerance=2.,
                 xyz_tolerance_mm=10., point_timeout=60., total_timeout=360.):
    """按粗采样目标逐点反馈补偿，记录规划/发送/实测；不发送夹爪目标。"""
    import csv
    np = model.np
    initial = client.read()
    seed = plan['source_joint_angles_deg']
    if max(abs(a-b) for a,b in zip(initial,seed)) > .5:
        raise RuntimeError('真机姿态已偏离规划起点，未发送运动')
    settings=plan.get('execution_settings',{})
    steps=settings.get('control_step_deg_by_joint',[.5,1.,1.,.5,.5])
    if len(steps)!=5 or any(not math.isfinite(v) or not 0<v<=1 for v in steps) or steps[0]!=steps[3] or steps[0]!=steps[4]:
        raise ValueError('每轴补偿步长配置无效，未发送运动')
    result = {'motion_sent':False,'status':'running','samples':[],'commands':[],
              'completed_points':0,'waypoint_results':[],
              'completed_strokes':0,
              'line_completed':False,'lift_completed':False,
              'reference_point':plan.get('reference_point'),
              'tcp_offset_cm':plan.get('tcp_offset_cm'),
              'arrival_mode':'TCP XYZ only; home also checks joint angles' if model.tcp_mode else 'joint angles and end-frame XYZ',
              'planning_file':str(destination.resolve()),
              'arrival_joint_tolerance_deg':joint_tolerance,
              'arrival_xyz_tolerance_mm':xyz_tolerance_mm,
              'drawing_z_protection':plan.get('drawing_z_protection'),
              'drawing_compensation':'One changed joint target per round; FK prediction of pending joint combinations; lift priority within 2 mm of paper',
              'point_timeout_s':point_timeout,'control_step_deg':steps[0],
              'control_step_deg_by_joint':steps,
              'total_timeout_s':total_timeout,
              'feedback_interval_s':.25,'max_compensation_deg':15.}
    result['execution_api'] = 'POST /joints/write, first five angles in radians, no joints_ids; no gripper write'
    log = destination.parent/'execution.json'
    stream_path = destination.parent/'actual_feedback.jsonl'
    log.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    bias = np.zeros(5)
    init_logs = sorted((ROOT/'垃圾文件夹/logs').glob('init-*.json'),reverse=True)
    if init_logs and max(abs(a) for a in initial[:5]) <= 1:
        previous = json.loads(init_logs[0].read_text())
        if previous.get('status')=='completed':
            bias = np.array(previous['last_command_angles'][:5])-np.array(previous['last_actual_angles'][:5])
    result['initial_feedforward_deg'] = bias.tolist()
    command = np.array(initial[:5])+bias
    if 'initial_command_angles_deg' in plan:
        reference=np.array(plan['initial_command_feedback_angles_deg'])
        known=np.array(plan['initial_command_angles_deg'])
        if reference.shape!=(5,) or known.shape!=(5,) or max(abs(reference-np.array(initial[:5])))>3:
            raise RuntimeError('实际姿态与已记录的保持目标不匹配，未发送运动')
        command=known.copy()
        result['initial_command_source']=plan.get('initial_command_source')
        result['initial_feedforward_deg']=(command-np.array(initial[:5])).tolist()
    result['initial_command_angles_deg']=command.tolist()
    started = time.monotonic()
    stream = stream_path.open('w')
    def write_arm(values):
        raw = values*np.array(config['servos_offsets_signs'][:5])*4095/360+np.array(config['servos_offsets'][:5])
        if (not np.isfinite(values).all() or np.any(raw<0) or np.any(raw>4095)
                or np.any(np.deg2rad(values)<model.lower) or np.any(np.deg2rad(values)>model.upper)):
            raise RuntimeError('补偿后的发送目标超出角度/编码器限位')
        result['motion_sent'] = True
        client.call('/joints/write',{'unit':'rad','angles':np.deg2rad(values).tolist()})
        entry={'time_s':time.monotonic()-started,'angles_deg':values.tolist()}
        if result['status']=='running' and result['samples']:
            choice=result['samples'][-1].get('compensation_choice')
            if choice is not None:
                entry['compensation_choice']=choice.copy()
        result['commands'].append(entry)
    try:
        for index,point in enumerate(plan['solutions']):
            print(f'目标 {index+1}/{len(plan["solutions"])}: XYZ={np.round(point["xyz_cm"],3).tolist()} cm；'
                  f'规划角度={np.round(point["joint_angles_deg"],2).tolist()}°；'
                  f'阶段={point["phase"]}'+(f'，第 {point["stroke_id"]} 笔' if 'stroke_id' in point else ''),flush=True)
            def record(sample):
                row={'time_s':time.monotonic()-started,'point':index+1,'phase':point['phase'],**sample}
                if 'stroke_id' in point:
                    row['stroke_id']=point['stroke_id']
                result['samples'].append(row)
                result['last_actual_angles']=sample['actual_angles_deg'].copy()
                stream.write(json.dumps(row,ensure_ascii=False)+'\n');stream.flush()
                return row
            command,actual = move_point_with_compensation(
                client,model,point,command,write_arm,record,started+total_timeout,
                joint_tolerance,xyz_tolerance_mm,point_timeout,
                step_deg=steps[0],shoulder_step_deg=steps[1],elbow_step_deg=steps[2],
                paper_z_cm=plan.get('drawing_z_protection',{}).get('paper_z_cm'),
                z_below_tolerance_mm=plan.get('drawing_z_protection',{}).get('z_below_tolerance_mm',5.))
            result['completed_points']=index+1
            if point.get('stroke_finished'):
                result['completed_strokes']+=1
                print(f'第 {point["stroke_id"]} 笔完成并抬笔到位。',flush=True)
            if point['phase']=='draw' and math.dist(point['xyz_cm'],plan['line_end_cm']) < 1e-8:
                result['line_completed']=True
            if point['phase']=='lift':
                result['lift_completed']=True
            result['waypoint_results'].append({'point':index+1,**actual})
            print(f'  到位：实测 XYZ={np.round(actual["actual_xyz_cm"],3).tolist()} cm，'
                  f'关节误差 {actual["max_joint_error_deg"]:.2f}°，XYZ误差 {actual["position_error_mm"]:.2f} mm',flush=True)
        result['status']='completed'
        print('三个目标点的反馈检查已完成。' if len(plan['solutions'])==3 else '全部目标点的反馈检查已完成。',flush=True)
    except BaseException as exc:
        result['status']='stopped';result['reason']=str(exc) or '用户中断'
        if result['motion_sent']:
            try:
                hold=client.read()
                write_arm(np.array(hold[:5]))
                result['hold_requested_angles']=hold[:5]
            except Exception as hold_error:
                result['hold_error']=str(hold_error)
        raise
    finally:
        stream.close()
        result['elapsed_s']=time.monotonic()-started
        log.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        with (destination.parent/'actual_feedback.csv').open('w',newline='') as f:
            writer=csv.writer(f)
            writer.writerow(['time_s','point','phase','x_cm','y_cm','z_cm','max_joint_error_deg','position_error_mm'])
            for row in result['samples']:
                writer.writerow([row['time_s'],row['point'],row['phase'],*row['actual_xyz_cm'],row['max_joint_error_deg'],row['position_error_mm']])
        if result['samples']:
            try:
                save_execution_preview(plan,result,destination.parent/'actual_path.png')
            except Exception as preview_error:
                print(f'实际轨迹预览生成失败：{preview_error}',flush=True)
        print(f'规划日志：{destination}\n实际执行日志：{log}\n逐帧反馈：{stream_path}',flush=True)


def segment(start, end, spacing_cm):
    distance = math.dist(start, end)
    steps = max(1, math.ceil(distance/spacing_cm-1e-12))
    return [[a+(b-a)*i/steps for a,b in zip(start,end)] for i in range(steps+1)]


def insert_line_points(plan, count):
    """起点终点之间均匀插入 count 个点，不移动端点。"""
    if not isinstance(count,int) or not 0 <= count <= 998:
        raise ValueError('插入点数范围 0..998')
    start,end=plan['line_start_cm'],plan['line_end_cm']
    plan['drawing_points_cm']=[[a+(b-a)*i/(count+1) for a,b in zip(start,end)] for i in range(count+2)]
    plan['drawing_point_count']=count+2
    plan['inserted_points']=count
    plan['sampling_mode']='uniform_inserted_points'
    plan['max_spacing_cm']=math.dist(start,end)/(count+1)
    # 该清单只用于参考；执行使用完整求解后的关节请求。
    plan['phosphobot_requests']=[]


def resample_replay_points(plan, spacing_cm):
    """按距离重新采样直线，保留起终点。"""
    sampled=segment(plan['line_start_cm'],plan['line_end_cm'],spacing_cm)
    plan.update(drawing_points_cm=sampled,drawing_point_count=len(sampled),
                sampling_mode='dense_spacing',
                max_spacing_cm=max(math.dist(a,b) for a,b in zip(sampled,sampled[1:])),
                phosphobot_requests=[])


def add_final_lift(plan, lift_cm):
    if not math.isfinite(lift_cm) or not 0 <= lift_cm <= 10:
        raise ValueError('抬升高度范围 0..10 cm')
    plan['lift_cm']=lift_cm
    plan['lift_points_cm']=[]
    if lift_cm:
        point=list(plan['line_end_cm']);point[2]+=lift_cm
        plan['lift_points_cm']=[point]


def build_plan(current_xyz_cm, z_cm=15., length_cm=10., axis='x', direction=-1, spacing_cm=.2, points=3):
    if (len(current_xyz_cm) != 3 or any(not math.isfinite(v) for v in current_xyz_cm)
            or not all(math.isfinite(v) for v in (z_cm,length_cm,spacing_cm))
            or length_cm <= 0 or spacing_cm <= 0 or axis not in ('x','y') or direction not in (-1,1)):
        raise ValueError('路径参数无效')
    if points is not None and (not isinstance(points,int) or not 2 <= points <= 1000):
        raise ValueError('points 必须是 2..1000 的整数')
    start = [current_xyz_cm[0],current_xyz_cm[1],z_cm]
    end = start.copy()
    end[0 if axis=='x' else 1] += direction*length_cm
    if points is None:
        approach = segment(current_xyz_cm,start,spacing_cm)
        stroke = segment(start,end,spacing_cm)
    else:
        approach = [list(current_xyz_cm),start.copy()]
        stroke = [[a+(b-a)*i/(points-1) for a,b in zip(start,end)] for i in range(points)]
    commands = []
    previous = list(current_xyz_cm)
    for phase, phase_points in [('approach',approach[1:]),('draw',stroke[1:])]:
        for point in phase_points:
            delta = [b-a for a,b in zip(previous,point)]
            if math.dist(previous,point) > 1e-10:
                commands.append({'phase':phase,'method':'POST','endpoint':'/move/relative',
                                 'body':dict(zip(('x','y','z'),delta)),
                                 'target_model_xyz_cm':point.copy()})
            previous = point
    return {'coordinate_frame':'SO100 model base origin, Phosphobot SO100 axis orientation',
            'unit':'cm','reference_point':'URDF end-effector link frame; no rod/TCP offset',
            'current_xyz_cm':list(current_xyz_cm),'line_start_cm':start,'line_end_cm':end,
            'line_length_cm':math.dist(start,end),'line_z_cm':z_cm,
            'axis':axis,'direction':direction,'sampling_mode':'dense_spacing' if points is None else 'coarse_waypoints',
            'drawing_point_count':len(stroke),
            'max_spacing_cm':spacing_cm if points is None else length_cm/(points-1),
            'approach_points_cm':approach,'drawing_points_cm':stroke,
            'phosphobot_requests':commands,
            'request_notes':['Translation deltas are centimeters; these are motion APIs, not a planning API.',
                             'No gripper command or initialization request is included.',
                             'The current service relative-move handler retains model end-frame orientation.',
                             'Execution must separately check real feedback and model synchronization.'],
            'motion_sent':False,'ik_validated':False,'collision_checked':False,
            'joint_limits_checked':False}


def save_preview(plan, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(10,4))
    ax=fig.add_subplot(121,projection='3d')
    side=fig.add_subplot(122)
    approach=plan['approach_points_cm'];stroke=plan['drawing_points_cm']
    segments=[(approach,f'Approach Z={plan["line_z_cm"]:g} cm','#d97706'),
              (stroke,f'{plan["line_length_cm"]:g} cm drawing line','#2563eb')]
    if plan.get('lift_points_cm'):
        segments.append(([plan['line_end_cm']]+plan['lift_points_cm'],
                         f'Final lift {plan["lift_cm"]:g} cm','#9333ea'))
    for points,label,color in segments:
        ax.plot(*zip(*points),color=color,label=label,linewidth=2)
        distances=[math.dist(points[0],p) for p in points]
        side.plot(distances,[p[2] for p in points],color=color,label=label,linewidth=2)
    ax.scatter(*plan['current_xyz_cm'],color='black',label='Current end frame')
    ax.set(xlabel='X (cm)',ylabel='Y (cm)',zlabel='Z (cm)',title='Geometric path in model base frame')
    ax.legend(fontsize=7)
    side.set(xlabel='Distance along each segment (cm)',ylabel='Z (cm)',title='Approach + horizontal stroke')
    side.legend(fontsize=7);side.grid(alpha=.25)
    fig.suptitle('Nominal target path — waypoint interpolation can curve between targets',fontsize=11)
    fig.tight_layout();fig.savefig(path,dpi=150);plt.close(fig)


def save_execution_preview(plan, result, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    actual=np.array([s['actual_xyz_cm'] for s in result['samples']])
    goals=np.array(plan['drawing_points_cm'])
    arrivals=np.array([s['actual_xyz_cm'] for s in result['waypoint_results']])
    fig=plt.figure(figsize=(11,4))
    ax=fig.add_subplot(121,projection='3d')
    ax.plot(*actual.T,color='#16a34a',label='Feedback-derived end-frame path')
    ax.plot(*goals.T,'o--',color='#2563eb',label='Three requested waypoints')
    if len(arrivals):ax.scatter(*arrivals.T,color='#db2777',s=45,label='Accepted real-feedback arrivals')
    ax.set(xlabel='X (cm)',ylabel='Y (cm)',zlabel='Z (cm)',title=f'Execution: {result["status"]}')
    ax.legend(fontsize=7)
    side=fig.add_subplot(122)
    side.plot([s['time_s'] for s in result['samples']],actual[:,2],color='#16a34a',label='Feedback-derived Z')
    side.axhline(plan['line_z_cm'],linestyle='--',color='#2563eb',label='Requested plane')
    side.set(xlabel='Time (s)',ylabel='Z (cm)',title='Height during approach and coarse motion')
    side.legend(fontsize=8);side.grid(alpha=.25)
    fig.suptitle('REAL joint feedback + modeled TCP; tool tip not measured' if plan.get('tcp_offset_cm') is not None
                 else 'REAL joint feedback + URDF end frame; tool tip not measured',fontsize=11)
    fig.tight_layout();fig.savefig(path,dpi=150);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://127.0.0.1:80')
    parser.add_argument('--robot-id',type=int,default=0)
    tool=parser.add_mutually_exclusive_group()
    tool.add_argument('--tcp-offset-cm',nargs=3,type=finite,metavar=('X','Y','Z'),
                        help='启用 Position-only TCP IK：棍尖在 Fixed_Jaw 局部坐标系中的偏移，cm；不约束朝向')
    tool.add_argument('--rod-length-cm',type=finite,
                      help='启用 TCP IK：从 Fixed_Jaw 原点沿夹爪局部 -Y 向前延伸的长度，cm')
    parser.add_argument('--z',type=finite,default=15,help='模型底座参考系 Z，cm')
    parser.add_argument('--length',type=finite,default=10,help='线长 cm')
    parser.add_argument('--axis',choices=['x','y'],default='x')
    parser.add_argument('--direction',type=int,choices=[-1,1],default=-1)
    sampling=parser.add_mutually_exclusive_group()
    sampling.add_argument('--points',type=int,default=3,help='绘图目标点数量，默认起点/中点/终点三个')
    sampling.add_argument('--spacing',type=finite,help='改为密采样，相邻点最大距离 cm')
    sampling.add_argument('--insert-points',type=int,help='在起终点之间均匀插入几个点，例如 5 对应共 7 个点')
    parser.add_argument('--lift-cm',type=finite,default=0,help='线完成后在终点上抬的高度 cm，默认不抬')
    parser.add_argument('--joint-tolerance',type=finite,default=2,help='到位关节误差，度，默认 2')
    parser.add_argument('--xyz-tolerance-mm',type=finite,default=10,help='到位模型 TCP/末端误差，mm，默认 10')
    parser.add_argument('--paper-z-cm',type=finite,help='绘图 Z 保护的纸面高度，cm，默认使用绘图路径 Z')
    parser.add_argument('--z-below-tolerance-mm',type=finite,default=5,
                        help='TCP 绘图段低于纸面的允许量，mm，默认 5；超过后停止补偿')
    parser.add_argument('--point-timeout',type=finite,default=60,help='每个目标点补偿超时秒数')
    parser.add_argument('--joint-step-deg',type=finite,help='统一覆盖五轴每轮最大补偿角度，例如 0.5；默认肩/肘 1，其余 0.5')
    parser.add_argument('--replay',type=Path,help='重走已有 planning.json 中的三个目标位置')
    parser.add_argument('--return-home',action='store_true',help='路径前增加五个运动关节回零，夹爪不动')
    parser.add_argument('--output',type=Path,default=ROOT/'垃圾文件夹/plans')
    parser.add_argument('--solve',action='store_true',help='离线求 IK 并检查原始模型自碰撞/限位')
    parser.add_argument('--execute',action='store_true',help='完整求解预检通过后执行，并检查真实反馈')
    args=parser.parse_args()
    if args.rod_length_cm is not None:
        if args.rod_length_cm <= 0:
            parser.error('rod-length-cm 必须大于零')
        args.tcp_offset_cm=[0.,-args.rod_length_cm,0.]
    if args.robot_id < 0 or args.length <= 0 or (args.spacing is not None and args.spacing <= 0):
        parser.error('robot-id 必须非负，length 和 spacing 必须大于零')
    if not 2 <= args.points <= 1000:
        parser.error('points 范围 2..1000')
    if args.insert_points is not None and not 0 <= args.insert_points <= 998:
        parser.error('insert-points 范围 0..998')
    if not 0 <= args.lift_cm <= 10:
        parser.error('lift-cm 范围 0..10 cm')
    if not 0 <= args.z_below_tolerance_mm <= 5:
        parser.error('z-below-tolerance-mm 范围 0..5 mm')
    if args.joint_step_deg is not None and not 0<args.joint_step_deg<=1:
        parser.error('joint-step-deg 范围大于 0 且不超过 1°')
    if not .1 <= args.joint_tolerance <= 5 or not 1 <= args.xyz_tolerance_mm <= 20 or not 1 <= args.point_timeout <= 120:
        parser.error('joint-tolerance 范围 0.1..5°，xyz-tolerance-mm 范围 1..20 mm，point-timeout 范围 1..120 秒')
    client=Client(args.url,args.robot_id)
    try:
        config=client.precheck()
        api=client.call('/openapi.json',get=True)
        paths=api['paths']
        if 'post' not in paths.get('/move/relative',{}):
            raise RuntimeError('服务不提供原生 /move/relative')
        angles=FeedbackClient(client,config).read()
    finally:
        client.close()
    tcp_model=Model(args.tcp_offset_cm) if args.tcp_offset_cm is not None else None
    current=(tcp_model.position(tcp_model.np.deg2rad(angles[:5]))*100).tolist() if tcp_model is not None else forward_xyz(angles)
    plan=build_plan(current,args.z,args.length,args.axis,args.direction,
                    args.spacing if args.spacing is not None else .2,
                    None if args.spacing is not None else args.points)
    if args.replay:
        source=json.loads(args.replay.read_text())
        points=source['drawing_points_cm']
        if (source.get('unit')!='cm' or len(points)<2
                or any(len(p)!=3 or any(not math.isfinite(v) for v in p) for p in points)):
            raise ValueError('重放路径坐标无效')
        plan.update({'drawing_points_cm':points,'drawing_point_count':len(points),
                     'line_start_cm':points[0],'line_end_cm':points[-1],
                     'line_length_cm':math.dist(points[0],points[-1]),
                     'line_z_cm':source['line_z_cm'],
                     'approach_points_cm':[current,points[0]],
                     'replay_source':str(args.replay.resolve()),
                     'phosphobot_requests':[]})
        prior_log=args.replay.parent/'execution.json'
        if prior_log.exists():
            previous=json.loads(prior_log.read_text())
            if (previous.get('status')=='completed' and previous.get('commands') and previous.get('last_actual_angles')
                    and max(abs(a-b) for a,b in zip(angles[:5],previous['last_actual_angles'][:5]))<=3):
                plan['initial_command_angles_deg']=previous['commands'][-1]['angles_deg']
                plan['initial_command_feedback_angles_deg']=previous['last_actual_angles'][:5]
                plan['initial_command_source']=str(prior_log.resolve())
    if args.insert_points is not None:
        insert_line_points(plan,args.insert_points)
    elif args.replay and args.spacing is not None:
        resample_replay_points(plan,args.spacing)
    add_final_lift(plan,args.lift_cm)
    plan['return_home']=args.return_home
    if args.return_home:
        home_xyz=(tcp_model.position(tcp_model.np.zeros(5))*100).tolist() if tcp_model is not None else forward_xyz([0.,0.,0.,0.,0.,angles[5]])
        plan['approach_points_cm']=[current,home_xyz,plan['drawing_points_cm'][0]]
    plan['recorded_at']=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat()
    plan['source_joint_angles_deg']=angles
    plan['tcp_offset_cm']=args.tcp_offset_cm
    if tcp_model is not None:
        plan['drawing_z_protection']={'scope':'draw phase only',
            'paper_z_cm':plan['line_z_cm'] if args.paper_z_cm is None else args.paper_z_cm,
            'z_below_tolerance_mm':args.z_below_tolerance_mm}
    if tcp_model is not None:
        plan['reference_point']='Rod tip TCP attached to Fixed_Jaw; position-only, free orientation'
        plan['tcp_offset_frame']='Fixed_Jaw local frame; offset in cm'
    plan['arrival_mode']='TCP XYZ only; home also checks joint angles' if tcp_model is not None else 'joint angles and end-frame XYZ'
    plan['robot_id']=args.robot_id
    plan['service_url']=args.url
    plan['phosphobot_version']=api.get('info',{}).get('version')
    plan['planning_method']='Local Cartesian interpolation; no standalone planning/IK endpoint available in the inspected service'
    plan['feedback_source']='Individual motor positions via /joints/read motor_units, source=robot; converted with current Phosphobot calibration'
    plan['execution_settings']={'joint_tolerance_deg':args.joint_tolerance,
                                'xyz_tolerance_mm':args.xyz_tolerance_mm,
                                'point_timeout_s':args.point_timeout,
                                'control_step_deg':.5,'feedback_interval_s':.25}
    plan['execution_settings']['control_step_deg_by_joint']=[.5,1.,1.,.5,.5]
    if args.joint_step_deg is not None:
        plan['execution_settings']['control_step_deg']=args.joint_step_deg
        plan['execution_settings']['control_step_deg_by_joint']=[args.joint_step_deg]*5
    if tcp_model is not None:
        plan['execution_settings']['drawing_compensation']='One changed joint target per round; pending-joint FK height screening; lift priority within 2 mm of paper'
    for request in plan['phosphobot_requests']:
        request['query']={'robot_id':args.robot_id}
    run_dir=args.output/f'line-{datetime.now(ZoneInfo("Asia/Shanghai")):%Y%m%d-%H%M%S-%f}'
    run_dir.mkdir(parents=True,exist_ok=False)
    destination=run_dir/'planning.json'
    destination.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n')
    model=None
    try:
        model=solve_plan(plan,tcp_model) if args.solve or args.execute else None
    except Exception as exc:
        plan['precheck_error']=str(exc)
        destination.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n')
        raise
    destination.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n')
    save_preview(plan,run_dir/'planning.png')
    print('当前棍尖 TCP XYZ（cm）：' if tcp_model is not None else '当前模型末端 XYZ（cm）：',[round(v,3) for v in current])
    if tcp_model is not None:
        print('TCP 偏移（Fixed_Jaw 局部坐标，cm）：',args.tcp_offset_cm)
        protection=plan['drawing_z_protection']
        print(f'绘图 Z 保护：低于纸面 Z={protection["paper_z_cm"]:g} cm '
              f'超过 {protection["z_below_tolerance_mm"]:g} mm 就停止补偿。')
    print('直线起点（cm）：',[round(v,3) for v in plan['line_start_cm']])
    print('直线终点（cm）：',[round(v,3) for v in plan['line_end_cm']])
    print(f'线长 {plan["line_length_cm"]:.3f} cm，目标 Z={plan["line_z_cm"]:g} cm，绘图点 {len(plan["drawing_points_cm"])} 个')
    if plan['lift_points_cm']:
        print('线完成后抬笔目标（模型末端 cm）：',plan['lift_points_cm'][-1])
    print('路径及原生 API 请求清单：',destination)
    print('预览图：',run_dir/'planning.png')
    if tcp_model is not None:
        print(f'到位条件：棍尖 TCP XYZ误差≤{args.xyz_tolerance_mm:g} mm，连续两次满足；不约束朝向。回零另检查关节误差≤{args.joint_tolerance:g}°。')
    else:
        print(f'到位条件：关节误差≤{args.joint_tolerance:g}°，XYZ误差≤{args.xyz_tolerance_mm:g} mm，连续两次满足。')
    if model is None:
        print('仅规划，未调用任何移动接口；无独立规划 API，尚未验证 IK、关节限位或碰撞。')
    else:
        print(f'独立 IK 及模型预检通过：{len(plan["solutions"])} 个点，最大误差 {max(p["ik_error_mm"] for p in plan["solutions"]):.6f} mm。')
    if args.execute:
        client=Client(args.url,args.robot_id)
        try:
            current_config=client.precheck()
            if current_config != config:
                raise RuntimeError('规划后标定配置发生变化，未执行')
            execute_plan(FeedbackClient(client,current_config),model,plan,config,destination,args.joint_tolerance,
                         args.xyz_tolerance_mm,args.point_timeout)
        finally:
            client.close()


if __name__=='__main__':
    main()
