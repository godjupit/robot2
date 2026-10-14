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
    def __init__(self):
        import numpy as np
        import trimesh
        from scipy.spatial.transform import Rotation
        from trimesh.collision import CollisionManager
        self.np, self.rotation = np, Rotation
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
        return self.transforms(q,0)['Fixed_Jaw'][:3,3]

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


def solve_plan(plan):
    import numpy as np
    from scipy.optimize import least_squares
    model = Model()
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
    targets += [('lift',p) for p in plan.get('lift_points_cm',[])]
    for index,(phase,point) in enumerate(targets):
        target = np.asarray(point)/100
        seed = q.copy()
        result = least_squares(lambda angles: np.r_[(model.position(angles)-target)*1000,
                                                   (angles-seed)*.001],
                               q,bounds=(model.lower,model.upper),max_nfev=200)
        q = result.x
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
    plan.update({'ik_validated':True,'collision_checked':True,'joint_limits_checked':True,
                 'solver':'Independent SciPy position-only IK using Phosphobot SO100 URDF, not its PyBullet solver',
                 'collision_scope':'Original SO100 meshes, nonadjacent self collisions and distal clearance above model Z=0. No external objects or rod/TCP modeled, per requested scope.',
                 'orientation':'Free orientation, regularized toward preceding joint pose',
                 'solutions':solutions})
    return model


def move_point_with_compensation(client, model, point, command, write_arm, record,
                                 deadline, joint_tolerance=2., xyz_tolerance_mm=10.,
                                 point_timeout=60., max_compensation=15., step_deg=.5,
                                 interval=.25):
    """每个目标点独立闭环；固定期望角度，每次最多修正 0.5° 的发送角度。"""
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
        record(sample)
        arrived = max(abs(error)) <= joint_tolerance and position_error_mm <= xyz_tolerance_mm
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
        proposed = command+np.where(mask,np.clip(error,-step_deg,step_deg),0.)
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
                 xyz_tolerance_mm=10., point_timeout=60.):
    """按粗采样目标逐点反馈补偿，记录规划/发送/实测；不发送夹爪目标。"""
    import csv
    np = model.np
    initial = client.read()
    seed = plan['source_joint_angles_deg']
    if max(abs(a-b) for a,b in zip(initial,seed)) > .5:
        raise RuntimeError('真机姿态已偏离规划起点，未发送运动')
    result = {'motion_sent':False,'status':'running','samples':[],'commands':[],
              'completed_points':0,'waypoint_results':[],
              'line_completed':False,'lift_completed':False,
              'planning_file':str(destination.resolve()),
              'arrival_joint_tolerance_deg':joint_tolerance,
              'arrival_xyz_tolerance_mm':xyz_tolerance_mm,
              'point_timeout_s':point_timeout,'control_step_deg':.5,
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
        result['commands'].append({'time_s':time.monotonic()-started,'angles_deg':values.tolist()})
    try:
        for index,point in enumerate(plan['solutions']):
            print(f'目标 {index+1}/{len(plan["solutions"])}: XYZ={np.round(point["xyz_cm"],3).tolist()} cm；'
                  f'规划角度={np.round(point["joint_angles_deg"],2).tolist()}°',flush=True)
            def record(sample):
                row={'time_s':time.monotonic()-started,'point':index+1,'phase':point['phase'],**sample}
                result['samples'].append(row)
                result['last_actual_angles']=sample['actual_angles_deg'].copy()
                stream.write(json.dumps(row,ensure_ascii=False)+'\n');stream.flush()
            command,actual = move_point_with_compensation(
                client,model,point,command,write_arm,record,started+360,
                joint_tolerance,xyz_tolerance_mm,point_timeout)
            result['completed_points']=index+1
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
    steps = max(1, math.ceil(distance/spacing_cm))
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
    fig.suptitle('REAL joint feedback + URDF end frame; tool tip not measured',fontsize=11)
    fig.tight_layout();fig.savefig(path,dpi=150);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://127.0.0.1:80')
    parser.add_argument('--robot-id',type=int,default=0)
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
    parser.add_argument('--xyz-tolerance-mm',type=finite,default=10,help='到位模型末端误差，mm，默认 10')
    parser.add_argument('--point-timeout',type=finite,default=60,help='每个目标点补偿超时秒数')
    parser.add_argument('--replay',type=Path,help='重走已有 planning.json 中的三个目标位置')
    parser.add_argument('--return-home',action='store_true',help='路径前增加五个运动关节回零，夹爪不动')
    parser.add_argument('--output',type=Path,default=ROOT/'垃圾文件夹/plans')
    parser.add_argument('--solve',action='store_true',help='离线求 IK 并检查原始模型自碰撞/限位')
    parser.add_argument('--execute',action='store_true',help='完整求解预检通过后执行，并检查真实反馈')
    args=parser.parse_args()
    if args.robot_id < 0 or args.length <= 0 or (args.spacing is not None and args.spacing <= 0):
        parser.error('robot-id 必须非负，length 和 spacing 必须大于零')
    if not 2 <= args.points <= 1000:
        parser.error('points 范围 2..1000')
    if args.insert_points is not None and not 0 <= args.insert_points <= 998:
        parser.error('insert-points 范围 0..998')
    if not 0 <= args.lift_cm <= 10:
        parser.error('lift-cm 范围 0..10 cm')
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
    current=forward_xyz(angles)
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
    add_final_lift(plan,args.lift_cm)
    plan['return_home']=args.return_home
    if args.return_home:
        home_xyz=forward_xyz([0.,0.,0.,0.,0.,angles[5]])
        plan['approach_points_cm']=[current,home_xyz,plan['drawing_points_cm'][0]]
    plan['recorded_at']=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat()
    plan['source_joint_angles_deg']=angles
    plan['robot_id']=args.robot_id
    plan['service_url']=args.url
    plan['phosphobot_version']=api.get('info',{}).get('version')
    plan['planning_method']='Local Cartesian interpolation; no standalone planning/IK endpoint available in the inspected service'
    plan['feedback_source']='Individual motor positions via /joints/read motor_units, source=robot; converted with current Phosphobot calibration'
    plan['execution_settings']={'joint_tolerance_deg':args.joint_tolerance,
                                'xyz_tolerance_mm':args.xyz_tolerance_mm,
                                'point_timeout_s':args.point_timeout,
                                'control_step_deg':.5,'feedback_interval_s':.25}
    for request in plan['phosphobot_requests']:
        request['query']={'robot_id':args.robot_id}
    run_dir=args.output/f'line-{datetime.now(ZoneInfo("Asia/Shanghai")):%Y%m%d-%H%M%S-%f}'
    run_dir.mkdir(parents=True,exist_ok=False)
    destination=run_dir/'planning.json'
    destination.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n')
    model=None
    try:
        model=solve_plan(plan) if args.solve or args.execute else None
    except Exception as exc:
        plan['precheck_error']=str(exc)
        destination.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n')
        raise
    destination.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n')
    save_preview(plan,run_dir/'planning.png')
    print('当前模型末端 XYZ（cm）：',[round(v,3) for v in current])
    print('直线起点（cm）：',[round(v,3) for v in plan['line_start_cm']])
    print('直线终点（cm）：',[round(v,3) for v in plan['line_end_cm']])
    print(f'线长 {plan["line_length_cm"]:.3f} cm，目标 Z={plan["line_z_cm"]:g} cm，绘图点 {len(plan["drawing_points_cm"])} 个')
    if plan['lift_points_cm']:
        print('线完成后抬笔目标（模型末端 cm）：',plan['lift_points_cm'][-1])
    print('路径及原生 API 请求清单：',destination)
    print('预览图：',run_dir/'planning.png')
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
