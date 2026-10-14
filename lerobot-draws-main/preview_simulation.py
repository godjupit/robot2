"""Offline URDF kinematic preview. Never opens serial or sends HTTP requests."""
import argparse
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    args = parser.parse_args()
    out = ROOT / 'outputs/triangle/simulation'
    out.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(args.urdf, out / 'so-100.urdf')
    source = ET.parse(args.urdf).getroot()
    chain = []
    for joint in source.findall('joint')[:5]:
        origin = joint.find('origin')
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler('xyz',
            np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        transform[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
        axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
        limits = joint.find('limit')
        chain.append((transform, axis, float(limits.get('lower')), float(limits.get('upper'))))

    def fk(q):
        transform = np.eye(4)
        # Same base rotation as SO100Hardware.AXIS_ORIENTATION=[0,0,1,1].
        transform[:3, :3] = Rotation.from_quat([0, 0, 1, 1]).as_matrix()
        nodes = [transform[:3, 3].copy()]
        for (origin, axis, _, _), angle in zip(chain, q):
            transform = transform @ origin
            nodes.append(transform[:3, 3].copy())
            motion = np.eye(4)
            motion[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            transform = transform @ motion
        return np.array(nodes), transform[:3, :3]

    initial_nodes, initial_rotation = fk(np.zeros(5))
    initial = initial_nodes[-1]
    commands = json.loads((ROOT / 'outputs/triangle/http_commands.json').read_text())
    frames = [{'target': initial, 'nodes': initial_nodes, 'kind': 'Initialize', 'q': np.zeros(5), 'error': 0.}]
    q = np.zeros(5)
    lower = np.array([j[2] for j in chain])
    upper = np.array([j[3] for j in chain])
    previous = initial
    for index, command in enumerate(commands[1:]):
        point = command['data']
        target = initial + np.array([point['x'], point['y'], point['z']]) / 100
        kind = 'Draw' if point['z'] == 2 else ('Return' if index == len(commands)-2 else 'Pen up / travel')
        count = max(4, int(np.ceil(np.linalg.norm(target-previous)/.006)))
        # Smooth Cartesian preview, not a replay of measured motor feedback.
        for fraction in np.linspace(0, 1, count+1)[1:]:
            goal = previous + fraction*(target-previous)
            fixed_orientation = all(k in point for k in ('rx', 'ry', 'rz'))
            seed = q.copy()
            def residual(angles):
                nodes, rotation = fk(angles)
                orientation = Rotation.from_matrix(initial_rotation.T @ rotation).as_rotvec()
                return np.r_[(nodes[-1]-goal)*1000,
                             orientation*20 if fixed_orientation else orientation*0,
                             (angles-seed)*.05]
            solution = least_squares(residual, np.clip(q, lower+1e-8, upper-1e-8),
                                     bounds=(lower, upper), max_nfev=100)
            q = solution.x
            nodes, _ = fk(q)
            frames.append({'target': goal, 'nodes': nodes, 'kind': kind,
                           'q': q.copy(), 'error': float(np.linalg.norm(nodes[-1]-goal)*1000)})
        previous = target
    targets = np.array([f['target'] for f in frames])*100
    actual = np.array([f['nodes'][-1] for f in frames])*100
    draw_ids = [i for i, f in enumerate(frames) if f['kind']=='Draw']
    draw_targets = targets[draw_ids]
    plane_z = float(initial[2]*100+2)
    report = {'preview_only': True, 'motion_sent': False,
              'solver': 'Independent SciPy IK using the Phospho SO100 URDF; not the Phospho PyBullet solver',
              'initial_model_position_cm': (initial*100).tolist(),
              'drawing_model_z_cm': plane_z, 'frames': len(frames),
              'max_model_position_error_mm': max(f['error'] for f in frames),
              'pen_tip_measured': False, 'collision_checked': False}
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    (out/'frames.json').write_text(json.dumps([
        {k: v.tolist() if isinstance(v, np.ndarray) else v for k,v in f.items()} for f in frames]))

    fig = plt.figure(figsize=(12, 5), facecolor='#f6f8fa')
    ax = fig.add_subplot(121, projection='3d')
    top = fig.add_subplot(222)
    side = fig.add_subplot(224)
    ax.set(xlabel='X (cm)', ylabel='Y (cm)', zlabel='Z (cm)', title='SO100 model + requested drawing')
    bounds = np.concatenate([targets, *(f['nodes']*100 for f in frames)])
    center = (bounds.min(axis=0)+bounds.max(axis=0))/2
    radius = max(np.ptp(bounds, axis=0))/2+3
    ax.set_xlim(center[0]-radius,center[0]+radius)
    ax.set_ylim(center[1]-radius,center[1]+radius)
    ax.set_zlim(min(-1,center[2]-radius),center[2]+radius)
    ax.set_box_aspect((1,1,1))
    ax.view_init(elev=24, azim=-55)
    arm, = ax.plot([],[],[], 'o-', color='#2463eb', lw=6, ms=5, label='URDF arm')
    goal, = ax.plot([],[],[], 'x', color='#db2777', ms=9, label='Requested end frame')
    trace, = ax.plot([],[],[], color='#16a34a', lw=2, label='Requested stroke')
    ax.plot(draw_targets[:,0],draw_targets[:,1],draw_targets[:,2], color='#16a34a', alpha=.25)
    ax.legend(loc='upper left',fontsize=8)
    top.plot(draw_targets[:,0],draw_targets[:,1], color='#16a34a', lw=2)
    top.set(xlabel='X (cm)',ylabel='Y (cm)',title='Top view: commanded triangle')
    top.set_aspect('equal',adjustable='datalim')
    top.grid(alpha=.2)
    top_dot, = top.plot([],[], 'o',color='#db2777')
    side.plot(targets[:,2],color='#db2777',label='Requested Z')
    side.plot(actual[:,2],color='#2463eb',label='Independent IK model Z')
    side.set(xlabel='Preview frame',ylabel='Z (cm)',title='Height: travel 5 cm / draw 2 cm relative to init')
    cursor=side.axvline(0,color='black',alpha=.5)
    side.legend(fontsize=7)
    label=fig.text(.04,.02,'',fontsize=9)
    fig.suptitle('OFFLINE KINEMATIC PREVIEW — no measured pen tip or motor feedback',fontsize=12)
    fig.tight_layout(rect=(0,.05,1,.93))
    def update(i):
        f=frames[i]; nodes=f['nodes']*100; target=targets[i]
        arm.set_data_3d(nodes[:,0],nodes[:,1],nodes[:,2])
        goal.set_data_3d([target[0]],[target[1]],[target[2]])
        ids=[j for j in draw_ids if j<=i]
        drawn=targets[ids]
        trace.set_data_3d(drawn[:,0],drawn[:,1],drawn[:,2])
        top_dot.set_data([target[0]],[target[1]])
        cursor.set_xdata([i,i])
        label.set_text(f'{i+1}/{len(frames)}  {f["kind"]}   Requested XYZ: {target.round(2)} cm   Model error: {f["error"]:.1f} mm')
        return arm,goal,trace,top_dot,cursor,label
    update(len(frames)-1)
    fig.savefig(out/'overview.png',dpi=140)
    animation=FuncAnimation(fig,update,frames=len(frames),interval=90,blit=False)
    animation.save(out/'triangle.gif',writer=PillowWriter(fps=12),dpi=90)
    plt.close(fig)
    (out/'index.html').write_text('''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>三角形离线运动预览</title>
<style>body{font:16px system-ui;max-width:1200px;margin:30px auto;padding:0 20px;background:#f6f8fa}img{width:100%;border-radius:12px}p{line-height:1.7}</style>
<h1>10 cm 三角形：离线运动预览</h1><p>蓝色：机械臂模型。粉色叉号：指令目标。绿色：绘图路径。右上是俯视图，右下是高度变化。</p>
<img src="triangle.gif"><p>使用 Phospho SO100 的 URDF 和当前绘图指令，独立求解 IK 并平滑插值。此动画不是原 PyBullet 求解器的逐帧回放，也不是实测真机运动；末端参考点不等于未经测量的笔尖，粉色目标与蓝色模型可能有误差。</p>
<p><a href="overview.png">查看静态图</a> · <a href="report.json">查看模型误差报告</a></p></html>''')
    print(json.dumps(report,indent=2))
    print(out/'index.html')


if __name__ == '__main__':
    main()
