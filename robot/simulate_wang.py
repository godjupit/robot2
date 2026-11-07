#!/usr/bin/env python3
"""Offline MuJoCo kinematic replay of saved Wang IK and measured joint logs."""
import argparse
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent
DEFAULT = ROOT / '垃圾文件夹/plans/wang-20261014-221245-412748/planning.json'


def numbers(x):
    return ' '.join(str(float(v)) for v in x)


def origin(element):
    if element is None:
        return dict(pos='0 0 0', quat='1 0 0 0')
    q = Rotation.from_euler('xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_quat()
    return dict(pos=element.get('xyz', '0 0 0'), quat=numbers(q[[3, 0, 1, 2]]))


def make_model(plan, output):
    urdf = ET.parse(ROOT / 'models/so-100.urdf').getroot()
    root = ET.Element('mujoco', model='SO100 Wang offline replay')
    ET.SubElement(root, 'compiler', angle='radian', autolimits='true')
    ET.SubElement(root, 'option', gravity='0 0 0')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='960', offheight='720')
    asset = ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='0.2 -0.5 1', dir='0 0 -1', diffuse='0.9 0.9 0.9')
    ET.SubElement(world, 'geom', type='plane', size='1 1 .01', pos='0 0 -.008', rgba='.16 .19 .24 1', contype='0', conaffinity='0')
    strokes = np.array(plan['strokes_cm']) / 100
    center = strokes.reshape(-1, 3).mean(axis=0)
    ET.SubElement(world, 'geom', name='paper', type='box', pos=numbers(center - [0, 0, .003]), size='.13 .12 .001', rgba='.95 .95 .91 1', contype='0', conaffinity='0')
    for i, (a, b) in enumerate(strokes):
        ET.SubElement(world, 'geom', name=f'target_{i}', type='capsule', fromto=numbers(np.r_[a, b]), size='.0007', rgba='.12 .45 .95 1', contype='0', conaffinity='0')
    links = {link.get('name'): link for link in urdf.findall('link')}
    children = {}
    joints = urdf.findall('joint')
    for joint in joints:
        children.setdefault(joint.find('parent').get('link'), []).append(joint)
    def add_link(name, body):
        for i, geom in enumerate(links[name].findall('visual')):
            mesh = geom.find('geometry/mesh')
            mesh_name = f'{name}_{i}'
            path = ROOT / 'models' / mesh.get('filename').replace('package://', '')
            ET.SubElement(asset, 'mesh', name=mesh_name, file=str(path), scale=mesh.get('scale', '1 1 1'))
            ET.SubElement(body, 'geom', type='mesh', mesh=mesh_name, rgba='.65 .72 .82 1', contype='0', conaffinity='0', **origin(geom.find('origin')))
        if name == 'Fixed_Jaw':
            offset = np.array(plan['tcp_offset_cm']) / 100
            ET.SubElement(body, 'geom', name='pen', type='capsule', fromto=numbers(np.r_[np.zeros(3), offset]), size='.002', rgba='.95 .55 .1 1', contype='0', conaffinity='0')
            ET.SubElement(body, 'site', name='tcp', pos=numbers(offset), size='.003', rgba='1 .15 .1 1')
        for joint in children.get(name, []):
            child = joint.find('child').get('link')
            sub = ET.SubElement(body, 'body', name=child, **origin(joint.find('origin')))
            limit = joint.find('limit')
            ET.SubElement(sub, 'joint', name=joint.get('name'), type='hinge', axis=joint.find('axis').get('xyz'), range=f"{limit.get('lower')} {limit.get('upper')}")
            add_link(child, sub)
    base = ET.SubElement(world, 'body', name='Base', quat=numbers([2**-.5, 0, 0, 2**-.5]))
    add_link('Base', base)
    output.write_text(ET.tostring(root, encoding='unicode'))
    model = mujoco.MjModel.from_xml_path(str(output))
    addresses = [model.joint(j.get('name')).qposadr[0] for j in joints]
    return model, addresses


def set_pose(model, data, addresses, q):
    data.qpos[addresses] = np.deg2rad(q)
    mujoco.mj_forward(model, data)
    return data.site('tcp').xpos.copy()


def ideal_frames(plan):
    q = np.array(plan['source_joint_angles_deg'], float)
    frames = []
    for index, s in enumerate(plan['solutions']):
        goal = np.r_[s['joint_angles_deg'], q[5]]
        n = max(3, int(np.ceil(np.max(abs(goal-q)) / .35)))
        for f in np.linspace(0, 1, n+1)[1:]:
            frames.append(dict(q=q+(goal-q)*f, phase=s['phase'], point=index+1))
        q = goal
    return frames


def trail(scene, xyz, phases, color):
    for i in range(1, len(xyz)):
        if phases[i] != 'draw' or phases[i-1] != 'draw':
            continue
        if np.linalg.norm(xyz[i]-xyz[i-1]) < 1e-9:
            continue
        if scene.ngeom >= scene.maxgeom:
            break
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.array(color, np.float32))
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, .0009, xyz[i-1], xyz[i])
        scene.ngeom += 1


def camera():
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [.25, 0, .12]
    cam.distance = .82
    cam.azimuth = 135
    cam.elevation = -35
    return cam


def export_video(model, data, addresses, frames, xyz, output, label):
    import imageio.v2 as imageio
    from PIL import Image, ImageDraw
    phases = [f['phase'] for f in frames]
    cam = camera()
    # Bound movie size; every original sample is retained in numerical analysis.
    indices = np.unique(np.linspace(0, len(frames)-1, min(len(frames), 720)).astype(int))
    with mujoco.Renderer(model, height=720, width=960, max_geom=12000) as renderer:
        with imageio.get_writer(str(output), fps=24, codec='libx264') as writer:
            for index in indices:
                set_pose(model, data, addresses, frames[index]['q'])
                renderer.update_scene(data, camera=cam)
                trail(renderer.scene, xyz[:index+1], phases[:index+1], [1, .18, .05, 1])
                image = Image.fromarray(renderer.render())
                draw = ImageDraw.Draw(image)
                draw.rectangle((0, 0, 960, 65), fill=(20, 25, 32))
                draw.text((16, 12), f"{label} | phase: {phases[index]} | target: {frames[index]['point']}", fill='white')
                draw.text((16, 36), 'BLUE: target   RED: TCP path   ORANGE: pen | Kinematic replay, NOT contact dynamics; time compressed', fill='white')
                writer.append_data(np.array(image))
                if index == indices[-1]:
                    image.save(output.with_suffix('.png'))


def analyze(plan, model, data, addresses, ideal, feedback, output, plan_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from plan import Model
    original = Model(plan['tcp_offset_cm'])
    validation = []
    for frame in ideal[::max(1, len(ideal)//80)] + feedback[::max(1, len(feedback)//80)]:
        a = set_pose(model, data, addresses, frame['q'])
        b = original.position(np.deg2rad(frame['q'][:5]))
        validation.append(np.linalg.norm(a-b)*1000)
    if max(validation) > 1e-5:
        raise RuntimeError(f'MuJoCo/solver FK mismatch: {max(validation)} mm')
    trajectories = []
    for frames in (ideal, feedback):
        trajectories.append(np.array([set_pose(model, data, addresses, f['q']) for f in frames]))
    stats = dict(source_plan=str(plan_path.resolve()), replay_type='kinematic; no servo/contact dynamics', fk_crosscheck_max_mm=max(validation))
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    for a, b in np.array(plan['strokes_cm']):
        axes[0].plot([a[1]*10, b[1]*10], [a[0]*10, b[0]*10], color='royalblue', lw=3, alpha=.5)
    for frames, xyz, label, color in zip((ideal, feedback), trajectories, ('Ideal IK', 'Recorded joints FK'), ('green', 'orangered')):
        drawing = np.array([f['phase']=='draw' for f in frames])
        plotted = xyz.copy()*1000
        if len(plotted):
            plotted[~drawing] = np.nan
            axes[0].plot(plotted[:, 1], plotted[:, 0], color=color, label=label, lw=1.2)
        if not drawing.any():
            continue
        errors, zs = [], []
        for f, p in zip(frames, xyz):
            if f['phase'] != 'draw':
                continue
            s = plan['solutions'][f['point']-1]
            sid = s.get('stroke_id', 1)
            ids = plan.get('stroke_ids', list(range(1, len(plan['strokes_cm'])+1)))
            a, b = np.array(plan['strokes_cm'][ids.index(sid)])/100
            t = np.clip(np.dot(p-a, b-a)/np.dot(b-a, b-a), 0, 1)
            errors.append(np.linalg.norm(p-(a+t*(b-a)))*1000)
            zs.append((p[2]-a[2])*1000)
        stats[label] = dict(draw_samples=len(errors), line_distance_rms_mm=float(np.sqrt(np.mean(np.square(errors)))), line_distance_max_mm=float(max(errors)), z_min_mm=float(min(zs)), z_max_mm=float(max(zs)))
        axes[1].plot(errors, color=color, label=label)
        axes[2].plot(zs, color=color, label=label)
    axes[0].set(xlabel='Model Y (mm)', ylabel='Model X (mm)', title='Pen-down path (not optical measurement)', aspect='equal')
    axes[1].set(xlabel='Draw sample (different playback rates)', ylabel='3D distance to stroke (mm)', title='Distance to intended line')
    axes[2].set(xlabel='Draw sample', ylabel='TCP height above paper (mm)', title='Paper height error')
    for ax in axes:
        ax.grid(alpha=.25); ax.legend()
    fig.tight_layout(); fig.savefig(output/'comparison.png', dpi=160); plt.close(fig)
    execution = plan_path.with_name('execution.json')
    if execution.exists():
        d = json.loads(execution.read_text())
        stats['recorded_execution'] = {k:d.get(k) for k in ['status','reason','completed_points','completed_strokes','arrival_xyz_tolerance_mm']}
    (output/'metrics.json').write_text(json.dumps(stats, ensure_ascii=False, indent=2)+'\n')
    np.savez_compressed(output/'trajectories.npz', ideal_xyz_m=trajectories[0], feedback_xyz_m=trajectories[1])
    print(json.dumps(stats, ensure_ascii=False, indent=2), flush=True)
    return trajectories


def main(args):
    plan = json.loads(args.plan.read_text())
    if not plan.get('solutions') or not plan.get('tcp_offset_cm') or not plan.get('strokes_cm'):
        raise ValueError('Need a solved Wang planning.json with TCP offset and strokes')
    args.output.mkdir(parents=True, exist_ok=True)
    model, addresses = make_model(plan, args.output/'scene.xml')
    data = mujoco.MjData(model)
    ideal = ideal_frames(plan)
    file = args.plan.with_name('actual_feedback.jsonl')
    rows = [json.loads(line) for line in file.read_text().splitlines() if line.strip()] if file.exists() else []
    feedback = [dict(q=np.array(r['actual_angles_deg']), phase=r['phase'], point=r['point'], time_s=r['time_s']) for r in rows]
    trajectories = analyze(plan, model, data, addresses, ideal, feedback, args.output, args.plan)
    if not args.no_video:
        for frames, xyz, name in zip((ideal, feedback), trajectories, ('ideal', 'feedback')):
            if frames:
                export_video(model, data, addresses, frames, xyz, args.output/f'{name}.mp4', name.upper())
    if args.view:
        from mujoco import viewer as mj_viewer
        frames = feedback if args.view == 'feedback' else ideal
        xyz = trajectories[1 if args.view == 'feedback' else 0]
        if not frames:
            raise ValueError('No feedback recorded for this plan')
        phases = [f['phase'] for f in frames]
        index = 0
        paused = False
        def key(code):
            nonlocal paused, index
            if code == 32:
                paused = not paused
            if code in (82, 114):
                index = 0
        with mj_viewer.launch_passive(model, data, key_callback=key) as viewer:
            viewer.cam.lookat[:] = camera().lookat
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = .82, 135, -35
            while viewer.is_running():
                with viewer.lock():
                    set_pose(model, data, addresses, frames[index]['q'])
                    viewer.user_scn.ngeom = 0
                    trail(viewer.user_scn, xyz[:index+1], phases[:index+1], [1, .18, .05, 1])
                viewer.sync()
                if not paused:
                    index = min(index+1, len(frames)-1)
                time.sleep(1/60)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, default=DEFAULT)
    parser.add_argument('--output', type=Path, default=ROOT/'simulation/wang')
    parser.add_argument('--no-video', action='store_true')
    parser.add_argument('--view', choices=['ideal', 'feedback'])
    args = parser.parse_args()
    main(args)
