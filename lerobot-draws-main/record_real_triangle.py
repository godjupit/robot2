"""Record real Phospho joint feedback during an explicitly requested triangle run.

Replay uses measured angles directly, with no inverse kinematics or fabricated
intermediate samples. The URDF end frame is not a calibrated physical pen tip.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np
import requests
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Run the same 10 cm triangle while recording')
    parser.add_argument('--urdf', type=Path, default=ROOT/'outputs/triangle/simulation/so-100.urdf')
    args = parser.parse_args()
    output = ROOT/'outputs'/('real-feedback-'+datetime.now().strftime('%Y%m%d-%H%M%S'))
    output.mkdir(parents=True)
    chain = []
    source = ET.parse(args.urdf).getroot()
    for joint in source.findall('joint')[:5]:
        origin = joint.find('origin')
        transform = np.eye(4)
        transform[:3,:3] = Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
        transform[:3,3] = np.fromstring(origin.get('xyz'),sep=' ')
        chain.append((transform,np.fromstring(joint.find('axis').get('xyz'),sep=' ')))
    def fk(q):
        transform=np.eye(4)
        transform[:3,:3]=Rotation.from_quat([0,0,1,1]).as_matrix()
        nodes=[transform[:3,3].copy()]
        for (origin,axis),angle in zip(chain,q):
            transform=transform@origin
            nodes.append(transform[:3,3].copy())
            motion=np.eye(4)
            motion[:3,:3]=Rotation.from_rotvec(axis*angle).as_matrix()
            transform=transform@motion
        return np.array(nodes)
    def read(session):
        response=session.post('http://127.0.0.1/joints/read?robot_id=0',
                              json={'unit':'rad','source':'robot'},timeout=3)
        response.raise_for_status()
        data=response.json()
        angles=np.asarray(data['angles'],dtype=float)
        if data.get('unit')!='rad' or angles.shape!=(6,) or not np.all(np.isfinite(angles)):
            raise ValueError(f'Invalid feedback: {data}')
        return data
    with requests.Session() as session:
        initial=read(session)
        config=session.post('http://127.0.0.1/robot/config?robot_id=0',timeout=3)
        config.raise_for_status()
        (output/'calibration.json').write_text(json.dumps(config.json(),indent=2))
    (output/'initial_feedback.json').write_text(json.dumps(initial,indent=2))
    phase=['Initial pose']
    samples=[];errors=[];events=[];done=threading.Event()
    started=time.monotonic()
    def sample():
        with requests.Session() as session, (output/'feedback.jsonl').open('w') as handle:
            while not done.is_set():
                begin=time.monotonic()
                try:
                    data=read(session)
                    row={'time_s':time.monotonic()-started,'phase':phase[0],**data}
                    samples.append(row)
                    handle.write(json.dumps(row)+'\n');handle.flush()
                except (requests.RequestException,ValueError,KeyError) as exc:
                    errors.append({'time_s':time.monotonic()-started,'error':str(exc)})
                done.wait(max(0,.1-(time.monotonic()-begin)))
    worker=threading.Thread(target=sample)
    worker.start()
    returncode=0
    try:
        time.sleep(.5)
        if args.execute:
            with (output/'execution.log').open('w') as log:
                process=subprocess.Popen([sys.executable,'-u',str(ROOT/'run_image.py'),
                    str(ROOT/'examples/triangle.png'),'--port','80','--size-cm','10','--execute'],
                    cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
                for line in process.stdout:
                    print(line,end='',flush=True);log.write(line);log.flush()
                    events.append({'time_s':time.monotonic()-started,'text':line.strip()})
                    if line.startswith('Moving to start'):phase[0]='Travel to start'
                    elif line.startswith('Step '):phase[0]=line.split(' ->')[0]
                    elif line.startswith('Original Phospho'):phase[0]='Final pose'
                returncode=process.wait()
        time.sleep(2)
    finally:
        done.set();worker.join(timeout=5)
    (output/'events.json').write_text(json.dumps(events,indent=2))
    (output/'errors.json').write_text(json.dumps(errors,indent=2))
    if not samples:raise RuntimeError('No usable real feedback was recorded')
    nodes=np.array([fk(s['angles'][:5])*100 for s in samples])
    positions=nodes[:,-1]
    times=np.array([s['time_s'] for s in samples])
    stroke=np.array([s['phase'].startswith('Step') for s in samples])
    report={'source':'Measured /joints/read angles in radians; no IK or inserted frames',
        'execute_requested':args.execute,'execution_returncode':returncode,'samples':len(samples),
        'read_errors':len(errors),'recording_seconds':float(times[-1]),
        'median_sample_interval_seconds':float(np.median(np.diff(times))) if len(times)>1 else None,
        'end_frame_height_range_during_stroke_cm':np.ptp(positions[stroke,2]).item() if np.any(stroke) else None,
        'pen_tip_measured':False}
    (output/'report.json').write_text(json.dumps(report,indent=2))
    fig=plt.figure(figsize=(12,5),facecolor='#f6f8fa')
    ax=fig.add_subplot(121,projection='3d');top=fig.add_subplot(222);height=fig.add_subplot(224)
    bounds=nodes.reshape(-1,3);center=(bounds.min(0)+bounds.max(0))/2;radius=max(np.ptp(bounds,axis=0))/2+3
    ax.set(xlabel='X (cm)',ylabel='Y (cm)',zlabel='Z (cm)',title='Arm driven by REAL measured joint angles')
    ax.set_xlim(center[0]-radius,center[0]+radius);ax.set_ylim(center[1]-radius,center[1]+radius)
    ax.set_zlim(min(0,center[2]-radius),center[2]+radius);ax.set_box_aspect((1,1,1));ax.view_init(elev=25,azim=-55)
    arm,=ax.plot([],[],[],'o-',color='#2563eb',lw=6,ms=5)
    trail,=ax.plot([],[],[],color='#16a34a',lw=2)
    ax.plot(positions[:,0],positions[:,1],positions[:,2],color='#16a34a',alpha=.15)
    top.plot(positions[:,0],positions[:,1],color='#16a34a',lw=1.5)
    top.set(xlabel='X (cm)',ylabel='Y (cm)',title='Top view: end-frame path from real feedback')
    top.set_aspect('equal',adjustable='datalim');top.grid(alpha=.2)
    marker,=top.plot([],[],'o',color='#db2777')
    height.plot(times,positions[:,2],color='#2563eb');height.grid(alpha=.2)
    height.set(xlabel='Recorded time (s)',ylabel='Z (cm)',title='End-frame height from measured joint angles')
    cursor=height.axvline(0,color='#db2777');label=fig.text(.04,.025,'',fontsize=9)
    fig.suptitle('REAL FEEDBACK REPLAY — measured joints / model end frame / pen tip not measured',fontsize=12)
    fig.tight_layout(rect=(0,.07,1,.93))
    def update(i):
        n=nodes[i];p=positions[i]
        arm.set_data_3d(n[:,0],n[:,1],n[:,2]);trail.set_data_3d(positions[:i+1,0],positions[:i+1,1],positions[:i+1,2])
        marker.set_data([p[0]],[p[1]]);cursor.set_xdata([times[i],times[i]])
        label.set_text(f't={times[i]:.2f}s  {samples[i]["phase"]}   Model end-frame XYZ={p.round(2)} cm   REAL joints={np.round(np.rad2deg(samples[i]["angles"][:5]),1)} deg')
    # Select measured frames only; do not smooth or invent feedback.
    ids=np.unique(np.linspace(0,len(samples)-1,min(180,len(samples))).astype(int))
    fps=max(1,len(ids)/max(times[-1],.1))
    update(len(samples)-1);fig.savefig(output/'overview.png',dpi=130)
    animation=FuncAnimation(fig,update,frames=ids,interval=1000/fps)
    animation.save(output/'replay.gif',writer=PillowWriter(fps=fps),dpi=90);plt.close(fig)
    (output/'index.html').write_text('''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>真机关节反馈回放</title>
<style>body{font:16px system-ui;max-width:1200px;margin:30px auto;padding:0 20px;background:#f6f8fa}img{width:100%}p{line-height:1.7}</style>
<h1>10 cm 三角形：真机关节反馈回放</h1><p>蓝色机械臂由真机实际返回的关节角度驱动，没有重新求解 IK。绿色是真实关节反馈换算出的模型末端轨迹。右下是模型末端高度。</p>
<img src="replay.gif"><p>数据来自本次执行期间的 /joints/read。显示的是 URDF 末端参考点；笔的长度和安装未测量，不能把它称为实测笔尖轨迹。动画只选取已采集帧，不插入理想中间点。</p>
<p><a href="feedback.jsonl">原始反馈</a> · <a href="report.json">采集报告</a> · <a href="execution.log">执行日志</a></p></html>''')
    print(json.dumps(report,indent=2),flush=True)
    print(output/'index.html',flush=True)
    if returncode:raise SystemExit(returncode)


if __name__=='__main__':
    main()
