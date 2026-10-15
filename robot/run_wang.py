#!/usr/bin/env python3
"""王字四笔：横、横、竖、横；每笔先落笔、画线、抬笔，再转移。默认只预检。"""
import argparse
from datetime import datetime
import json
from pathlib import Path
from zoneinfo import ZoneInfo

from servo import Client, finite
from plan import ROOT, Model, FeedbackClient, build_plan, segment, solve_plan, execute_plan


def build_wang(current, center_x=27., center_y=.199, height=16., width=10.,
               paper_z=0., lift=5., spacing=1., selected_stroke=None, selected_strokes=None):
    top=center_x+height/2;bottom=center_x-height/2
    left=center_y-width/2;right=center_y+width/2
    strokes=[[[top,left,paper_z],[top,right,paper_z]],
             [[center_x,center_y-width*.35,paper_z],[center_x,center_y+width*.35,paper_z]],
             [[top,center_y,paper_z],[bottom,center_y,paper_z]],
             [[bottom,left,paper_z],[bottom,right,paper_z]]]
    if selected_stroke is not None and selected_stroke not in (1,2,3,4):
        raise ValueError('stroke 必须为 1..4')
    if selected_stroke is not None and selected_strokes is not None:
        raise ValueError('不能同时指定单笔与多笔')
    ids=selected_strokes if selected_strokes is not None else ([selected_stroke] if selected_stroke is not None else [1,2,3,4])
    if not ids or len(set(ids))!=len(ids) or any(i not in (1,2,3,4) for i in ids):
        raise ValueError('笔画编号必须为不重复的 1..4')
    chosen=[(i,strokes[i-1]) for i in ids]
    names=['上横','中横','竖','下横']
    plan=build_plan(current,paper_z, height)
    plan.update(character='王',strokes_cm=[s for _,s in chosen],stroke_order=[names[i-1] for i,_ in chosen],
                stroke_ids=[i for i,_ in chosen],selected_stroke=selected_stroke,
                return_home=False,line_start_cm=chosen[0][1][0],line_end_cm=chosen[-1][1][-1],
                drawing_points_cm=[p for _,stroke in chosen for p in segment(*stroke,spacing)],
                lift_cm=lift,lift_points_cm=[],stroke_targets=[],max_spacing_cm=spacing)
    targets=plan['stroke_targets'];previous=list(current)
    def append_to(phase,end,step,stroke_id,**meta):
        nonlocal previous
        for xyz in segment(previous,end,step)[1:]:
            targets.append(dict(phase=phase,xyz_cm=xyz,stroke_id=stroke_id,**meta))
        previous=list(end)
    # 先在当前位置垂直抬到转移高度，避免从低处斜着移动到起点。
    raised=[current[0],current[1],max(current[2],paper_z+lift)]
    if raised[2] > current[2]+1e-9:
        append_to('lift',raised,.5,chosen[0][0])
    for index,(start,end) in chosen:
        # 当前位于高处时，不在转移过程中同时下降；到达起点正上方后再落笔。
        transfer_z=max(previous[2],paper_z+lift)
        above=[start[0],start[1],transfer_z]
        append_to('travel',above,spacing,index,min_z_cm=paper_z+1.)
        hover=[start[0],start[1],paper_z+lift]
        append_to('hover',hover,.5,index,min_z_cm=paper_z+1.)
        targets[-1]['landing_ready']=True
        targets[-1]['arrival_tolerance_mm']=2.
        append_to('lower',start,.5,index,fixed_xy_cm=start[:2])
        append_to('draw',end,spacing,index)
        append_to('lift',[end[0],end[1],paper_z+lift],.5,index)
        targets[-1]['stroke_finished']=True
    plan['drawing_point_count']=len(plan['drawing_points_cm'])
    plan['require_hover_confirmation']=True
    plan['approach_policy']='Transfer above stroke start, confirm safe hover, then lower with fixed target XY'
    return plan


def preview(plan,destination):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(5,6))
    if plan.get('existing_vertical_cm'):
        a,b=plan['existing_vertical_cm']
        ax.plot([a[1],b[1]],[a[0],b[0]],'--',color='tab:blue',label='Existing vertical (not redrawn)')
        ax.legend(fontsize=8)
    for index,stroke in zip(plan['stroke_ids'],plan['strokes_cm']):
        ax.plot([p[1] for p in stroke],[p[0] for p in stroke],'-o',color='black')
        ax.text(stroke[0][1],stroke[0][0],str(index),color='tab:red')
    ax.set(xlabel='Model Y (cm)',ylabel='Model X (cm)',title='Wang: selected pen-down strokes')
    ax.set_aspect('equal');ax.grid(alpha=.3);fig.tight_layout();fig.savefig(destination,dpi=150);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://127.0.0.1:80')
    parser.add_argument('--robot-id',type=int,default=0)
    parser.add_argument('--rod-length-cm',type=finite,help='末端原点到笔尖长度；显式指定优先于 reference-line，否则继承记录或默认 16 cm')
    parser.add_argument('--center-x',type=finite,default=27.)
    parser.add_argument('--center-y',type=finite,default=.199)
    parser.add_argument('--offset-x-cm',type=finite,default=0.,help='在指定中心或 reference-line 中心基础上沿模型 X 平移，正数远离底座')
    parser.add_argument('--height-cm',type=finite,default=16.)
    parser.add_argument('--width-cm',type=finite,default=10.)
    selection=parser.add_mutually_exclusive_group()
    selection.add_argument('--stroke',type=int,choices=[1,2,3,4],help='只画指定笔画；3 是竖线')
    selection.add_argument('--strokes',type=int,nargs='+',choices=[1,2,3,4],help='选取多笔；1 2 4 只补三条横线')
    parser.add_argument('--reference-line',type=Path,help='读取已执行竖线的 planning.json，继承其位置、笔长、补偿及保护参数；宽度仍由 width-cm 指定')
    parser.add_argument('--no-return-home',action='store_true',help='不执行 reference-line 中的回零，从当前实测姿态开始抬笔转移')
    parser.add_argument('--paper-z-cm',type=finite,default=0.)
    parser.add_argument('--lift-cm',type=finite,default=5.)
    parser.add_argument('--spacing-mm',type=finite,default=10.)
    parser.add_argument('--xyz-tolerance-mm',type=finite,default=10.)
    parser.add_argument('--z-below-tolerance-mm',type=finite,default=5.)
    parser.add_argument('--total-timeout',type=finite,default=600.)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    reference=None
    if args.reference_line:
        reference=json.loads(args.reference_line.read_text())
        start=reference['line_start_cm'];end=reference['line_end_cm']
        if reference.get('unit')!='cm' or abs(start[1]-end[1])>1e-6 or abs(start[2]-end[2])>1e-6:
            parser.error('reference-line 必须是沿模型 X 的竖线，单位 cm')
        offset=reference.get('tcp_offset_cm')
        if not offset or len(offset)!=3 or offset[0]!=0 or offset[2]!=0 or offset[1]>=0:
            parser.error('reference-line 需要沿 Fixed_Jaw -Y 的 TCP 偏移')
        settings=reference['execution_settings'];guard=reference['drawing_z_protection']
        if args.rod_length_cm is None:
            args.rod_length_cm=-offset[1]
        args.center_x=(start[0]+end[0])/2;args.center_y=start[1]
        args.height_cm=abs(start[0]-end[0]);args.paper_z_cm=start[2]
        args.lift_cm=reference['lift_cm'];args.spacing_mm=reference['max_spacing_cm']*10
        args.xyz_tolerance_mm=settings['xyz_tolerance_mm']
        if guard['paper_z_cm']!=args.paper_z_cm:
            parser.error('reference-line 纸面高度与绘图高度不一致')
        args.z_below_tolerance_mm=guard['z_below_tolerance_mm']
        args.total_timeout=json.loads((args.reference_line.parent/'execution.json').read_text()).get('total_timeout_s',360.)
    if args.rod_length_cm is None:
        args.rod_length_cm=16.
    args.center_x+=args.offset_x_cm
    if (args.robot_id<0 or not 0<args.rod_length_cm<=50 or not 0<args.height_cm<=20
            or not 0<args.width_cm<=20 or not 1<args.lift_cm<=10
            or not 1<=args.spacing_mm<=10 or not 1<=args.xyz_tolerance_mm<=10
            or not 0<=args.z_below_tolerance_mm<=5 or not 60<=args.total_timeout<=1200):
        parser.error('尺寸/间距/误差/超时超出允许范围；lift-cm 需 >1 且 ≤10')
    client=Client(args.url,args.robot_id)
    try:
        config=client.precheck();feedback=FeedbackClient(client,config)
        angles=feedback.read();model=Model([0,-args.rod_length_cm,0])
        current=(model.position(model.np.deg2rad(angles[:5]))*100).tolist()
        returning=bool(reference and reference.get('return_home') and not args.no_return_home)
        path_seed=(model.position(model.np.zeros(5))*100).tolist() if returning else current
        plan=build_wang(path_seed,args.center_x,args.center_y,args.height_cm,args.width_cm,
                        args.paper_z_cm,args.lift_cm,args.spacing_mm/10,args.stroke,args.strokes)
        plan['current_xyz_cm']=current
        plan['return_home']=returning
        plan.update(source_joint_angles_deg=angles,tcp_offset_cm=[0,-args.rod_length_cm,0],
                    reference_point='Rod tip TCP attached to Fixed_Jaw',robot_id=args.robot_id,
                    service_url=args.url,recorded_at=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
                    drawing_z_protection=dict(paper_z_cm=args.paper_z_cm,
                                             z_below_tolerance_mm=args.z_below_tolerance_mm),
                    execution_settings=dict(xyz_tolerance_mm=args.xyz_tolerance_mm,
                                            control_step_deg_by_joint=[.5,1.,1.,.5,.5],
                                            total_timeout_s=args.total_timeout))
        if reference:
            plan['reference_line']=str(args.reference_line.resolve())
            plan['existing_vertical_cm']=[reference['line_start_cm'],reference['line_end_cm']]
            plan['offset_x_cm']=args.offset_x_cm
            for key in ('joint_tolerance_deg','point_timeout_s','control_step_deg','feedback_interval_s','control_step_deg_by_joint'):
                if key in reference['execution_settings']:
                    plan['execution_settings'][key]=reference['execution_settings'][key]
        run_dir=ROOT/'垃圾文件夹/plans'/f'wang-{datetime.now(ZoneInfo("Asia/Shanghai")):%Y%m%d-%H%M%S-%f}'
        run_dir.mkdir(parents=True,exist_ok=False);destination=run_dir/'planning.json'
        preview(plan,run_dir/'planning.png')
        try:
            solve_plan(plan,model)
        except Exception as exc:
            plan['precheck_error']=str(exc)
            raise
        finally:
            destination.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n')
        print(f'王字：高 {args.height_cm:g} cm、宽 {args.width_cm:g} cm；本次笔画：'+ '、'.join(plan['stroke_order']),flush=True)
        print(f'模型笔长 {args.rod_length_cm:g} cm；'+('先回零。' if returning else '不回零，从当前实测姿态开始。'),flush=True)
        print(f'纸面 Z={args.paper_z_cm:g} cm，每笔抬到 Z={args.paper_z_cm+args.lift_cm:g} cm 后转移。',flush=True)
        print(f'完整 IK/限位/模型碰撞预检通过，共 {len(plan["solutions"])} 个目标。',flush=True)
        print(f'规划：{destination}\n预览：{run_dir/"planning.png"}',flush=True)
        if args.execute:
            if client.precheck()!=config:
                raise RuntimeError('预检后标定配置改变，未发送运动')
            execute_plan(feedback,model,plan,config,destination,
                         joint_tolerance=plan['execution_settings'].get('joint_tolerance_deg',2.),
                         xyz_tolerance_mm=args.xyz_tolerance_mm,
                         point_timeout=plan['execution_settings'].get('point_timeout_s',60.),
                         total_timeout=args.total_timeout)
        else:
            print('仅预检，未发送运动。加 --execute 才执行。',flush=True)
    finally:
        client.close()


if __name__=='__main__':
    main()
