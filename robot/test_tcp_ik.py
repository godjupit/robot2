"""离线验证 TCP 变换、位置 IK 和位置到位判定；不访问真机。"""
import time
import unittest
import numpy as np
from plan import Model, move_point_with_compensation, build_plan, resample_replay_points, choose_drawing_compensation


class TcpIkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = Model([0, -17, 0])

    def test_offset_rotates_with_jaw(self):
        m = self.model
        for q in (np.zeros(5), np.array([.1, -.2, .3, -.1, .4])):
            t = m.transforms(q, 0)['Fixed_Jaw']
            expected = (t @ np.array([0, -.17, 0, 1]))[:3]
            np.testing.assert_allclose(m.position(q), expected, atol=1e-12)
            self.assertAlmostEqual(np.linalg.norm(m.position(q)-t[:3,3]), .17)

    def test_ik_targets_tip_instead_of_jaw_origin(self):
        m = self.model
        target = m.position(np.array([.1, -.2, .3, -.1, .2]))
        solved = m.solve_position(target, np.zeros(5))
        self.assertLess(np.linalg.norm(m.position(solved)-target), .0005)
        origin = m.transforms(solved, 0)['Fixed_Jaw'][:3,3]
        self.assertGreater(np.linalg.norm(origin-target), .16)

    def test_tcp_arrival_does_not_require_same_joint_solution(self):
        m = self.model
        # TCP 在腕旋转轴上：旋转腕部仍可保持尖端位置。
        actual = [0, 0, 0, 0, 20, 0]
        class Client:
            def read(self):
                return actual.copy()
        goal = {'phase':'draw', 'joint_angles_deg':[0]*5,
                'xyz_cm':(m.position(np.zeros(5))*100).tolist()}
        def reject_write(values):
            self.fail('TCP 已到位，不应为匹配腕部角度而写入运动')
        _, sample = move_point_with_compensation(
            Client(), m, goal, np.zeros(5), reject_write, lambda s: None,
            time.monotonic()+5, interval=0)
        self.assertGreater(sample['max_joint_error_deg'], 2)
        self.assertLess(sample['position_error_mm'], .001)

    def test_invalid_tcp_offset_is_rejected(self):
        with self.assertRaises(ValueError):
            Model([0, float('nan'), 0])

    def test_ten_mm_spacing_replaces_coarse_replay_points(self):
        plan=build_plan([30.413, .199, 5], z_cm=0, length_cm=10, points=7)
        resample_replay_points(plan,1.)
        self.assertEqual(plan['drawing_point_count'],11)
        for a,b in zip(plan['drawing_points_cm'],plan['drawing_points_cm'][1:]):
            self.assertAlmostEqual(np.linalg.norm(np.array(b)-a),1.)

    def test_z_protection_precedes_loose_arrival_and_writes(self):
        # 重现 TCP 已到位于 10 mm 容差内，但 Z 下扎 2 mm 的情况。
        class FakeModel:
            np=np
            tcp_mode=True
            def check(self,*args): pass
            def position(self,q): return np.array([0.,0.,-.002])
        class Client:
            def read(self): return [0.]*6
        rows=[]
        def reject_write(values):
            self.fail('Z 保护触发后不能继续写入补偿目标')
        with self.assertRaisesRegex(RuntimeError,'绘图 Z 高度保护'):
            move_point_with_compensation(Client(),FakeModel(),
                {'phase':'draw','joint_angles_deg':[0.]*5,'xyz_cm':[0.,0.,0.]},
                np.zeros(5),reject_write,rows.append,time.monotonic()+5,interval=0,
                xyz_tolerance_mm=10,paper_z_cm=0,z_below_tolerance_mm=1)
        self.assertEqual(len(rows),1)
        self.assertAlmostEqual(rows[0]['below_paper_mm'],2.)

    def test_recorded_drawing_pose_selects_one_height_checked_axis(self):
        actual=np.array([.2637362637,-26.989010989,43.516483516,43.340659341,.1758241758])
        command=np.array([1.2538,-28.10,43.28,43.73,-.26])
        goal=np.array([.4763506327,-30.19,44.15,49.55,0.])
        proposed,choice=choose_drawing_compensation(self.model,actual,command,goal-actual,
            np.array([28.4134714,.19865225,0.]),abs(goal-actual)>.1,0.)
        self.assertEqual(np.count_nonzero(abs(proposed-command)>1e-12),1)
        self.assertTrue(np.all(abs(proposed-command)<=np.array([.5,1.,1.,.5,.5])))
        self.assertGreaterEqual(choice['predicted_min_z_cm'],-1e-9)

    def test_near_paper_prioritizes_lifting_joint(self):
        class FakeModel:
            np=np
            def position(self,q):
                return np.array([q[1],0.,.001+q[0]-q[1]])
        actual=np.zeros(5)
        proposed,choice=choose_drawing_compensation(FakeModel(),actual,actual,
            np.array([1.,1.,0.,0.,0.]),np.array([2.,0.,0.]),
            np.array([True,True,False,False,False]),0.)
        self.assertEqual(choice['joint_id'],1)
        self.assertGreater(choice['predicted_next_xyz_cm'][2],.1)

    def test_shoulder_compensation_uses_one_degree_limit(self):
        class FakeModel:
            np=np
            def position(self,q):
                return np.array([0.,q[1],.01])
        actual=np.zeros(5)
        proposed,choice=choose_drawing_compensation(FakeModel(),actual,actual,
            np.array([0.,3.,0.,0.,0.]),np.array([0.,5.,1.]),
            np.array([False,True,False,False,False]),0.)
        self.assertEqual(choice['joint_id'],2)
        self.assertAlmostEqual(proposed[1],1.)

    def test_elbow_compensation_uses_one_degree_limit(self):
        class FakeModel:
            np=np
            def position(self,q):
                return np.array([q[2],0.,.01])
        actual=np.zeros(5)
        proposed,choice=choose_drawing_compensation(FakeModel(),actual,actual,
            np.array([0.,0.,3.,0.,0.]),np.array([5.,0.,1.]),
            np.array([False,False,True,False,False]),0.)
        self.assertEqual(choice['joint_id'],3)
        self.assertAlmostEqual(proposed[2],1.)

    def test_prediction_uses_configured_paper_tolerance(self):
        # 已有发送目标包含 -0.2 mm 的组合，实际允许 -1 mm，不应拒绝抬高候选。
        class FakeModel:
            np=np
            def position(self,q):
                return np.array([0.,0.,.001+q[0]-q[1]])
        actual=np.zeros(5)
        command=np.array([0.,np.rad2deg(.0012),0.,0.,0.])
        args=(FakeModel(),actual,command,np.array([1.,0.,0.,0.,0.]),
              np.array([0.,0.,1.]),np.array([True,False,False,False,False]),0.)
        with self.assertRaisesRegex(RuntimeError,'没有可改善位置'):
            choose_drawing_compensation(*args,z_below_tolerance_mm=0)
        proposed,choice=choose_drawing_compensation(*args,z_below_tolerance_mm=1)
        self.assertEqual(choice['joint_id'],1)
        self.assertAlmostEqual(choice['prediction_floor_z_cm'],-.1)
        self.assertGreaterEqual(choice['predicted_min_z_cm'],-.1)

    def test_rejects_pending_axis_motion_that_would_cross_paper(self):
        class FakeModel:
            np=np
            def position(self,q):
                return np.array([0.,0.,q[0]-q[1]])
        actual=np.zeros(5);command=np.array([0.,1.,0.,0.,0.])
        with self.assertRaisesRegex(RuntimeError,'没有可改善位置'):
            choose_drawing_compensation(FakeModel(),actual,command,
                np.array([1.,0.,0.,0.,0.]),np.array([1.,0.,0.]),
                np.array([True,False,False,False,False]),0.)

    def test_wang_has_four_strokes_with_lift_before_each_transfer(self):
        from run_wang import build_wang
        p=build_wang([20.,0.,5.])
        self.assertEqual(len(p['strokes_cm']),4)
        self.assertEqual(p['stroke_order'],['上横','中横','竖','下横'])
        finished=[t for t in p['stroke_targets'] if t.get('stroke_finished')]
        self.assertEqual(len(finished),4)
        for t in finished:
            self.assertEqual(t['phase'],'lift')
            self.assertEqual(t['xyz_cm'][2],5.)
        for index in range(1,5):
            targets=[t for t in p['stroke_targets'] if t['stroke_id']==index]
            phases=[t['phase'] for t in targets]
            self.assertLess(phases.index('travel'),phases.index('lower'))
            self.assertLess(phases.index('lower'),phases.index('draw'))
            self.assertLess(phases.index('draw'),phases.index('lift'))
            self.assertTrue(all(t['xyz_cm'][2]==5. for t in targets if t['phase']=='travel'))
            self.assertTrue(all(t['min_z_cm']==1. for t in targets if t['phase']=='travel'))

    def test_wang_vertical_only_uses_original_third_stroke(self):
        from run_wang import build_wang
        p=build_wang([35.,0.,5.],center_x=37.,center_y=.199,height=8.,width=6.,selected_stroke=3)
        self.assertEqual(p['strokes_cm'],[[[41.,.199,0.],[33.,.199,0.]]])
        self.assertEqual(p['stroke_order'],['竖'])
        self.assertEqual(p['stroke_ids'],[3])
        self.assertEqual(p['line_start_cm'],[41.,.199,0.])
        self.assertEqual(p['line_end_cm'],[33.,.199,0.])
        self.assertTrue(all(t['stroke_id']==3 for t in p['stroke_targets']))
        self.assertEqual(sum(t.get('stroke_finished',False) for t in p['stroke_targets']),1)
        draws=[t for t in p['stroke_targets'] if t['phase']=='draw']
        self.assertEqual(len(draws),8)
        self.assertTrue(all(t['xyz_cm'][1:]==[.199,0.] for t in draws))

    def test_three_horizontals_align_with_five_cm_vertical(self):
        from run_wang import build_wang
        p=build_wang([30.,0.,5.],center_x=27.,center_y=.199,height=5.,width=6.,selected_strokes=[1,2,4])
        self.assertEqual(p['stroke_ids'],[1,2,4])
        self.assertEqual(p['stroke_order'],['上横','中横','下横'])
        self.assertEqual([s[0][0] for s in p['strokes_cm']],[29.5,27.,24.5])
        self.assertTrue(all(s[0][0]==s[1][0] for s in p['strokes_cm']))
        self.assertEqual(sum(t.get('stroke_finished',False) for t in p['stroke_targets']),3)
        self.assertNotIn(3,{t['stroke_id'] for t in p['stroke_targets']})

    def test_high_pose_transfers_level_then_lowers_directly_above_start(self):
        from run_wang import build_wang
        p=build_wang([44.8,0.,24.3],center_x=32.,height=5.,width=6.,selected_stroke=3)
        travel=[t for t in p['stroke_targets'] if t['phase']=='travel']
        lower=[t for t in p['stroke_targets'] if t['phase']=='lower']
        self.assertTrue(all(abs(t['xyz_cm'][2]-24.3)<1e-9 for t in travel))
        self.assertTrue(all(t['xyz_cm'][:2]==[34.5,.199] for t in lower))
        self.assertTrue(all(a['xyz_cm'][2]>b['xyz_cm'][2] for a,b in zip(lower,lower[1:])))
        self.assertEqual(lower[-1]['xyz_cm'],[34.5,.199,0.])

    def test_each_stroke_confirms_hover_before_fixed_xy_descent(self):
        from run_wang import build_wang
        for initial_z in (2.,5.,24.3):
            p=build_wang([30.,0.,initial_z],center_x=32.,height=5.,width=6.)
            self.assertTrue(p['require_hover_confirmation'])
            for stroke_id,stroke in zip(p['stroke_ids'],p['strokes_cm']):
                targets=[t for t in p['stroke_targets'] if t['stroke_id']==stroke_id]
                ready=[i for i,t in enumerate(targets) if t.get('landing_ready')]
                self.assertEqual(len(ready),1)
                hover=targets[ready[0]]
                self.assertEqual(hover['phase'],'hover')
                self.assertEqual(hover['arrival_tolerance_mm'],2.)
                self.assertEqual(hover['xyz_cm'],stroke[0][:2]+[5.])
                previous=hover['xyz_cm']
                for i,t in enumerate(targets):
                    if t['phase']!='lower': continue
                    self.assertGreater(i,ready[0])
                    self.assertEqual(t['xyz_cm'][:2],stroke[0][:2])
                    self.assertEqual(t['fixed_xy_cm'],stroke[0][:2])
                    self.assertGreater(previous[2]-t['xyz_cm'][2],0)
                    self.assertLessEqual(previous[2]-t['xyz_cm'][2],.5+1e-9)
                    previous=t['xyz_cm']

    def test_default_paper_protection_triggers_below_five_mm(self):
        class FakeModel:
            np=np
            tcp_mode=True
            def __init__(self,z): self.z=z
            def check(self,*args): pass
            def position(self,q): return np.array([0.,0.,self.z])
        class Client:
            def read(self): return [0.]*6
        def run(z):
            return move_point_with_compensation(Client(),FakeModel(z),
                {'phase':'draw','joint_angles_deg':[0.]*5,'xyz_cm':[0.,0.,0.]},
                np.zeros(5),lambda values:self.fail('不应发送补偿'),lambda s:None,
                time.monotonic()+5,interval=0,paper_z_cm=0)
        run(-.0049)
        with self.assertRaisesRegex(RuntimeError,'绘图 Z 高度保护'):
            run(-.0051)

    def test_hover_cannot_arrive_with_normal_ten_mm_tolerance(self):
        class FakeModel:
            np=np
            tcp_mode=True
            def check(self,*args): pass
            def position(self,q): return np.array([.005,0.,.05])
        class Client:
            def read(self): return [0.]*6
        rows=[];writes=[]
        with self.assertRaisesRegex(RuntimeError,'连续 8 次反馈'):
            move_point_with_compensation(Client(),FakeModel(),
                {'phase':'hover','joint_angles_deg':[1.]*5,'xyz_cm':[0.,0.,5.],
                 'arrival_tolerance_mm':2.},np.zeros(5),writes.append,rows.append,
                time.monotonic()+5,interval=0,xyz_tolerance_mm=10)
        self.assertTrue(writes)  # 5 mm 偏差必须继续补偿，不能提前到位。
        self.assertTrue(all(r['arrival_tolerance_mm']==2. for r in rows))

    def test_execution_rejects_lower_without_hover_confirmation(self):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        from plan import execute_plan
        initial=[3.,0.,0.,0.,0.,115.]
        class Client:
            def read(self): return initial.copy()
            def call(self,*args,**kwargs): raise AssertionError('不应发送运动')
        p={'source_joint_angles_deg':initial,'require_hover_confirmation':True,
           'solutions':[{'phase':'lower','stroke_id':1,'xyz_cm':[32.,0.,0.],
                         'fixed_xy_cm':[32.,0.],'joint_angles_deg':initial[:5]}]}
        with tempfile.TemporaryDirectory() as folder, patch('plan.save_execution_preview'), \
                patch('plan.move_point_with_compensation') as move:
            with self.assertRaisesRegex(RuntimeError,'悬停尚未确认'):
                execute_plan(Client(),self.model,p,{},Path(folder)/'planning.json')
            move.assert_not_called()

    def test_execution_passes_saved_half_degree_steps_to_controller(self):
        import tempfile
        import contextlib
        import io
        from pathlib import Path
        from unittest.mock import patch
        from plan import execute_plan
        initial=[3.,0.,0.,0.,0.,115.]
        xyz=(self.model.position(np.deg2rad(initial[:5]))*100).tolist()
        class Client:
            def read(self): return initial.copy()
            def call(self,*args,**kwargs): raise AssertionError('离线测试不应发送运动')
        p={'source_joint_angles_deg':initial,'solutions':[
            {'phase':'draw','xyz_cm':xyz,'joint_angles_deg':initial[:5]}],
           'line_end_cm':xyz,'line_z_cm':xyz[2],'drawing_points_cm':[xyz,xyz],
           'execution_settings':{'control_step_deg_by_joint':[.5]*5}}
        received={}
        def move(client,model,point,command,write,record,*args,**kwargs):
            received.update(kwargs)
            sample={'actual_angles_deg':initial,'actual_xyz_cm':xyz,
                    'max_joint_error_deg':0.,'position_error_mm':0.}
            record(sample)
            return command,sample
        with tempfile.TemporaryDirectory() as folder, patch('plan.move_point_with_compensation',move), \
                patch('plan.save_execution_preview'),contextlib.redirect_stdout(io.StringIO()):
            execute_plan(Client(),self.model,p,{},Path(folder)/'planning.json')
            result=__import__('json').loads((Path(folder)/'execution.json').read_text())
            self.assertEqual(result['control_step_deg_by_joint'],[.5]*5)
        self.assertEqual(received['step_deg'],.5)
        self.assertEqual(received['shoulder_step_deg'],.5)
        self.assertEqual(received['elbow_step_deg'],.5)


if __name__ == '__main__':
    unittest.main()
