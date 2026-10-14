"""HTTP controller tests; all movement requests are mocked."""
import contextlib
import io
import math
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, Mock
import servo


class FakeClient:
    writes = []
    stuck = False
    invalid = False
    def __init__(self, *_):
        self.angles = [0, 4, 2, 0, 0, 115]
    def precheck(self):
        return {'servos_offsets': [2048]*6, 'servos_offsets_signs': [-1,1,1,1,-1,1]}
    def read(self):
        return self.angles.copy()
    def write(self, angles):
        self.writes.append(angles.copy())
        if not self.stuck:
            self.angles = angles.copy()
    def close(self):
        pass


class ControllerTest(unittest.TestCase):
    def setUp(self):
        FakeClient.writes = []
        FakeClient.stuck = False
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.log = Path(self.directory.name)/'motion.json'
    def run_cli(self, args):
        if args[0] == 'move':
            args = args + ['--log-file', str(self.log)]
        with patch.object(servo, 'Client', FakeClient), patch.object(servo.time, 'sleep'), contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            return servo.main(args)
    def test_preview_never_writes(self):
        self.assertEqual(self.run_cli(['move','--ids','1','--delta','2']), 0)
        self.assertEqual(FakeClient.writes, [])
        self.assertFalse(self.log.exists())
    def test_selected_axis_and_gripper_preservation(self):
        self.assertEqual(self.run_cli(['move','--ids','1','--delta','2','--execute']), 0)
        self.assertGreater(len(FakeClient.writes), 1)
        for row in FakeClient.writes:
            self.assertEqual(len(row), 6)
            self.assertEqual(row[1:], [4,2,0,0,115])
        self.assertEqual(FakeClient.writes[-1][0], 2)
        report = json.loads(self.log.read_text())
        self.assertEqual(report['status'], 'completed')
        self.assertEqual(report['planned_goal_angles'][0], 2)
        self.assertEqual(report['last_feedback_before_stop']['angles'][0], 2)
    def test_six_axis_mapping(self):
        self.assertEqual(self.run_cli(['move','--ids','1','2','3','4','5','6','--angles','1','2','3','4','5','116','--execute']), 0)
        self.assertEqual(FakeClient.writes[-1], [1,2,3,4,5,116])
    def test_reject_out_of_bounds_before_writing(self):
        self.assertEqual(self.run_cli(['move','--ids','1','2','--angles','1','999','--execute']), 1)
        self.assertEqual(FakeClient.writes, [])
    def test_tracking_error_stops_and_holds(self):
        FakeClient.stuck = True
        self.assertEqual(self.run_cli(['move','--ids','1','--delta','12','--execute']), 1)
        self.assertEqual(FakeClient.writes[-1], [0,4,2,0,0,115])
        self.assertLess(max(row[0] for row in FakeClient.writes), 12)
    def test_curve_speed_acceleration_and_endpoint(self):
        for distance in [0.1, 20]:
            dt = .01
            frames = servo.trajectory([0,0], [distance,-distance/2], 5, 10, dt)
            rows = [[0,0]] + frames
            speeds = [(b[0]-a[0])/dt for a,b in zip(rows,rows[1:])]
            self.assertLessEqual(max(speeds), 5+1e-8)
            for a,b in zip([0]+speeds, speeds+[0]):
                self.assertLessEqual(abs(b-a)/dt, 10+1e-7)
            self.assertEqual(frames[-1], [distance,-distance/2])
    def test_write_payload_uses_full_radians_without_subset(self):
        client = servo.Client('http://localhost', 0)
        client.call = Mock(return_value={})
        client.write([0,90,0,0,0,115])
        endpoint, payload = client.call.call_args.args
        self.assertEqual(endpoint, '/joints/write')
        self.assertEqual(payload['unit'], 'rad')
        self.assertNotIn('joints_ids', payload)
        self.assertAlmostEqual(payload['angles'][1], math.pi/2)
        client.close()
    def test_invalid_feedback_rejected(self):
        client = servo.Client('http://localhost', 0)
        client.call = Mock(return_value={'unit':'rad','angles':[None]*6})
        with self.assertRaises(RuntimeError): client.read()
        client.close()

    def test_timeout_preserves_pre_hold_feedback_and_goal(self):
        FakeClient.stuck = True
        clock = iter(i*.02 for i in range(10000))
        with patch.object(servo.time, 'monotonic', side_effect=lambda: next(clock)):
            result = self.run_cli(['move','--ids','1','--delta','2','--timeout','2','--execute'])
        self.assertEqual(result, 1)
        report = json.loads(self.log.read_text())
        self.assertIn('超时', report['stop_reason'])
        self.assertEqual(report['planned_goal_angles'][0], 2)
        self.assertEqual(report['last_feedback_before_stop']['angles'][0], 0)
        self.assertEqual(report['last_feedback_before_stop']['goal_error_deg'][0], 2)
        self.assertEqual(report['last_command_angles'][0], 2)
        self.assertEqual(report['hold_feedback_angles'][0], 0)

    def test_unwritable_log_prevents_movement(self):
        with patch.object(Path, 'write_text', side_effect=OSError('read-only')):
            self.assertEqual(self.run_cli(['move','--ids','1','--delta','2','--execute']), 1)
        self.assertEqual(FakeClient.writes, [])


if __name__ == '__main__':
    unittest.main()
