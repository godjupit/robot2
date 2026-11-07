import argparse
import threading
import time
import unittest
from server import State, control, mapped, validate


class ProtocolTests(unittest.TestCase):
    def test_invalid_depth_and_numbers(self):
        for xyz in ([0, 0, float('nan')], [0, 0, 2], [True, 0, .3], [0, .3]):
            with self.assertRaises(ValueError):
                validate({'enabled': True, 'xyz': xyz})
        self.assertIsNone(validate({'enabled': False}))

    def test_relative_axes(self):
        target = mapped([.25, 0, .2], [0, 0, .4], [.1, -.1, .5], .4)
        for actual, expected in zip(target, [.29, -.04, .24]):
            self.assertAlmostEqual(actual, expected)

    def test_preview_timeout_and_reanchor(self):
        state, stop = State(), threading.Event()
        args = argparse.Namespace(execute=False, scale=.4)
        thread = threading.Thread(target=control, args=(state, stop, args))
        thread.start()
        try:
            state.put([0, 0, .4])
            time.sleep(.16)
            self.assertTrue(state.result['active'])
            state.put([.05, 0, .4])
            time.sleep(.16)
            self.assertAlmostEqual(state.result['target_m'][1], -.02)
            time.sleep(.4)
            self.assertFalse(state.result['active'])
            state.put([.2, 0, .4])
            time.sleep(.16)
            self.assertEqual(state.result['target_m'], [.25, 0., .2])
        finally:
            stop.set()
            thread.join()

    def test_workspace_fault_latches(self):
        state, stop = State(), threading.Event()
        args = argparse.Namespace(execute=False, scale=.4)
        thread = threading.Thread(target=control, args=(state, stop, args))
        thread.start()
        state.put([0, 0, .4])
        time.sleep(.16)
        state.put([.3, 0, .4])
        thread.join(timeout=1)
        self.assertFalse(thread.is_alive())
        self.assertIn('workspace', state.result['fault'])
        state.put([0, 0, .4])
        self.assertFalse(state.result['active'])


if __name__ == '__main__':
    unittest.main()
