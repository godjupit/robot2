"""Offline image/coordinate/reach audit. No HTTP requests or robot motion."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from image_to_trajectory.api import ImageToPathConverter, PipelineConfig

ROOT = Path(__file__).resolve().parent


def main():
    out = ROOT / 'outputs/triangle/diagnosis'
    out.mkdir(parents=True, exist_ok=True)
    converter = ImageToPathConverter(PipelineConfig())
    extracted = converter.process_image(cv2.imread(str(ROOT/'examples/triangle.png')))
    points = np.array([[p.x, p.y] for p in extracted[0].points])
    commands = json.loads((ROOT/'outputs/triangle/http_commands.json').read_text())
    draw = np.array([[c['data'][k] for k in ('x', 'y', 'z')]
                     for c in commands if c['data'].get('z') == 2])
    assert len(draw) == len(points)
    assert np.allclose(draw[:, :2], points * 10 / np.ptp(points, axis=0).max())
    chain = []
    for joint in ET.parse(ROOT/'outputs/triangle/simulation/so-100.urdf').getroot().findall('joint')[:5]:
        origin = joint.find('origin')
        t = np.eye(4)
        t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
        t[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
        limit = joint.find('limit')
        chain.append((t, np.fromstring(joint.find('axis').get('xyz'), sep=' '),
                      float(limit.get('lower')), float(limit.get('upper'))))

    def fk(q):
        t = np.eye(4)
        t[:3, :3] = Rotation.from_quat([0, 0, 1, 1]).as_matrix()
        for (origin, axis, _, _), angle in zip(chain, q):
            t = t @ origin
            motion = np.eye(4)
            motion[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            t = t @ motion
        return t[:3, 3]

    initial = fk(np.zeros(5)) * 100
    targets = initial + draw
    base_rotation = Rotation.from_quat([0, 0, 1, 1]).as_matrix()
    pivot = base_rotation @ chain[0][0][:3, 3] * 100
    # Triangle inequality: rigorous upper reach bound, regardless of joint angles.
    reach_bound = sum(np.linalg.norm(j[0][:3, 3]) for j in chain[1:]) * 100
    lower, upper = np.array([[j[2], j[3]] for j in chain]).T
    rng = np.random.default_rng(0)
    rows = []
    for i, target in enumerate(targets):
        solutions = []
        for seed in [np.zeros(5), *rng.uniform(lower, upper, size=(12, 5))]:
            result = least_squares(lambda q: fk(q)*100-target, seed,
                                   bounds=(lower, upper), max_nfev=200)
            solutions.append(float(np.linalg.norm(result.fun)))
        rows.append({'point': i+1, 'target_model_cm': target.tolist(),
                     'distance_from_first_pivot_cm': float(np.linalg.norm(target-pivot)),
                     'provable_min_error_cm': float(max(0, np.linalg.norm(target-pivot)-reach_bound)),
                     'best_position_only_ik_error_cm': min(solutions)})
    report = {'motion_sent': False, 'image_contour_points': len(converter.last_contours[0]),
              'simplified_points': len(points), 'drawing_span_cm': np.ptp(draw, axis=0).tolist(),
              'closure_gap_cm': float(np.linalg.norm(draw[-1]-draw[0])),
              'initial_model_cm': initial.tolist(), 'first_pivot_cm': pivot.tolist(),
              'upper_reach_bound_from_first_pivot_cm': float(reach_bound),
              'solver_note': 'Independent multi-start position-only SciPy IK, not a Phospho solver replay. Bound is geometric.',
              'points': rows}
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    actual = np.genfromtxt(ROOT/'outputs/real-feedback-20261013-224927/actual_points.csv',
                          delimiter=',', names=True)
    measured = np.column_stack([actual['model_x_cm'], actual['model_y_cm'], actual['model_z_cm']])
    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    axes[0].plot(draw[:, 0], draw[:, 1], 'o-')
    axes[0].set(title='Extracted triangle / HTTP offsets', xlabel='X offset (cm)', ylabel='Y offset (cm)')
    axes[1].plot(targets[:, 0], targets[:, 1], 'o-', label='Requested model endpoint')
    axes[1].plot(measured[:, 0], measured[:, 1], 'o-', label='Measured joints -> model endpoint')
    axes[1].set(title='Same model frame: top view', xlabel='X (cm)', ylabel='Y (cm)')
    axes[1].legend(fontsize=7)
    axes[2].plot(range(1, 8), targets[:, 2], 'o-', label='Requested Z')
    axes[2].plot(range(1, 8), measured[:, 2], 'o-', label='Feedback-derived Z')
    axes[2].set(title='Height at each point', xlabel='Point', ylabel='Z (cm)')
    axes[2].legend(fontsize=7)
    for ax in axes:
        ax.grid(alpha=.25)
    for ax in axes[:2]:
        ax.set_aspect('equal', adjustable='datalim')
    fig.tight_layout()
    fig.savefig(out/'comparison.png', dpi=160)
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
