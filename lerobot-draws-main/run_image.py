"""Local image input for the original image converter and Phospho executor."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path

import cv2
import requests

from image_to_trajectory.api import ImageToPathConverter, PipelineConfig
from phospho_api import robot_control

ROOT = Path(__file__).resolve().parent


def verify_drawing_start_pose(port, pose, config):
    """Read-only check; never home, change calibration, or move the gripper."""
    for key in ('servos_offsets', 'servos_offsets_signs', 'servos_calibration_position'):
        if pose['calibration'].get(key) != config.get(key):
            raise RuntimeError('Calibration changed since the drawing start pose was saved.')
    response = requests.post(f'http://127.0.0.1:{port}/joints/read?robot_id=0',
                             json={'unit': 'rad', 'source': 'robot'}, timeout=5)
    response.raise_for_status()
    feedback = response.json()
    angles = feedback.get('angles', [])
    if (feedback.get('unit') != 'rad' or len(angles) != 6
            or any(v is None or not math.isfinite(v) for v in angles)):
        raise RuntimeError('Invalid real joint feedback; drawing was not started.')
    errors = [abs(math.degrees(angles[i] - pose['angles'][i])) for i in range(5)]
    if max(errors) > 3:
        raise RuntimeError(f'Arm is away from saved drawing start pose (joint errors {errors} degrees). '
                           'No automatic return or drawing motion was sent.')
    # Preserve the existing service reference frame. Reading must succeed before
    # any /move/absolute call, since that endpoint otherwise auto-initializes.
    response = requests.post(f'http://127.0.0.1:{port}/end-effector/read?robot_id=0',
                             json={'sync': True, 'only_gripper': False}, timeout=5)
    response.raise_for_status()
    reference = response.json()
    if any(reference.get(k) is None or not math.isfinite(reference[k]) for k in ('x', 'y', 'z')):
        raise RuntimeError('Phospho reference frame is unavailable; no motion was sent.')
    print('Saved drawing start pose verified; skipping /move/init. '
          'Existing Phospho XYZ reference frame is retained.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--port', type=int, default=80)
    parser.add_argument('--size-cm', type=float, default=10.0,
                        help='Longest drawing dimension in cm (default: 10)')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--start-pose', type=Path, default=ROOT/'drawing_start_pose.json',
                        help='Saved drawing start posture; verified without automatic homing')
    args = parser.parse_args()
    if not 0 < args.size_cm <= 15:
        parser.error('--size-cm must be greater than 0 and at most 15')
    start_pose = json.loads(args.start_pose.read_text()) if args.start_pose.exists() else None
    image = cv2.imread(str(args.image))
    if image is None:
        parser.error(f'Cannot read image: {args.image}')
    output = ROOT / 'outputs' / args.image.stem
    output.mkdir(parents=True, exist_ok=True)
    converter = ImageToPathConverter(PipelineConfig())
    paths = [asdict(path) for path in converter.process_image(image)]
    if not paths:
        parser.error('No drawable contours found')
    drawing_file = output / 'drawing_paths.json'
    drawing_file.write_text(json.dumps({'paths': paths}, indent=2) + '\n')
    converter.visualize_processing_steps(str(output / 'processing.png'))
    robot_control.SCALE_FACTOR = robot_control.analyze_and_scale_paths(paths)
    robot_control.SCALE_FACTOR *= args.size_cm / 15.0
    print(f'Actual drawing size: {args.size_cm:g} cm on the longest dimension', flush=True)
    commands = ([{'action': 'verify_start_pose', 'file': str(args.start_pose.resolve()),
                  'motion_sent': False, 'data': {}}] if start_pose else
                [{'endpoint': 'init', 'data': {}}])
    for path in paths:
        coordinates = robot_control.convert_2d_to_robot_coords_FIXED(
            path['points'], 2.0 if path['pen_down'] else 5.0)
        commands.append({'endpoint': 'absolute', 'data': {**coordinates[0], 'z': 5.0}})
        commands.extend({'endpoint': 'absolute', 'data': point} for point in coordinates)
    commands.append({'endpoint': 'absolute', 'data': {'x': 0, 'y': 0, 'z': 10, 'rz': 0}})
    (output / 'http_commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    print(f'{len(paths)} paths, {sum(len(p["points"]) for p in paths)} points; output: {output}', flush=True)
    if not args.execute:
        print('Preview only; no motion requests sent.')
        return
    robot_control.PI_PORT = args.port
    config_response = requests.post(
        f'http://127.0.0.1:{args.port}/robot/config?robot_id=0', timeout=10)
    config_response.raise_for_status()
    config = config_response.json().get('config')
    if config is None:
        parser.error('Phospho robot calibration is missing. Calibrate in the dashboard before execution; no motion requests sent.')
    # The upstream file forgot to import requests. Inject the checked transport
    # without changing its coordinate mapping, heights, gripper, or timings.
    def checked_request(endpoint, data=None):
        response = requests.post(f'http://127.0.0.1:{args.port}/move/{endpoint}',
                                 json={} if data is None else data, timeout=30)
        response.raise_for_status()
        result = response.json()
        (output / 'last_response.json').write_text(json.dumps(result, indent=2))
        if isinstance(result, dict) and result.get('status') == 'error':
            raise RuntimeError(f'Phospho rejected {endpoint}: {result}')
        return result
    robot_control.call_to_api = checked_request
    if start_pose:
        verify_drawing_start_pose(args.port, start_pose, config)
    else:
        if args.start_pose != ROOT/'drawing_start_pose.json':
            parser.error(f'Start pose file not found: {args.start_pose}; no motion requests sent.')
        checked_request('init')
    robot_control.time.sleep(3)
    for path in paths:
        robot_control.move_robot_along_path_FIXED(path)
    checked_request('absolute', {'x': 0, 'y': 0, 'z': 10, 'rz': 0})
    print('Original Phospho command sequence completed.')


if __name__ == '__main__':
    main()
