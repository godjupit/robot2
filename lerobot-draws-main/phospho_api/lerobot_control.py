#!/usr/bin/env python

"""
Script to move SO101 follower robot through drawing path points using PyRoki IK solver.

Example usage:
python draw_path.py \
    --robot_port /dev/tty.usbmodem58760431541 \
    --camera_index 0 \
    --drawing_file drawing_paths.json \
    --workspace_width 200 \
    --workspace_height 200 \
    --base_height 0.2 \
    --urdf_path /path/to/robot.urdf
"""

import json
import logging
import time
import argparse
import sys
import os
from typing import Dict, List, Tuple
import numpy as np

# Add pyroki_snippets to Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
pyroki_snippets_path = os.path.join(current_dir, "pyroki", "examples")
if pyroki_snippets_path not in sys.path:
    sys.path.insert(0, pyroki_snippets_path)

# PyRoki imports for IK
import pyroki as pk
from yourdfpy import URDF
import pyroki_snippets as pks

from lerobot.common.robots.so101_follower import SO101Follower
from lerobot.common.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.common.cameras.opencv.configuration_opencv import OpenCVCameraConfig

logger = logging.getLogger(__name__)


def load_drawing_paths(file_path: str) -> List[Dict]:
    """Load drawing paths from JSON file."""
    with open(file_path, 'r') as f:
        data = json.load(f)
    return data['paths']


def pixel_to_robot_coords(x: float, y: float, workspace_width: float, workspace_height: float, 
                         robot_reach: float = 0.3) -> Tuple[float, float]:
    """
    Convert pixel coordinates to robot workspace coordinates.
    
    Args:
        x, y: Pixel coordinates
        workspace_width, workspace_height: Size of the drawing workspace in pixels
        robot_reach: Maximum reach of the robot in meters
    
    Returns:
        Tuple of (x, y) coordinates in robot workspace
    """
    # Normalize to 0-1 range
    norm_x = x / workspace_width
    norm_y = y / workspace_height
    
    # Convert to robot coordinates - place drawing area in front of robot
    # X: left-right relative to robot
    robot_x = (norm_x - 0.5) * robot_reach * 0.8  # Scale down to 80% of reach
    # Y: forward from robot base (positive Y is forward)
    robot_y = 0.1 + norm_y * robot_reach * 0.6  # Start 10cm forward, extend 60% of reach
    
    return robot_x, robot_y


class RobotController:
    """Robot controller with PyRoki IK integration."""
    
    def __init__(self, urdf_path: str, target_link_name: str = "gripper"):
        """
        Initialize robot controller with URDF and IK solver.
        
        Args:
            urdf_path: Path to robot URDF file
            target_link_name: Name of the target link for IK (usually end-effector)
        """
        self.urdf = URDF.load(urdf_path)
        self.robot = pk.Robot.from_urdf(self.urdf)
        self.target_link_name = target_link_name
        self.previous_joints = None
        
    def solve_ik(self, target_position: np.ndarray, target_orientation: np.ndarray = None) -> Dict[str, float]:
        """
        Solve inverse kinematics using PyRoki.
        
        Args:
            target_position: Target position [x, y, z] in meters
            target_orientation: Target orientation as quaternion [w, x, y, z] (optional)
        
        Returns:
            Dictionary of joint angles in degrees
        """
        # Default orientation if not provided (pointing down)
        if target_orientation is None:
            target_orientation = np.array([0, 1, 0, 0])  # wxyz format - pointing down
        
        try:
            # Convert previous joints to radians for seed
            seed = None
            if self.previous_joints:
                seed_values = []
                for joint in self.robot.joints.names:
                    mapped_name = self._map_joint_name(joint)
                    if mapped_name and mapped_name in self.previous_joints:
                        seed_values.append(np.radians(self.previous_joints[mapped_name]))
                    else:
                        seed_values.append(0.0)  # Default value if joint not found
                seed = np.array(seed_values) if seed_values else None
            
            # Solve IK using PyRoki
            solution = pks.solve_ik(
                robot=self.robot,
                target_link_name=self.target_link_name,
                target_position=target_position,
                target_wxyz=target_orientation,
            )
            
            if solution is None:
                logger.warning("IK solver returned no solution")
                return None
            
            logger.info(f"Raw IK solution (radians): {solution}")
            logger.info(f"Joint names: {self.robot.joints.names}")
            
            # Convert joint names and values to match LeRobot API format
            joint_positions = {}
            joint_names = self.robot.joints.names  # Fix: use joints, not links
            
            for i, joint_name in enumerate(joint_names):
                if i < len(solution):
                    # Convert from radians to degrees and map to LeRobot joint names
                    solution_degrees = np.degrees(solution[i])
                    lerobot_joint_name = self._map_joint_name(joint_name)
                    if lerobot_joint_name:
                        joint_positions[lerobot_joint_name] = solution_degrees
            
            # Store for next iteration
            self.previous_joints = joint_positions.copy()
            return joint_positions
            
        except Exception as e:
            logger.error(f"IK solution failed: {e}")
            return None
    
    def _map_joint_name(self, pyroki_joint_name: str) -> str:
        """
        Map PyRoki joint names to LeRobot joint names.
        Adjust this mapping based on your robot's joint naming convention.
        """
        joint_mapping = {
            "6": "shoulder_pan.pos",
            "5": "shoulder_lift.pos", 
            "4": "elbow_flex.pos",
            "3": "wrist_flex.pos",
            "2": "wrist_roll.pos",
            "1": "gripper.pos",
        }
        return joint_mapping.get(pyroki_joint_name, None)


def move_to_position(robot: SO101Follower, target_joints: Dict[str, float], duration: float = 2.0):
    """
    Smoothly move robot to target joint positions.
    
    Args:
        robot: SO101Follower instance
        target_joints: Dictionary of target joint positions
        duration: Time to complete the movement
    """
    start_time = time.time()
    
    # Get current position
    current_obs = robot.get_observation()
    start_joints = {key: current_obs[key] for key in target_joints if key in current_obs}
    
    while time.time() - start_time < duration:
        # Calculate interpolation factor
        t = (time.time() - start_time) / duration
        t = min(1.0, t)
        
        # Smooth interpolation (ease in/out)
        t_smooth = 3 * t**2 - 2 * t**3
        
        # Interpolate joint positions
        current_action = {}
        for joint, target_pos in target_joints.items():
            if joint in start_joints:
                start_pos = start_joints[joint]
                current_pos = start_pos + (target_pos - start_pos) * t_smooth
                current_action[joint] = current_pos
            else:
                current_action[joint] = target_pos
        
        # Send action to robot
        robot.send_action(current_action)
        logger.debug(f"Sent action: {current_action}")
        time.sleep(0.05)  # 50Hz control loop


def execute_drawing_path(robot: SO101Follower, robot_controller: RobotController, paths: List[Dict], 
                        workspace_width: float, workspace_height: float, base_height: float):
    """
    Execute the drawing paths on the robot using PyRoki IK solver.
    
    Args:
        robot: SO101Follower instance
        robot_controller: RobotController instance with IK solver
        paths: List of path dictionaries from JSON
        workspace_width: Width of drawing workspace in pixels
        workspace_height: Height of drawing workspace in pixels
        base_height: Height above robot base to draw at
    """
    logger.info(f"Starting to execute {len(paths)} drawing paths using PyRoki IK")
    
    for path_idx, path in enumerate(paths):
        logger.info(f"Executing path {path_idx + 1}/{len(paths)}")
        points = path['points']
        pen_down = path.get('pen_down', True)
        
        for point_idx, point in enumerate(points):
            # Convert pixel coordinates to robot workspace
            robot_x, robot_y = pixel_to_robot_coords(
                point['x'], point['y'], workspace_width, workspace_height
            )
            
            # Create target position for IK solver
            target_position = np.array([robot_x, robot_y, base_height])
            
            # Solve IK using PyRoki
            target_joints = robot_controller.solve_ik(target_position)
            
            if target_joints is None:
                logger.error(f"IK failed for position {target_position}")
                continue
            
            logger.info(f"IK solution: {target_joints}")
            
            # Adjust gripper based on pen state
            if pen_down:
                target_joints["gripper.pos"] = 90.0  # More closed for drawing
            else:
                target_joints["gripper.pos"] = 10.0  # Open for moving
            
            logger.info(f"Moving to point {point_idx + 1}/{len(points)}: "
                       f"pixel({point['x']:.1f}, {point['y']:.1f}) -> "
                       f"robot({robot_x:.3f}, {robot_y:.3f}, {base_height:.3f})")
            
            # Move to position
            move_duration = 1.5 if point_idx == 0 else 1.0  # Slower for first point
            move_to_position(robot, target_joints, move_duration)
            # Brief pause at each point
            time.sleep(0.1)
        
        logger.info(f"Completed path {path_idx + 1}")
        time.sleep(0.5)  # Pause between paths


def main():
    parser = argparse.ArgumentParser(description="Execute drawing paths on SO101 robot with PyRoki IK")
    parser.add_argument("--robot_port", required=True, help="Robot serial port")
    parser.add_argument("--urdf_path", required=True, help="Path to robot URDF file")
    parser.add_argument("--target_link", default="gripper", help="Target link name for IK")
    parser.add_argument("--camera_index", type=int, default=0, help="Camera index")
    parser.add_argument("--drawing_file", default="drawing_paths.json", help="JSON file with drawing paths")
    parser.add_argument("--workspace_width", type=float, default=200, help="Drawing workspace width in pixels")
    parser.add_argument("--workspace_height", type=float, default=200, help="Drawing workspace height in pixels")
    parser.add_argument("--base_height", type=float, default=0.15, help="Height above robot base to draw at")
    parser.add_argument("--robot_id", default="drawing_robot", help="Robot ID")
    
    args = parser.parse_args()
    
    # Setup logging
    logging.basicConfig(level=logging.INFO)
    
    # Load drawing paths
    paths = load_drawing_paths(args.drawing_file)
    logger.info(f"Loaded {len(paths)} drawing paths")
    
    # Initialize robot controller with PyRoki IK
    try:
        robot_controller = RobotController(args.urdf_path, args.target_link)
        logger.info("PyRoki IK solver initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize PyRoki IK solver: {e}")
        return
    
    # Configure robot
    robot_config = SO101FollowerConfig(
        port=args.robot_port
    )
    
    # Create and connect robot
    robot = SO101Follower(robot_config)
    
    try:
        logger.info("Connecting to robot...")
        robot.connect()
        logger.info("Robot connected successfully")
        
        # Execute drawing paths with PyRoki IK
        execute_drawing_path(robot, robot_controller, paths, args.workspace_width, 
                           args.workspace_height, args.base_height)
        
        logger.info("Drawing completed successfully!")
        
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Error during execution: {e}")
    finally:
        logger.info("Disconnecting robot...")
        robot.disconnect()
        logger.info("Robot disconnected")


if __name__ == "__main__":
    main()