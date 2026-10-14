#!/usr/bin/env python

"""
Script to test and understand SO101 arm kinematics and coordinate frames.
Based on the actual URDF structure with joints 1-6.

Usage:
python check_kinematics.py --robot_port /dev/tty.usbmodem58760431541 --urdf_path /path/to/robot.urdf
"""

import argparse
import logging
import time
import sys
import os
import numpy as np

# Add pyroki_snippets to Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
pyroki_snippets_path = os.path.join(current_dir, "pyroki", "examples")
if pyroki_snippets_path not in sys.path:
    sys.path.insert(0, pyroki_snippets_path)

import pyroki as pk
from yourdfpy import URDF
import pyroki_snippets as pks

from lerobot.common.robots.so101_follower import SO101Follower
from lerobot.common.robots.so101_follower.config_so101_follower import SO101FollowerConfig

logger = logging.getLogger(__name__)


class SO101KinematicsChecker:
    """Test kinematics and coordinate frames for SO101 based on actual URDF."""
    
    def __init__(self, urdf_path: str, target_link_name: str = "gripper"):
        self.urdf = URDF.load(urdf_path)
        self.robot = pk.Robot.from_urdf(self.urdf)
        self.target_link_name = target_link_name
        
        # Joint mapping based on URDF analysis
        self.joint_mapping = {
            "1": "shoulder_pan.pos",   # Base rotation
            "2": "shoulder_lift.pos",  # Shoulder pitch 
            "3": "elbow_flex.pos",     # Elbow flex
            "4": "wrist_flex.pos",     # Wrist flex
            "5": "wrist_roll.pos",     # Wrist roll
            "6": "gripper.pos",        # Gripper
        }
        
        self.print_robot_info()
        self.test_forward_kinematics()
    
    def print_robot_info(self):
        """Print comprehensive robot information from URDF."""
        print("=== SO101 ROBOT INFORMATION ===")
        print(f"End effector link: {self.target_link_name}")
        print(f"Joint names: {self.robot.joints.names}")
        print(f"Link names: {self.robot.links.names}")
        
        print("\n=== JOINT MAPPING ===")
        for urdf_joint, lerobot_joint in self.joint_mapping.items():
            print(f"URDF Joint {urdf_joint} → LeRobot {lerobot_joint}")
        
        print("\n=== COORDINATE SYSTEM (from URDF) ===")
        print("X-axis: Forward from base")
        print("Y-axis: Left from base") 
        print("Z-axis: Up from base")
        print("Default gripper orientation: Pointing down (-Z direction)")
    
    def test_forward_kinematics(self):
        """Test forward kinematics at different joint configurations."""
        print("\n=== FORWARD KINEMATICS TESTS ===")
        
        test_configs = {
            "all_zero": np.zeros(6),
            "home_position": np.array([0, -0.5, 0.8, 0.5, 0, 0]),  # Typical home pose
            "extended": np.array([0, 0, 0, 0, 0, 0]),              # Fully extended
            "folded": np.array([0, -1.5, 1.5, 1.0, 0, 0]),        # Folded back
        }
        
        for config_name, joint_angles in test_configs.items():
            try:
                fk_result = self.robot.forward_kinematics(joint_angles)
                pos = fk_result[:3, 3]  # Extract position
                print(f"{config_name:12}: Position = [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            except Exception as e:
                print(f"{config_name:12}: FK ERROR - {e}")
    
    def test_orientations_for_drawing(self, target_position: np.ndarray):
        """Test orientations specifically for drawing applications."""
        print(f"\n=== DRAWING ORIENTATIONS TEST at {target_position} ===")
        
        drawing_orientations = {
            "natural_down": np.array([1, 0, 0, 0]),        # Identity - natural gripper down
            "straight_down": np.array([0, 1, 0, 0]),       # 180° around X - straight down
            "tilt_45_down": np.array([0.924, 0.383, 0, 0]), # 45° tilt down
            "tilt_30_down": np.array([0.966, 0.259, 0, 0]), # 30° tilt down
            "roll_90_ccw": np.array([0.707, 0, 0, 0.707]),  # 90° roll counterclockwise
            "roll_90_cw": np.array([0.707, 0, 0, -0.707]),  # 90° roll clockwise
        }
        
        for name, quat in drawing_orientations.items():
            try:
                solution = pks.solve_ik(
                    robot=self.robot,
                    target_link_name=self.target_link_name,
                    target_position=target_position,
                    target_wxyz=quat,
                )
                
                if solution is not None:
                    joint_degrees = np.degrees(solution)
                    print(f"{name:15}: SUCCESS")
                    print(f"                Joints: {joint_degrees}")
                else:
                    print(f"{name:15}: NO SOLUTION")
                    
            except Exception as e:
                print(f"{name:15}: ERROR - {e}")
    
    def test_drawing_workspace(self):
        """Test reachability for typical drawing positions."""
        print("\n=== DRAWING WORKSPACE TEST ===")
        
        # Test grid of positions in front of robot
        drawing_positions = []
        
        # Create a grid in front of the robot
        x_range = np.linspace(0.15, 0.25, 3)  # 15-25cm forward
        y_range = np.linspace(-0.1, 0.1, 3)   # 10cm left to 10cm right
        z_height = 0.15                        # 15cm above base
        
        for x in x_range:
            for y in y_range:
                drawing_positions.append((f"x{x:.2f}_y{y:.2f}", np.array([x, y, z_height])))
        
        # Test with natural drawing orientation
        drawing_orientation = np.array([1, 0, 0, 0])  # Natural gripper down
        
        print("Testing 3x3 grid in front of robot:")
        reachable_count = 0
        
        for name, position in drawing_positions:
            try:
                solution = pks.solve_ik(
                    robot=self.robot,
                    target_link_name=self.target_link_name,
                    target_position=position,
                    target_wxyz=drawing_orientation,
                )
                
                if solution is not None:
                    print(f"{name:12}: REACHABLE")
                    reachable_count += 1
                else:
                    print(f"{name:12}: UNREACHABLE")
                    
            except Exception as e:
                print(f"{name:12}: ERROR")
        
        print(f"\nReachable positions: {reachable_count}/{len(drawing_positions)}")
    
    def interactive_orientation_test(self, robot: SO101Follower):
        """Interactive test of different orientations for drawing."""
        print("\n=== INTERACTIVE ORIENTATION TEST ===")
        print("Testing different orientations for drawing at a fixed position.")
        
        # Safe drawing position in front of robot
        test_position = np.array([0.0, 0.0, 0.0])
        
        orientations_to_test = [
            ("Natural down (identity)", np.array([1, 0, 0, 0])),
            # ("Straight down (180° X)", np.array([0, 1, 0, 0])),
            # ("45° tilt down", np.array([0.924, 0.383, 0, 0])),
            # ("90° roll (pen sideways)", np.array([0.707, 0, 0, 0.707])),
        ]
        
        for name, orientation in orientations_to_test:
            try:
                solution = pks.solve_ik(
                    robot=self.robot,
                    target_link_name=self.target_link_name,
                    target_position=test_position,
                    target_wxyz=orientation,
                )
                
                if solution is None:
                    print(f"Skipping {name} - No IK solution")
                    continue
                
                # Convert to LeRobot format
                joint_positions = {}
                for i, joint_name in enumerate(self.robot.joints.names):
                    if i < len(solution):
                        lerobot_name = self.joint_mapping.get(joint_name)
                        if lerobot_name:
                            joint_positions[lerobot_name] = np.degrees(solution[i])
                # set all joint positions to 0.0 except gripper
                # for joint in self.joint_mapping.values():
                    # if joint != "gripper.pos":
                        # joint_positions[joint] = 0.0
                joint_positions["gripper.pos"] = 45.0  # Half open for visibility
                
                print(f"\n=== {name} ===")
                print(f"Position: {test_position}")
                print(f"Quaternion (wxyz): {orientation}")
                print(f"Joint angles (degrees): {joint_positions}")
                
                response = input("Move robot to this orientation? (y/n/q): ").lower()
                if response == 'q':
                    break
                elif response == 'y':
                    print("Moving robot...")
                    self.move_robot_smooth(robot, joint_positions, duration=3.0)
                    input("Observe the end-effector orientation. Press Enter to continue...")
                    
            except Exception as e:
                print(f"Error testing {name}: {e}")
    
    def move_robot_smooth(self, robot: SO101Follower, target_joints: dict, duration: float = 3.0):
        """Smoothly move robot to target position."""
        start_time = time.time()
        current_obs = robot.get_observation()
        start_joints = {key: current_obs[key] for key in target_joints if key in current_obs}
        
        while time.time() - start_time < duration:
            t = (time.time() - start_time) / duration
            t = min(1.0, t)
            t_smooth = 3 * t**2 - 2 * t**3  # Smooth interpolation
            
            current_action = {}
            for joint, target_pos in target_joints.items():
                if joint in start_joints:
                    start_pos = start_joints[joint]
                    current_pos = start_pos + (target_pos - start_pos) * t_smooth
                    current_action[joint] = current_pos
                else:
                    current_action[joint] = target_pos
            
            robot.send_action(current_action)
            time.sleep(0.05)
    
    def find_optimal_drawing_orientation(self):
        """Find the best orientation for drawing based on workspace coverage."""
        print("\n=== FINDING OPTIMAL DRAWING ORIENTATION ===")
        
        orientations_to_test = {
            "natural": np.array([1, 0, 0, 0]),
            "straight_down": np.array([0, 1, 0, 0]),
            "tilt_30": np.array([0.966, 0.259, 0, 0]),
            "tilt_45": np.array([0.924, 0.383, 0, 0]),
        }
        
        # Test positions in drawing area
        test_positions = []
        for x in np.linspace(0.15, 0.25, 5):
            for y in np.linspace(-0.08, 0.08, 5):
                test_positions.append(np.array([x, y, 0.15]))
        
        best_orientation = None
        best_success_rate = 0
        
        for orient_name, orientation in orientations_to_test.items():
            successes = 0
            
            for position in test_positions:
                try:
                    solution = pks.solve_ik(
                        robot=self.robot,
                        target_link_name=self.target_link_name,
                        target_position=position,
                        target_wxyz=orientation,
                    )
                    if solution is not None:
                        successes += 1
                except:
                    pass
            
            success_rate = successes / len(test_positions)
            print(f"{orient_name:15}: {success_rate:.1%} reachable ({successes}/{len(test_positions)})")
            
            if success_rate > best_success_rate:
                best_success_rate = success_rate
                best_orientation = (orient_name, orientation)
        
        if best_orientation:
            print(f"\nRECOMMENDED: Use '{best_orientation[0]}' orientation")
            print(f"Quaternion (wxyz): {best_orientation[1]}")
        else:
            print("\nNo optimal orientation found - check robot workspace")


def main():
    parser = argparse.ArgumentParser(description="Test SO101 kinematics for drawing applications")
    parser.add_argument("--robot_port", required=True, help="Robot serial port")
    parser.add_argument("--urdf_path", required=True, help="Path to robot URDF file")
    parser.add_argument("--target_link", default="gripper", help="Target link name for IK")
    parser.add_argument("--test_robot", action="store_true", help="Test with real robot movement")
    
    args = parser.parse_args()
    
    # Setup logging
    logging.basicConfig(level=logging.INFO)
    
    # Initialize kinematics checker
    try:
        checker = SO101KinematicsChecker(args.urdf_path, args.target_link)
    except Exception as e:
        logger.error(f"Failed to initialize kinematics checker: {e}")
        return
    
    # Test orientations for drawing
    drawing_position = np.array([0.2, 0.0, 0.15])  # 20cm forward, center, 15cm up
    checker.test_orientations_for_drawing(drawing_position)
    
    # Test drawing workspace
    checker.test_drawing_workspace()
    
    # Find optimal orientation
    checker.find_optimal_drawing_orientation()
    
    # Interactive robot testing if requested
    if args.test_robot:
        robot_config = SO101FollowerConfig(port=args.robot_port)
        robot = SO101Follower(robot_config)
        
        try:
            logger.info("Connecting to robot...")
            robot.connect()
            logger.info("Robot connected successfully")
            
            checker.interactive_orientation_test(robot)
            
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        except Exception as e:
            logger.error(f"Error during robot testing: {e}")
        finally:
            logger.info("Disconnecting robot...")
            robot.disconnect()
    
    print("\n=== SUMMARY ===")
    print("1. Use the orientation with highest workspace coverage")
    print("2. Test with --test_robot to see actual end-effector orientations")
    print("3. Choose the orientation that points the gripper downward for drawing")


if __name__ == "__main__":
    main()