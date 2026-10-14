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
from typing import List, Dict

# --- Configuration ---
PI_IP: str = "127.0.0.1"
PI_PORT: int = 80

# CRITICAL TUNING PARAMETER: Time in seconds for the robot to move between each small step.
# Start with a larger value (like 0.5s) and decrease it slowly to find the sweet spot
# between speed and accuracy for your robot.
TIME_PER_STEP = 0.5

# This will be set by the analysis function
SCALE_FACTOR = 1.0

# --- Helper Functions (Mostly Unchanged) ---

def call_to_api(endpoint: str, data: dict = {}) -> dict:
    """Sends a command to a /move/ endpoint."""
    try:
        url = f"http://{PI_IP}:{PI_PORT}/move/{endpoint}"
        response = requests.post(url, json=data)
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"API call to {url} failed: {e}")
        return {}

def load_drawing_paths(file_path: str) -> List[Dict]:
    """Loads drawing path data from a JSON file."""
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
        return data.get('paths', [])
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Error loading {file_path}: {e}")
        return []

def analyze_and_scale_paths(paths: List[Dict]) -> float:
    """Analyze path data and suggest optimal scale factor."""
    if not paths: return 1.0
    
    all_x, all_y = [], []
    for path in paths:
        if 'points' in path:
            for point in path['points']:
                all_x.append(point['x'])
                all_y.append(point['y'])
    
    if not all_x or not all_y: return 1.0
    
    x_range = max(all_x) - min(all_x)
    y_range = max(all_y) - min(all_y)
    max_range = max(x_range, y_range)
    
    # Let's target a drawing size of 15cm for the largest dimension
    TARGET_DRAWING_SIZE_CM = 15.0
    
    suggested_scale = TARGET_DRAWING_SIZE_CM / max_range if max_range > 0 else 1.0
    print(f"Path data ranges (input units): X={x_range:.2f}, Y={y_range:.2f}")
    print(f"To make the drawing {TARGET_DRAWING_SIZE_CM}cm wide, the suggested scale factor is: {suggested_scale:.2f}")
    
    return suggested_scale

# --- CORE LOGIC (FIXED) ---

def convert_2d_to_robot_coords_FIXED(points: List[Dict], z_level_cm: float) -> List[Dict]:
    """
    CORRECT MAPPING: Convert 2D drawing coordinates to the robot's X-Y plane.
    """
    robot_coords = []
    
    for point in points:
        # Drawing X -> Robot Y (assuming robot's +Y is "forward")
        # Drawing Y -> Robot X (assuming robot's +X is "right")
        # NOTE: You may need to swap or negate these based on your Axis Probe Test!
        robot_point = {
            "x": point['x'] * SCALE_FACTOR,
            "y": point['y'] * SCALE_FACTOR,
            "z": z_level_cm,
            "rx": 0,  # Keep orientation fixed, pointing down
            "ry": 0,
            "rz": 0,
            "open": 0.5 # Gripper partially closed like a pen holder
        }
        robot_coords.append(robot_point)
    
    return robot_coords

def move_robot_along_path_FIXED(path: Dict):
    """
    Executes a single path with correct mapping and timing.
    """
    points = path.get('points', [])
    is_drawing_stroke = path.get('pen_down', True)
    
    if not points:
        return

    # Define Z levels for drawing and moving
    Z_DRAWING_LEVEL = 2.0  # Pen is ON the paper
    Z_MOVING_LEVEL = 5.0   # Pen is LIFTED above the paper

    z_level = Z_DRAWING_LEVEL if is_drawing_stroke else Z_MOVING_LEVEL
    
    robot_coords = convert_2d_to_robot_coords_FIXED(points, z_level_cm=z_level)

    # --- Execute the path ---
    # First, lift the pen and move to the start of the path
    start_coord = robot_coords[0].copy()
    start_coord['z'] = Z_MOVING_LEVEL # Ensure pen is up
    print(f"Moving to start of path: {start_coord}")
    call_to_api("absolute", start_coord)
    time.sleep(2) # Give it time to get to the start position

    # Now, execute the actual path (either drawing or moving)
    for i, coord in enumerate(robot_coords):
        print(f"Step {i+1}/{len(robot_coords)} -> {coord}")
        call_to_api("absolute", coord)
        
        # This corrected delay is CRITICAL for accuracy
        time.sleep(TIME_PER_STEP)

def main():
    print("Shape Path Following - Corrected Version")
    
    # 1. Initialize robot
    print("Initializing robot...")
    call_to_api("init")
    time.sleep(3) # Give it plenty of time to initialize
    
    # 2. Load and analyze paths
    # Note: Assumes your script is in a folder, and the json is in ../...
    paths = load_drawing_paths("drawing_paths.json") 
    if not paths: return
    
    global SCALE_FACTOR
    SCALE_FACTOR = analyze_and_scale_paths(paths)

    # 3. Execute paths
    for i, path in enumerate(paths):
        print(f"\n--- Executing path {i+1}/{len(paths)} ---")
        move_robot_along_path_FIXED(path)
            
    print("\n--- Drawing complete! ---")
    
    # 4. Return to a safe, neutral position
    print("Returning to neutral...")
    call_to_api("absolute", {"x": 0, "y": 0, "z": 10, "rz": 0})

if __name__ == "__main__":
    main()