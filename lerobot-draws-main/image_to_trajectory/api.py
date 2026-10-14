"""
Drawing Pipeline Component for LeRobot SO-100

Clean, modular component that converts text prompts to robot drawing paths.
Perfect for integration with voice commands and collaborative art systems.

Usage:
    pipeline = DrawingPipeline()
    result = pipeline.prompt_to_robot_paths("draw a simple house")
    print(result.summary)
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Optional, Dict, Any
import json
from dataclasses import dataclass, asdict
from pathlib import Path
import os
from PIL import Image
import io
import base64
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

style_suffix = "A black-and-white minimalistic vector-style line drawing. Only clean, simple outlines, no shading, no color, no background. Style similar to icons or coloring book pages. High-contrast, on a plain white background, with bold black strokes"

@dataclass
class Point:
    """2D point coordinates"""
    x: float
    y: float

@dataclass
class DrawingPath:
    """Series of connected points for drawing"""
    points: List[Point]
    pen_down: bool = True

@dataclass
class PipelineConfig:
    """Configuration for the drawing pipeline"""
    # Image processing parameters
    canny_low: int = 50
    canny_high: int = 150
    gaussian_blur: int = 3
    simplification_tolerance: float = 2.0
    min_contour_length: int = 10
    
    # Robot workspace parameters
    workspace_size_mm: int = 200
    
    # SO-100 specific parameters
    paper_center_x: float = 200.0  # mm from robot base
    paper_center_y: float = 0.0    # mm from robot base
    paper_z: float = 100.0         # mm table height
    paper_width: float = 210.0     # A4 width
    paper_height: float = 297.0    # A4 height
    
    # Motion parameters
    travel_speed: float = 80.0     # mm/s
    draw_speed: float = 25.0       # mm/s
    pen_lift_height: float = 10.0  # mm

@dataclass
class RobotAction:
    """Single robot action in trajectory"""
    timestamp: float
    position: Dict[str, float]  # {'x': 0.2, 'y': 0.0, 'z': 0.1} in meters
    orientation: Dict[str, float]  # {'rx': 0.0, 'ry': 0.0, 'rz': 0.0} in radians
    pen_down: bool
    speed: float  # m/s
    action_type: str

@dataclass
class PipelineResult:
    """Complete result from the drawing pipeline"""
    success: bool
    prompt: str
    revised_prompt: Optional[str]
    drawing_paths: List[DrawingPath]
    robot_actions: List[RobotAction]
    stats: Dict[str, Any]
    error_message: Optional[str] = None
    
    @property
    def summary(self) -> str:
        """Human-readable summary of the result"""
        if not self.success:
            return f"Pipeline failed: {self.error_message}"
        
        return (f"Generated {self.stats['num_paths']} drawing paths with "
               f"{self.stats['total_points']} points. "
               f"Estimated drawing time: {self.stats['estimated_time']:.1f}s")

class OpenAIImageGenerator:
    """Generate images optimized for robot drawing"""
    
    def __init__(self, api_key: Optional[str] = None):
        try:
            from openai import OpenAI
            
            if api_key is None:
                api_key = os.getenv('OPENAI_API_KEY')
                if not api_key:
                    raise ValueError("OpenAI API key required. Set OPENAI_API_KEY env var.")
            
            self.client = OpenAI(api_key=api_key)
        except ImportError:
            raise ImportError("OpenAI package required: pip install openai")
    
    def generate(self, prompt: str) -> tuple[np.ndarray, Optional[str]]:
        """Generate drawable image from text prompt"""
        
        # Optimize prompt for robot drawing
        enhanced_prompt = f"{prompt}, {style_suffix}"
        
        try:
            response = self.client.responses.create(
                model="gpt-4.1-mini",
                input=enhanced_prompt,
                tools=[{
                    "type": "image_generation",
                    "quality": "low",  # Fast generation for line art
                    "size": "1024x1024",
                    "background": "opaque"
                }],
            )
            
            # Extract image data and revised prompt
            image_data = None
            revised_prompt = None
            
            for output in response.output:
                if output.type == "image_generation_call":
                    image_data = output.result
                    revised_prompt = getattr(output, 'revised_prompt', None)
                    break
            
            if image_data:
                image_bytes = base64.b64decode(image_data)
                image = Image.open(io.BytesIO(image_bytes))
                opencv_image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
                return opencv_image, revised_prompt
            else:
                raise Exception("No image data in response")
                
        except Exception as e:
            raise Exception(f"Image generation failed: {str(e)}")

class ImageToPathConverter:
    """Convert images to drawing paths"""
    
    def __init__(self, config: PipelineConfig):
        self.config = config
        # Store intermediate results for visualization
        self.last_image = None
        self.last_gray = None
        self.last_blurred = None
        self.last_edges = None
        self.last_contours = None
    
    def process_image(self, image: np.ndarray) -> List[DrawingPath]:
        """Convert image to drawing paths"""
        
        self.last_image = image.copy()
        
        # Step 1: Preprocess
        self.last_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.last_blurred = cv2.GaussianBlur(self.last_gray, (self.config.gaussian_blur, self.config.gaussian_blur), 0)
        
        # Step 2: Edge detection
        self.last_edges = cv2.Canny(self.last_blurred, self.config.canny_low, self.config.canny_high)
        
        # Step 3: Find contours
        self.last_contours, _ = cv2.findContours(self.last_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Step 4: Convert to drawing paths
        paths = []
        height, width = image.shape[:2]
        
        for contour in self.last_contours:
            if len(contour) < self.config.min_contour_length:
                continue
            
            # Simplify contour
            epsilon = self.config.simplification_tolerance
            simplified = cv2.approxPolyDP(contour, epsilon, closed=False)
            
            # Convert to scaled points
            points = []
            for point in simplified:
                x, y = point[0]
                # Scale to workspace and flip Y axis
                scaled_x = (x / width) * self.config.workspace_size_mm
                scaled_y = ((height - y) / height) * self.config.workspace_size_mm
                points.append(Point(scaled_x, scaled_y))
            
            if len(points) >= 2:
                paths.append(DrawingPath(points))
        
        return paths
    
    def visualize_processing_steps(self, save_path: str):
        """Create visualization of all processing steps"""
        if self.last_image is None:
            raise ValueError("No image processed yet")
        
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        
        # Original image
        axes[0, 0].imshow(cv2.cvtColor(self.last_image, cv2.COLOR_BGR2RGB))
        axes[0, 0].set_title('1. Original Image', fontsize=14, fontweight='bold')
        axes[0, 0].axis('off')
        
        # Grayscale
        axes[0, 1].imshow(self.last_gray, cmap='gray')
        axes[0, 1].set_title('2. Grayscale', fontsize=14, fontweight='bold')
        axes[0, 1].axis('off')
        
        # Blurred
        axes[0, 2].imshow(self.last_blurred, cmap='gray')
        axes[0, 2].set_title('3. Gaussian Blur', fontsize=14, fontweight='bold')
        axes[0, 2].axis('off')
        
        # Edge detection
        axes[1, 0].imshow(self.last_edges, cmap='gray')
        axes[1, 0].set_title('4. Edge Detection (Canny)', fontsize=14, fontweight='bold')
        axes[1, 0].axis('off')
        
        # Contours
        contour_img = np.zeros_like(self.last_image)
        if self.last_contours:
            cv2.drawContours(contour_img, self.last_contours, -1, (0, 255, 0), 2)
        axes[1, 1].imshow(cv2.cvtColor(contour_img, cv2.COLOR_BGR2RGB))
        axes[1, 1].set_title('5. Detected Contours', fontsize=14, fontweight='bold')
        axes[1, 1].axis('off')
        
        # Processing info
        axes[1, 2].text(0.1, 0.9, 'Processing Parameters:', fontsize=12, fontweight='bold', transform=axes[1, 2].transAxes)
        axes[1, 2].text(0.1, 0.8, f'Canny Low: {self.config.canny_low}', fontsize=10, transform=axes[1, 2].transAxes)
        axes[1, 2].text(0.1, 0.7, f'Canny High: {self.config.canny_high}', fontsize=10, transform=axes[1, 2].transAxes)
        axes[1, 2].text(0.1, 0.6, f'Gaussian Blur: {self.config.gaussian_blur}', fontsize=10, transform=axes[1, 2].transAxes)
        axes[1, 2].text(0.1, 0.5, f'Simplification: {self.config.simplification_tolerance}', fontsize=10, transform=axes[1, 2].transAxes)
        axes[1, 2].text(0.1, 0.4, f'Min Contour Length: {self.config.min_contour_length}', fontsize=10, transform=axes[1, 2].transAxes)
        axes[1, 2].text(0.1, 0.3, f'Workspace Size: {self.config.workspace_size_mm}mm', fontsize=10, transform=axes[1, 2].transAxes)
        
        if self.last_contours:
            axes[1, 2].text(0.1, 0.1, f'Contours Found: {len(self.last_contours)}', fontsize=10, fontweight='bold', transform=axes[1, 2].transAxes)
        
        axes[1, 2].set_xlim(0, 1)
        axes[1, 2].set_ylim(0, 1)
        axes[1, 2].axis('off')
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        plt.close()  # Close to free memory

class PathToRobotConverter:
    """Convert 2D drawing paths to 3D robot actions"""
    
    def __init__(self, config: PipelineConfig):
        self.config = config
        self.last_paths = None
        self.last_actions = None
    
    def paths_to_actions(self, paths: List[DrawingPath]) -> List[RobotAction]:
        """Convert drawing paths to robot actions"""
        
        self.last_paths = paths
        actions = []
        current_time = 0.0
        
        # Start position (safe above paper)
        start_pos = {
            'x': self.config.paper_center_x / 1000.0,  # Convert mm to m
            'y': self.config.paper_center_y / 1000.0,
            'z': (self.config.paper_z + 50.0) / 1000.0
        }
        
        actions.append(RobotAction(
            timestamp=current_time,
            position=start_pos,
            orientation={'rx': 0.0, 'ry': 0.0, 'rz': 0.0},
            pen_down=False,
            speed=self.config.travel_speed / 1000.0,
            action_type='move_to_start'
        ))
        
        for path_idx, path in enumerate(paths):
            if len(path.points) < 2:
                continue
            
            # Convert image coordinates to robot coordinates
            robot_points = []
            for point in path.points:
                # Map from workspace to paper coordinates
                norm_x = point.x / self.config.workspace_size_mm
                norm_y = point.y / self.config.workspace_size_mm
                
                # Scale to paper size (maintain aspect ratio)
                paper_aspect = self.config.paper_width / self.config.paper_height
                workspace_aspect = 1.0  # Square workspace
                
                if workspace_aspect > paper_aspect:
                    # Fit to paper width
                    scaled_width = self.config.paper_width
                    scaled_height = self.config.paper_width / workspace_aspect
                else:
                    # Fit to paper height
                    scaled_height = self.config.paper_height
                    scaled_width = self.config.paper_height * workspace_aspect
                
                # Convert to robot coordinates (in meters)
                robot_x = (self.config.paper_center_x + (norm_x - 0.5) * scaled_width) / 1000.0
                robot_y = (self.config.paper_center_y + (norm_y - 0.5) * scaled_height) / 1000.0
                robot_points.append((robot_x, robot_y))
            
            # Move to path start (pen up)
            start_x, start_y = robot_points[0]
            travel_z = (self.config.paper_z + self.config.pen_lift_height) / 1000.0
            
            # Calculate travel time
            if actions:
                last_pos = actions[-1].position
                travel_dist = np.sqrt((start_x - last_pos['x'])**2 + (start_y - last_pos['y'])**2)
                travel_time = travel_dist / (self.config.travel_speed / 1000.0)
                current_time += travel_time
            
            actions.append(RobotAction(
                timestamp=current_time,
                position={'x': start_x, 'y': start_y, 'z': travel_z},
                orientation={'rx': 0.0, 'ry': 0.0, 'rz': 0.0},
                pen_down=False,
                speed=self.config.travel_speed / 1000.0,
                action_type=f'move_to_path_{path_idx}'
            ))
            
            # Lower pen to drawing height
            draw_z = self.config.paper_z / 1000.0
            pen_down_time = self.config.pen_lift_height / self.config.travel_speed
            current_time += pen_down_time
            
            actions.append(RobotAction(
                timestamp=current_time,
                position={'x': start_x, 'y': start_y, 'z': draw_z},
                orientation={'rx': 0.0, 'ry': 0.0, 'rz': 0.0},
                pen_down=True,
                speed=self.config.travel_speed / 1000.0 * 0.3,
                action_type='pen_down'
            ))
            
            # Draw the path
            for i, (robot_x, robot_y) in enumerate(robot_points[1:], 1):
                last_pos = actions[-1].position
                draw_dist = np.sqrt((robot_x - last_pos['x'])**2 + (robot_y - last_pos['y'])**2)
                draw_time = draw_dist / (self.config.draw_speed / 1000.0)
                current_time += draw_time
                
                actions.append(RobotAction(
                    timestamp=current_time,
                    position={'x': robot_x, 'y': robot_y, 'z': draw_z},
                    orientation={'rx': 0.0, 'ry': 0.0, 'rz': 0.0},
                    pen_down=True,
                    speed=self.config.draw_speed / 1000.0,
                    action_type=f'draw_line_{i}'
                ))
            
            # Lift pen
            last_pos = actions[-1].position
            pen_up_time = self.config.pen_lift_height / self.config.travel_speed
            current_time += pen_up_time
            
            actions.append(RobotAction(
                timestamp=current_time,
                position={'x': last_pos['x'], 'y': last_pos['y'], 'z': travel_z},
                orientation={'rx': 0.0, 'ry': 0.0, 'rz': 0.0},
                pen_down=False,
                speed=self.config.travel_speed / 1000.0 * 0.3,
                action_type='pen_up'
            ))
        
        self.last_actions = actions
        return actions
    
    def visualize_robot_trajectory(self, save_path: str):
        """Visualize complete robot trajectory in 3D and 2D views"""
        if not self.last_actions:
            raise ValueError("No robot actions generated yet")
        
        fig = plt.figure(figsize=(20, 12))
        
        # Separate travel and drawing actions
        travel_actions = [a for a in self.last_actions if not a.pen_down]
        draw_actions = [a for a in self.last_actions if a.pen_down]
        
        # 3D trajectory plot
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        
        if travel_actions:
            travel_x = [a.position['x'] * 1000 for a in travel_actions]  # Convert to mm for display
            travel_y = [a.position['y'] * 1000 for a in travel_actions]
            travel_z = [a.position['z'] * 1000 for a in travel_actions]
            ax1.plot(travel_x, travel_y, travel_z, 'b--', alpha=0.6, linewidth=2, label='Travel (pen up)')
        
        if draw_actions:
            draw_x = [a.position['x'] * 1000 for a in draw_actions]
            draw_y = [a.position['y'] * 1000 for a in draw_actions]
            draw_z = [a.position['z'] * 1000 for a in draw_actions]
            ax1.plot(draw_x, draw_y, draw_z, 'r-', linewidth=3, label='Drawing (pen down)')
        
        # Mark start and end points
        start_pos = self.last_actions[0].position
        end_pos = self.last_actions[-1].position
        ax1.scatter([start_pos['x'] * 1000], [start_pos['y'] * 1000], [start_pos['z'] * 1000], 
                   c='green', s=100, label='Start')
        ax1.scatter([end_pos['x'] * 1000], [end_pos['y'] * 1000], [end_pos['z'] * 1000], 
                   c='red', s=100, label='End')
        
        ax1.set_xlabel('X (mm)')
        ax1.set_ylabel('Y (mm)')
        ax1.set_zlabel('Z (mm)')
        ax1.set_title('3D Robot Trajectory', fontsize=14, fontweight='bold')
        ax1.legend()
        
        # Top-down view (drawing plane)
        ax2 = fig.add_subplot(2, 3, 2)
        if draw_actions:
            ax2.plot(draw_x, draw_y, 'r-', linewidth=3, label='Drawing path')
        if travel_actions:
            ax2.plot(travel_x, travel_y, 'b--', alpha=0.6, linewidth=2, label='Travel path')
        
        # Show paper bounds
        paper_x = self.config.paper_center_x
        paper_y = self.config.paper_center_y
        paper_w = self.config.paper_width / 2
        paper_h = self.config.paper_height / 2
        
        paper_rect = plt.Rectangle(
            (paper_x - paper_w, paper_y - paper_h),
            self.config.paper_width, self.config.paper_height,
            fill=False, edgecolor='black', linestyle=':', linewidth=2, label='Paper bounds (A4)'
        )
        ax2.add_patch(paper_rect)
        
        ax2.set_xlabel('X (mm)')
        ax2.set_ylabel('Y (mm)')
        ax2.set_title('Top View - Drawing Plane', fontsize=14, fontweight='bold')
        ax2.axis('equal')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Timeline plots
        ax3 = fig.add_subplot(2, 3, 3)
        times = [a.timestamp for a in self.last_actions]
        speeds = [a.speed * 1000 for a in self.last_actions]  # Convert to mm/s
        pen_states = [1 if a.pen_down else 0 for a in self.last_actions]
        
        ax3.plot(times, speeds, 'g-', linewidth=2, label='Speed (mm/s)')
        ax3.fill_between(times, 0, [ps * max(speeds) for ps in pen_states], 
                        alpha=0.3, color='red', label='Pen down periods')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Speed (mm/s)')
        ax3.set_title('Speed & Pen State Timeline', fontsize=14, fontweight='bold')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 2D Drawing paths (workspace coordinates)
        ax4 = fig.add_subplot(2, 3, 4)
        if self.last_paths:
            for i, path in enumerate(self.last_paths):
                if len(path.points) >= 2:
                    x_coords = [p.x for p in path.points]
                    y_coords = [p.y for p in path.points]
                    ax4.plot(x_coords, y_coords, 'b-', linewidth=2, alpha=0.8)
                    # Mark start point
                    ax4.plot(x_coords[0], y_coords[0], 'go', markersize=6)
                    # Mark end point
                    ax4.plot(x_coords[-1], y_coords[-1], 'ro', markersize=4)
        
        ax4.set_xlim(0, self.config.workspace_size_mm)
        ax4.set_ylim(0, self.config.workspace_size_mm)
        ax4.set_aspect('equal')
        ax4.set_xlabel('X (mm)')
        ax4.set_ylabel('Y (mm)')
        ax4.set_title('2D Drawing Paths (Workspace)', fontsize=14, fontweight='bold')
        ax4.grid(True, alpha=0.3)
        
        # Statistics and info
        ax5 = fig.add_subplot(2, 3, 5)
        ax5.text(0.05, 0.95, 'Trajectory Statistics:', fontsize=12, fontweight='bold', transform=ax5.transAxes)
        ax5.text(0.05, 0.85, f'Total Actions: {len(self.last_actions)}', fontsize=10, transform=ax5.transAxes)
        ax5.text(0.05, 0.80, f'Drawing Actions: {len(draw_actions)}', fontsize=10, transform=ax5.transAxes)
        ax5.text(0.05, 0.75, f'Travel Actions: {len(travel_actions)}', fontsize=10, transform=ax5.transAxes)
        ax5.text(0.05, 0.70, f'Total Time: {self.last_actions[-1].timestamp:.1f}s', fontsize=10, transform=ax5.transAxes)
        
        # Calculate distances
        total_draw_dist = 0
        total_travel_dist = 0
        for i in range(1, len(self.last_actions)):
            curr = self.last_actions[i].position
            prev = self.last_actions[i-1].position
            dist = np.sqrt((curr['x'] - prev['x'])**2 + (curr['y'] - prev['y'])**2) * 1000  # mm
            
            if self.last_actions[i].pen_down:
                total_draw_dist += dist
            else:
                total_travel_dist += dist
        
        ax5.text(0.05, 0.60, f'Drawing Distance: {total_draw_dist:.1f}mm', fontsize=10, transform=ax5.transAxes)
        ax5.text(0.05, 0.55, f'Travel Distance: {total_travel_dist:.1f}mm', fontsize=10, transform=ax5.transAxes)
        ax5.text(0.05, 0.50, f'Total Distance: {total_draw_dist + total_travel_dist:.1f}mm', fontsize=10, transform=ax5.transAxes)
        
        ax5.text(0.05, 0.35, 'Robot Configuration:', fontsize=12, fontweight='bold', transform=ax5.transAxes)
        ax5.text(0.05, 0.30, f'Paper Center: ({self.config.paper_center_x}, {self.config.paper_center_y})mm', fontsize=10, transform=ax5.transAxes)
        ax5.text(0.05, 0.25, f'Paper Height: {self.config.paper_z}mm', fontsize=10, transform=ax5.transAxes)
        ax5.text(0.05, 0.20, f'Travel Speed: {self.config.travel_speed}mm/s', fontsize=10, transform=ax5.transAxes)
        ax5.text(0.05, 0.15, f'Draw Speed: {self.config.draw_speed}mm/s', fontsize=10, transform=ax5.transAxes)
        ax5.text(0.05, 0.10, f'Pen Lift: {self.config.pen_lift_height}mm', fontsize=10, transform=ax5.transAxes)
        
        ax5.set_xlim(0, 1)
        ax5.set_ylim(0, 1)
        ax5.axis('off')
        
        # Z-height profile
        ax6 = fig.add_subplot(2, 3, 6)
        z_heights = [a.position['z'] * 1000 for a in self.last_actions]  # Convert to mm
        pen_colors = ['red' if a.pen_down else 'blue' for a in self.last_actions]
        
        for i in range(len(times)-1):
            ax6.plot([times[i], times[i+1]], [z_heights[i], z_heights[i+1]], 
                    color=pen_colors[i], linewidth=2, alpha=0.8)
        
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Z Height (mm)')
        ax6.set_title('Z-Height Profile', fontsize=14, fontweight='bold')
        ax6.grid(True, alpha=0.3)
        
        # Add legend for Z-height colors
        from matplotlib.lines import Line2D
        legend_elements = [Line2D([0], [0], color='red', lw=2, label='Pen down'),
                          Line2D([0], [0], color='blue', lw=2, label='Pen up')]
        ax6.legend(handles=legend_elements)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        plt.close()  # Close to free memory((robot_x - last_pos['x'])**2 + (robot_y - last_pos['y'])**2)
        

class DrawingPipeline:
    """Main pipeline: prompt → image → paths → robot actions"""
    
    def __init__(self, config: Optional[PipelineConfig] = None, 
                 openai_api_key: Optional[str] = None):
        self.config = config or PipelineConfig()
        self.image_generator = OpenAIImageGenerator(openai_api_key)
        self.image_processor = ImageToPathConverter(self.config)
        self.robot_converter = PathToRobotConverter(self.config)
    
    def prompt_to_robot_paths(self, prompt: str, save_outputs: bool = True, 
                            output_dir: str = "outputs") -> PipelineResult:
        """
        Convert text prompt to robot drawing paths
        
        Args:
            prompt: Text description of what to draw
            save_outputs: Whether to save output files
            output_dir: Directory to save outputs
            
        Returns:
            PipelineResult with paths and robot actions
        """
        
        try:
            # Step 1: Generate image
            print(f"Generating image for: '{prompt}'")
            image, revised_prompt = self.image_generator.generate(prompt)
            
            # Step 2: Convert image to drawing paths
            print("Converting image to drawing paths...")
            drawing_paths = self.image_processor.process_image(image)
            
            # Step 3: Convert paths to robot actions
            print("Generating robot trajectory...")
            robot_actions = self.robot_converter.paths_to_actions(drawing_paths)
            
            # Step 4: Calculate statistics
            stats = self._calculate_stats(drawing_paths, robot_actions)
            
            # Step 5: Save outputs if requested
            if save_outputs:
                self._save_outputs(prompt, image, drawing_paths, robot_actions, 
                                 stats, output_dir, revised_prompt)
            
            return PipelineResult(
                success=True,
                prompt=prompt,
                revised_prompt=revised_prompt,
                drawing_paths=drawing_paths,
                robot_actions=robot_actions,
                stats=stats
            )
            
        except Exception as e:
            return PipelineResult(
                success=False,
                prompt=prompt,
                revised_prompt=None,
                drawing_paths=[],
                robot_actions=[],
                stats={},
                error_message=str(e)
            )
    
    def _calculate_stats(self, paths: List[DrawingPath], actions: List[RobotAction]) -> Dict[str, Any]:
        """Calculate pipeline statistics"""
        
        total_points = sum(len(path.points) for path in paths)
        
        # Calculate total drawing distance
        total_distance = 0.0
        for path in paths:
            for i in range(1, len(path.points)):
                dx = path.points[i].x - path.points[i-1].x
                dy = path.points[i].y - path.points[i-1].y
                total_distance += np.sqrt(dx**2 + dy**2)
        
        # Get execution time from robot actions
        estimated_time = actions[-1].timestamp if actions else 0.0
        
        return {
            'num_paths': len(paths),
            'total_points': total_points,
            'total_distance_mm': total_distance,
            'estimated_time': estimated_time,
            'num_robot_actions': len(actions),
            'drawing_actions': len([a for a in actions if a.pen_down]),
            'travel_actions': len([a for a in actions if not a.pen_down])
        }
    
    def _save_outputs(self, prompt: str, image: np.ndarray, paths: List[DrawingPath], 
                     actions: List[RobotAction], stats: Dict[str, Any], 
                     output_dir: str, revised_prompt: Optional[str]):
        """Save all pipeline outputs including visualizations"""
        
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Create safe filename from prompt
        safe_name = "".join(c for c in prompt if c.isalnum() or c in (' ', '-', '_')).rstrip()
        safe_name = safe_name.replace(' ', '_')[:30]
        
        # Save generated image
        cv2.imwrite(str(output_path / f"{safe_name}_image.png"), image)
        
        # Save drawing paths as JSON
        paths_data = {
            'metadata': {
                'prompt': prompt,
                'revised_prompt': revised_prompt,
                'stats': stats,
                'config': asdict(self.config)
            },
            'paths': [
                {
                    'points': [{'x': p.x, 'y': p.y} for p in path.points],
                    'pen_down': path.pen_down
                }
                for path in paths
            ]
        }
        
        with open(output_path / f"{safe_name}_paths.json", 'w') as f:
            json.dump(paths_data, f, indent=2)
        
        # Save robot actions in LeRobot format
        robot_data = {
            'metadata': {
                'robot_type': 'SO-100',
                'task': 'drawing',
                'prompt': prompt,
                'revised_prompt': revised_prompt,
                'stats': stats
            },
            'actions': [asdict(action) for action in actions]
        }
        
        with open(output_path / f"{safe_name}_robot.json", 'w') as f:
            json.dump(robot_data, f, indent=2)
        
        # Save simple robot commands (text format)
        commands = self._generate_simple_commands(paths)
        with open(output_path / f"{safe_name}_commands.txt", 'w') as f:
            f.write('\n'.join(commands))
        
        # Generate and save visualizations
        try:
            # Image processing steps visualization
            self.image_processor.visualize_processing_steps(
                str(output_path / f"{safe_name}_processing.png")
            )
            
            # Robot trajectory visualization
            self.robot_converter.visualize_robot_trajectory(
                str(output_path / f"{safe_name}_trajectory.png")
            )
            
            print(f"\nOutputs saved to {output_dir}/:")
            print(f"  - {safe_name}_image.png (generated image)")
            print(f"  - {safe_name}_paths.json (2D drawing paths)")
            print(f"  - {safe_name}_robot.json (SO-100 robot actions)")
            print(f"  - {safe_name}_commands.txt (simple text commands)")
            print(f"  - {safe_name}_processing.png (image processing steps)")
            print(f"  - {safe_name}_trajectory.png (robot trajectory visualization)")
            
        except Exception as e:
            print(f"Warning: Could not generate visualizations: {e}")
            print(f"\nOutputs saved to {output_dir}/:")
            print(f"  - {safe_name}_image.png (generated image)")
            print(f"  - {safe_name}_paths.json (2D drawing paths)")
            print(f"  - {safe_name}_robot.json (SO-100 robot actions)")
            print(f"  - {safe_name}_commands.txt (simple text commands)")
    
    def _generate_simple_commands(self, paths: List[DrawingPath]) -> List[str]:
        """Generate simple text commands for debugging"""
        
        commands = [
            "# Robot drawing commands",
            f"# Generated from prompt",
            ""
        ]
        
        for i, path in enumerate(paths):
            if len(path.points) < 2:
                continue
            
            commands.append(f"# Path {i + 1}")
            start = path.points[0]
            commands.append(f"MOVE {start.x:.1f} {start.y:.1f} 0")
            commands.append("PEN_DOWN")
            
            for point in path.points[1:]:
                commands.append(f"DRAW {point.x:.1f} {point.y:.1f} 0")
            
            commands.append("PEN_UP")
            commands.append("")
        
        return commands


# Convenience functions for easy integration
def quick_draw(prompt: str, output_dir: str = "outputs") -> PipelineResult:
    """Quick function to generate drawing from prompt"""
    pipeline = DrawingPipeline()
    return pipeline.prompt_to_robot_paths(prompt, output_dir=output_dir)



def main_function(imagePrompt: str): 
     # Simple example
    result = quick_draw(imagePrompt)
    print(result.summary)
    
    if result.success:
        print(f"\nGenerated {len(result.robot_actions)} robot actions")
        print("First 5 actions:")
        for action in result.robot_actions[:5]:
            pos = action.position
            print(f"  {action.action_type}: ({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f}), pen={'down' if action.pen_down else 'up'}")
    


# Example usage and testing
if __name__ == "__main__":
    # Simple example
    result = quick_draw("a clock face with numbers and hands, technical illustration")
    print(result.summary)
    
    if result.success:
        print(f"\nGenerated {len(result.robot_actions)} robot actions")
        print("First 5 actions:")
        for action in result.robot_actions[:5]:
            pos = action.position
            print(f"  {action.action_type}: ({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f}), pen={'down' if action.pen_down else 'up'}")
            
            
