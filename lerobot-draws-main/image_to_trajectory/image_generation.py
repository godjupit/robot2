import cv2
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Optional
import json
from dataclasses import dataclass
from pathlib import Path
import requests
from PIL import Image
import io
import os
import base64
from dotenv import load_dotenv

# Load environment variables
load_dotenv()
style_suffix = "A black-and-white minimalistic vector-style line drawing of a dog. Only clean, simple outlines, no shading, no color, no background. Style similar to icons or coloring book pages. High-contrast, on a plain white background, with bold black strokes"
@dataclass
class Point:
    x: float
    y: float

@dataclass
class DrawingPath:
    points: List[Point]
    pen_down: bool = True

@dataclass
class ProcessingParams:
    canny_low: int = 50
    canny_high: int = 150
    gaussian_blur: int = 3
    simplification_tolerance: float = 2.0
    workspace_size: int = 200  # mm
    min_contour_length: int = 10

class ImageProcessor:
    def __init__(self, params: ProcessingParams = None):
        self.params = params or ProcessingParams()
        self.original_image = None
        self.processed_paths = []
        
    def load_image(self, image_path: str) -> np.ndarray:
        """Load image from file path"""
        self.original_image = cv2.imread(image_path)
        if self.original_image is None:
            raise ValueError(f"Could not load image from {image_path}")
        return self.original_image
    
    def load_image_from_url(self, url: str) -> np.ndarray:
        """Load image from URL (for API-generated images)"""
        response = requests.get(url)
        image = Image.open(io.BytesIO(response.content))
        self.original_image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        return self.original_image
    
    def load_image_from_array(self, img_array: np.ndarray) -> np.ndarray:
        """Load image from numpy array"""
        self.original_image = img_array
        return self.original_image
    
    def preprocess_image(self) -> np.ndarray:
        """Convert to grayscale and apply Gaussian blur"""
        if self.original_image is None:
            raise ValueError("No image loaded")
        
        # Convert to grayscale
        gray = cv2.cvtColor(self.original_image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (self.params.gaussian_blur, self.params.gaussian_blur), 0)
        
        return blurred
    
    def detect_edges(self, image: np.ndarray) -> np.ndarray:
        """Apply Canny edge detection"""
        edges = cv2.Canny(image, self.params.canny_low, self.params.canny_high)
        return edges
    
    def find_contours(self, edges: np.ndarray) -> List[np.ndarray]:
        """Find contours from edge image"""
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by minimum length
        filtered_contours = []
        for contour in contours:
            if len(contour) >= self.params.min_contour_length:
                filtered_contours.append(contour)
        
        return filtered_contours
    
    def simplify_contour(self, contour: np.ndarray) -> np.ndarray:
        """Simplify contour using Douglas-Peucker algorithm"""
        epsilon = self.params.simplification_tolerance
        simplified = cv2.approxPolyDP(contour, epsilon, closed=False)
        return simplified
    
    def contour_to_points(self, contour: np.ndarray) -> List[Point]:
        """Convert OpenCV contour to list of Points"""
        points = []
        for point in contour:
            x, y = point[0]
            points.append(Point(float(x), float(y)))
        return points
    
    def scale_to_workspace(self, paths: List[DrawingPath]) -> List[DrawingPath]:
        """Scale paths to robot workspace coordinates"""
        if not paths or self.original_image is None:
            return paths
        
        height, width = self.original_image.shape[:2]
        
        # Calculate scaling factors
        scale_x = self.params.workspace_size / width
        scale_y = self.params.workspace_size / height
        
        scaled_paths = []
        for path in paths:
            scaled_points = []
            for point in path.points:
                # Scale and flip Y axis (robot coordinates)
                scaled_x = round(point.x * scale_x, 1)
                scaled_y = round((height - point.y) * scale_y, 1)
                scaled_points.append(Point(scaled_x, scaled_y))
            
            scaled_paths.append(DrawingPath(scaled_points, path.pen_down))
        
        return scaled_paths
    
    def process_image(self) -> List[DrawingPath]:
        """Main processing pipeline"""
        if self.original_image is None:
            raise ValueError("No image loaded")
        
        # Step 1: Preprocess
        preprocessed = self.preprocess_image()
        
        # Step 2: Edge detection
        edges = self.detect_edges(preprocessed)
        
        # Step 3: Find contours
        contours = self.find_contours(edges)
        
        # Step 4: Convert to drawing paths
        paths = []
        for contour in contours:
            # Simplify contour
            simplified = self.simplify_contour(contour)
            
            # Convert to points
            points = self.contour_to_points(simplified)
            
            if len(points) >= 2:  # Only keep paths with at least 2 points
                paths.append(DrawingPath(points))
        
        # Step 5: Scale to workspace
        scaled_paths = self.scale_to_workspace(paths)
        
        self.processed_paths = scaled_paths
        return scaled_paths
    
    def visualize_results(self, save_path: Optional[str] = None):
        """Visualize processing results"""
        if self.original_image is None:
            raise ValueError("No image loaded")
        
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # Original image
        axes[0, 0].imshow(cv2.cvtColor(self.original_image, cv2.COLOR_BGR2RGB))
        axes[0, 0].set_title('Original Image')
        axes[0, 0].axis('off')
        
        # Edge detection
        preprocessed = self.preprocess_image()
        edges = self.detect_edges(preprocessed)
        axes[0, 1].imshow(edges, cmap='gray')
        axes[0, 1].set_title('Edge Detection')
        axes[0, 1].axis('off')
        
        # Contours
        contour_img = np.zeros_like(self.original_image)
        contours = self.find_contours(edges)
        cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 2)
        axes[1, 0].imshow(cv2.cvtColor(contour_img, cv2.COLOR_BGR2RGB))
        axes[1, 0].set_title('Detected Contours')
        axes[1, 0].axis('off')
        
        # Final drawing paths
        axes[1, 1].set_xlim(0, self.params.workspace_size)
        axes[1, 1].set_ylim(0, self.params.workspace_size)
        axes[1, 1].set_aspect('equal')
        axes[1, 1].set_title('Robot Drawing Paths')
        
        for i, path in enumerate(self.processed_paths):
            if len(path.points) >= 2:
                x_coords = [p.x for p in path.points]
                y_coords = [p.y for p in path.points]
                axes[1, 1].plot(x_coords, y_coords, 'b-', linewidth=2)
                # Mark start point
                axes[1, 1].plot(x_coords[0], y_coords[0], 'go', markersize=6)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
        
        plt.show()
    
    def get_stats(self) -> dict:
        """Get processing statistics"""
        if not self.processed_paths:
            return {}
        
        total_points = sum(len(path.points) for path in self.processed_paths)
        total_distance = self.calculate_total_distance()
        estimated_time = self.estimate_drawing_time()
        
        return {
            'num_paths': len(self.processed_paths),
            'total_points': total_points,
            'avg_points_per_path': total_points / len(self.processed_paths) if self.processed_paths else 0,
            'total_distance_mm': total_distance,
            'estimated_time_seconds': estimated_time,
            'workspace_size_mm': self.params.workspace_size
        }
    
    def calculate_total_distance(self) -> float:
        """Calculate total drawing distance"""
        total_distance = 0
        for path in self.processed_paths:
            for i in range(1, len(path.points)):
                dx = path.points[i].x - path.points[i-1].x
                dy = path.points[i].y - path.points[i-1].y
                total_distance += np.sqrt(dx**2 + dy**2)
        return total_distance
    
    def estimate_drawing_time(self) -> float:
        """Estimate drawing time (rough approximation)"""
        distance = self.calculate_total_distance()
        # Assume 10mm/s drawing speed + 0.5s per path for pen up/down
        movement_time = distance / 10.0
        pen_time = len(self.processed_paths) * 0.5
        return movement_time + pen_time

class RobotCommandGenerator:
    """Generate robot commands from drawing paths"""
    
    def __init__(self, paths: List[DrawingPath]):
        self.paths = paths
    
    def generate_commands(self) -> List[str]:
        """Generate robot movement commands"""
        commands = []
        commands.append("# Robot drawing commands")
        commands.append("# Generated from image processing pipeline")
        commands.append("")
        
        for i, path in enumerate(self.paths):
            if len(path.points) < 2:
                continue
            
            commands.append(f"# Path {i + 1}")
            
            # Move to start position
            start = path.points[0]
            commands.append(f"MOVE {start.x} {start.y} 0")
            
            # Pen down
            if path.pen_down:
                commands.append("PEN_DOWN")
            
            # Draw path
            for point in path.points[1:]:
                commands.append(f"DRAW {point.x} {point.y} 0")
            
            # Pen up
            commands.append("PEN_UP")
            commands.append("")
        
        return commands
    
    def save_commands(self, filename: str):
        """Save commands to file"""
        commands = self.generate_commands()
        
        # Create output directory if it doesn't exist
        output_dir = Path(filename).parent
        output_dir.mkdir(parents=True, exist_ok=True)
        
        with open(filename, 'w') as f:
            f.write('\n'.join(commands))
    
    def to_json(self, filename: str = None) -> str:
        """Export paths as JSON for API integration"""
        data = {
            'paths': [
                {
                    'points': [{'x': p.x, 'y': p.y} for p in path.points],
                    'pen_down': path.pen_down
                }
                for path in self.paths
            ]
        }
        
        json_str = json.dumps(data, indent=2)
        
        if filename:
            # Create output directory if it doesn't exist
            output_dir = Path(filename).parent
            output_dir.mkdir(parents=True, exist_ok=True)
            
            with open(filename, 'w') as f:
                f.write(json_str)
        
        return json_str

class OpenAIImageGenerator:
    """Helper class for generating images via OpenAI API"""
    
    def __init__(self, api_key: str = None):
        from openai import OpenAI
        
        # Load API key from environment if not provided
        if api_key is None:
            api_key = os.getenv('OPENAI_API_KEY')
            if not api_key:
                raise ValueError("OpenAI API key not found. Set OPENAI_API_KEY environment variable or pass api_key parameter.")
        
        self.client = OpenAI(api_key=api_key)
    
    def generate_drawable_image(self, prompt: str, 
                              style_suffix:str) -> np.ndarray:
        """Generate an image optimized for robot drawing using OpenAI"""
        
        # Enhance prompt for drawable content
        enhanced_prompt = f"{prompt}, {style_suffix}"
        
        try:
            response = self.client.responses.create(
                model="gpt-4.1-mini",
                input=enhanced_prompt,
                tools=[{
                    "type": "image_generation",
                    "quality": "low",  # Faster generation, sufficient for line art
                    "size": "1024x1024",  # Square is fastest
                    "background": "opaque"  # White background for better edge detection
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
                import base64
                image_bytes = base64.b64decode(image_data)
                
                # Convert to numpy array
                image = Image.open(io.BytesIO(image_bytes))
                
                # Print the revised prompt for debugging
                if revised_prompt:
                    print(f"OpenAI revised prompt: {revised_prompt}")
                
                return cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
            else:
                raise Exception("No image data in response")
                
        except Exception as e:
            raise Exception(f"OpenAI API call failed: {str(e)}")
    
    def generate_with_streaming(self, prompt: str, 
                               style_suffix: str) -> np.ndarray:
        """Generate image with streaming for real-time feedback"""
        
        enhanced_prompt = f"{prompt}, {style_suffix}"
        
        try:
            stream = self.client.responses.create(
                model="gpt-4.1",
                input=enhanced_prompt,
                stream=True,
                tools=[{
                    "type": "image_generation", 
                    "partial_images": 2,  # Show 2 partial images during generation
                    "quality": "low",
                    "size": "1024x1024"
                }],
            )
            
            partial_count = 0
            final_image = None
            
            for event in stream:
                if event.type == "response.image_generation_call.partial_image":
                    partial_count += 1
                    idx = event.partial_image_index
                    image_base64 = event.partial_image_b64
                    
                    print(f"Received partial image {idx + 1}")
                    
                    # Optionally save partial images for debugging
                    image_bytes = base64.b64decode(image_base64)
                    with open(f"partial_{idx + 1}.png", "wb") as f:
                        f.write(image_bytes)
                
                elif hasattr(event, 'type') and 'image_generation_call' in event.type:
                    # Final image
                    if hasattr(event, 'result'):
                        final_image = event.result
            
            if final_image:
                image_bytes = base64.b64decode(final_image)
                image = Image.open(io.BytesIO(image_bytes))
                return cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
            else:
                raise Exception("No final image received from stream")
                
        except Exception as e:
            raise Exception(f"Streaming generation failed: {str(e)}")
    
    def generate_test_prompts(self) -> List[str]:
        """Get test prompts optimized for robot drawing"""
        return [
            "a simple house with a triangular roof and square body",
            "a basic cat sitting, outline only",
            "a tree with simple branches and leaves, line art style",
            "a coffee cup with steam lines, simple drawing",
            "a bicycle with two wheels and handlebars, technical drawing style",
            "a sun with rays and a simple smiley face, children's drawing style",
            "a flower with 5 petals and a stem, botanical line drawing",
            "a car viewed from the side, basic shape, blueprint style",
            "a bird flying with outstretched wings, simple silhouette",
            "a mountain with a simple peak and clouds, minimalist landscape",
            "a fish with fins and tail, simple outline drawing",
            "a butterfly with symmetrical wings, line art",
            "a sailboat on water with simple waves, nautical sketch",
            "a clock face with numbers and hands, technical illustration",
            "a geometric pattern with circles and triangles, mandala style"
        ]
    
    def generate_collaborative_story_image(self, story_element: str, previous_elements: List[str] = None) -> np.ndarray:
        """Generate image that builds on a collaborative story"""
        
        # Build context from previous story elements
        context = ""
        if previous_elements:
            context = f"Building on the story with these elements: {', '.join(previous_elements)}. "
        
        # Create a prompt that emphasizes simple, drawable elements
        prompt = f"{context}Draw {story_element} in a simple, clean line art style suitable for robot drawing"
                
        return self.generate_drawable_image(prompt, style_suffix)
    

# Example usage with real OpenAI image generation
def main():
    """Example usage of the pipeline with OpenAI image generation"""
    
    # Initialize processor with custom parameters
    params = ProcessingParams(
        canny_low=50,
        canny_high=150,
        simplification_tolerance=2.0,
        workspace_size=200
    )
    
    processor = ImageProcessor(params)
    
    image_generator = OpenAIImageGenerator()
    
    try:
        print("Generating image with OpenAI...")
        
        # Generate a drawable image
        test_prompts = image_generator.generate_test_prompts()
        selected_prompt = "a simple house with a door and windows"

        
        print(f"Using prompt: {selected_prompt}")
        
        # Generate image using OpenAI API
        generated_image = image_generator.generate_drawable_image(selected_prompt)
        
        # Load the generated image into processor
        processor.load_image_from_array(generated_image)
        
        print("Processing image for robot drawing...")
        
        # Process the image
        paths = processor.process_image()
        
        # Get statistics
        stats = processor.get_stats()
        print("\nProcessing Stats:")
        for key, value in stats.items():
            print(f"  {key}: {value}")
        
        # Generate robot commands
        command_gen = RobotCommandGenerator(paths)
        commands = command_gen.generate_commands()
        
        print(f"\nGenerated {len(commands)} robot commands")
        print("First 10 commands:")
        for cmd in commands[:10]:
            print(f"  {cmd}")
        
        # Save outputs
        command_gen.save_commands("outputs/robot_commands.txt")
        
        # Export as JSON for API integration
        command_gen.to_json("outputs/drawing_paths.json")
        
        # Save the original generated image
        cv2.imwrite("outputs/generated_image.png", generated_image)
        
        # Visualize results
        processor.visualize_results("outputs/processing_results.png")
        
        print("\nFiles saved:")
        print("  - outputs/generated_image.png (original OpenAI generated image)")
        print("  - outputs/robot_commands.txt (robot movement commands)")
        print("  - outputs/drawing_paths.json (JSON format for API)")
        print("  - outputs/processing_results.png (visualization)")
        
    except Exception as e:
        print(f"Error: {e}")
        print("\nFalling back to test image...")
        
        # Fallback to test image if OpenAI fails
        test_image = np.zeros((300, 300, 3), dtype=np.uint8)
        cv2.rectangle(test_image, (50, 50), (150, 150), (255, 255, 255), 2)
        cv2.circle(test_image, (200, 200), 50, (255, 255, 255), 2)
        processor.load_image_from_array(test_image)
        
        paths = processor.process_image()
        stats = processor.get_stats()
        print("Processing Stats (test image):")
        for key, value in stats.items():
            print(f"  {key}: {value}")


if __name__ == "__main__":
    main()