import cv2
import numpy as np
import pyrealsense2 as rs
import sys
import json
from pathlib import Path
from scipy.interpolate import RBFInterpolator
import time

# Add project root to Python path
project_root = str(Path(__file__).parent.parent)
sys.path.append(project_root)

# Add constants
CALIBRATION_FILE = 'calibration_points.json'
MIN_CALIBRATION_POINTS = 4

class CalibrationManager:
    def __init__(self):
        self.points = {}
        self.load_calibration()
    
    def create_template(self):
        """Create template calibration points with instructions"""
        template_points = [
            {
                "cam": ["<x coordinate from camera view>", "<y coordinate from camera view>"],
                "real": ["<x coordinate in robot space>", "<y coordinate in robot space>"]
            },
            {
                "cam": ["example: 100", "example: 100"],
                "real": ["example: 150", "example: 150"]
            }
        ]
        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(template_points, f, indent=2)
        print("Created template file. Please fill in the actual calibration points.")
        # Remove sys.exit(1) to allow program to continue
    
    def load_calibration(self):
        try:
            with open(CALIBRATION_FILE, 'r') as f:
                points_data = json.load(f)
                if not points_data:  # File is empty
                    points_data = self.create_template()
                # Clear existing points
                self.points.clear()
                # Load points from the list of dictionaries
                for point in points_data:
                    # Skip template/example values
                    if (isinstance(point["cam"][0], str) and 
                        ("<" in point["cam"][0] or "example" in point["cam"][0])):
                        continue
                    try:
                        cam = tuple(map(float, point["cam"]))
                        real = tuple(map(float, point["real"]))
                        self.points[cam] = real
                    except (ValueError, TypeError):
                        print(f"Skipping invalid point: {point}")
                        continue
                
                if not self.points:  # If no valid points were loaded
                    print("No valid calibration points found. Running in preview mode.")
                    print(f"Please add calibration points to {CALIBRATION_FILE}")
                    # Remove sys.exit(1) to allow program to continue
                    
        except FileNotFoundError:
            print(f"No calibration file found at {CALIBRATION_FILE}, creating template...")
            self.create_template()
        except json.JSONDecodeError:
            print(f"Invalid calibration file at {CALIBRATION_FILE}, creating template...")
            self.create_template()
            
    def save_calibration(self):
        points_data = []
        for cam_point, real_point in self.points.items():
            points_data.append({
                "cam": list(cam_point),
                "real": list(real_point)
            })
        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(points_data, f, indent=2)
            
    def validate_points(self):
        """Check if there are enough calibration points."""
        return len(self.points) >= MIN_CALIBRATION_POINTS

class ThinPlateSpline:
    def __init__(self, source_points, target_points):
        self.source = np.array(source_points)
        self.target = np.array(target_points)
        
        self.interpolator_x = RBFInterpolator(
            self.source, 
            self.target[:, 0],
            kernel='thin_plate_spline',
            epsilon=0.1
        )
        self.interpolator_y = RBFInterpolator(
            self.source,
            self.target[:, 1],
            kernel='thin_plate_spline',
            epsilon=0.1
        )
    
    def transform(self, points):
        points = np.array(points)
        if points.ndim == 1:
            points = points.reshape(1, -1)
        
        x = self.interpolator_x(points)
        y = self.interpolator_y(points)
        return np.column_stack((x, y))

def draw_workspace(image, bounds, offset_x=0):
    """Draw workspace boundaries and grid"""
    x = bounds['x_fixed'] + offset_x
    y = bounds['y_fixed']
    w = bounds['box_width']
    h = bounds['box_height']
    
    # Draw workspace boundary
    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 255), 2)
    
    # Draw grid lines
    grid_spacing = 50
    for i in range(x, x + w, grid_spacing):
        cv2.line(image, (i, y), (i, y + h), (128, 128, 128), 1)
    for i in range(y, y + h, grid_spacing):
        cv2.line(image, (x, i), (x + w, i), (128, 128, 128), 1)

def draw_transformed_workspace(image, bounds, tps, offset_x=0):
    """Draw transformed workspace boundaries and grid"""
    x = bounds['x_fixed']
    y = bounds['y_fixed']
    w = bounds['box_width']
    h = bounds['box_height']
    
    # Draw transformed workspace boundary corners
    corners = [
        (x, y),
        (x + w, y),
        (x + w, y + h),
        (x, y + h)
    ]
    
    # Transform corners
    transformed_corners = []
    for corner in corners:
        transformed = tps.transform(np.array(corner))
        transformed_corners.append((int(transformed[0][0]) + offset_x, int(transformed[0][1])))
    
    # Draw transformed boundary with magenta color
    for i in range(4):
        cv2.line(image, transformed_corners[i], transformed_corners[(i+1)%4], (255, 0, 255), 2)
    
    # Draw transformed grid lines
    grid_spacing = 50
    for i in range(x, x + w, grid_spacing):
        vertical_points = [(i, j) for j in range(y, y + h, 10)]
        transformed_points = []
        for point in vertical_points:
            transformed = tps.transform(np.array(point))
            transformed_points.append((int(transformed[0][0]) + offset_x, int(transformed[0][1])))
        for j in range(len(transformed_points)-1):
            # Draw vertical lines in blue
            cv2.line(image, transformed_points[j], transformed_points[j+1], (255, 128, 0), 1)
    
    for i in range(y, y + h, grid_spacing):
        horizontal_points = [(j, i) for j in range(x, x + w, 10)]
        transformed_points = []
        for point in horizontal_points:
            transformed = tps.transform(np.array(point))
            transformed_points.append((int(transformed[0][0]) + offset_x, int(transformed[0][1])))
        for j in range(len(transformed_points)-1):
            # Draw horizontal lines in orange
            cv2.line(image, transformed_points[j], transformed_points[j+1], (0, 128, 255), 1)

def mouse_callback(event, x, y, flags, param):
    calibration = param['calibration']
    points = param['points']
    last_click = param['last_click']
    
    if event == cv2.EVENT_LBUTTONDOWN:
        current_time = time.time()
        current_pos = (x if x < 640 else x - 640, y)
        
        # Check if this is a double click (within 0.5 seconds and 5 pixels)
        if (last_click['time'] and 
            current_time - last_click['time'] < 0.5 and
            abs(current_pos[0] - last_click['pos'][0]) < 5 and
            abs(current_pos[1] - last_click['pos'][1]) < 5):
            
            # Prompt for real coordinates
            try:
                real_x = float(input("Enter real X coordinate: "))
                real_y = float(input("Enter real Y coordinate: "))
                
                # Add calibration point
                calibration.points[current_pos] = (real_x, real_y)
                calibration.save_calibration()
                
                # Update TPS if we have enough points
                if calibration.validate_points():
                    source_points = np.array(list(calibration.points.keys()))
                    target_points = np.array(list(calibration.points.values()))
                    param['tps'] = ThinPlateSpline(source_points, target_points)
                    print("Calibration updated successfully!")
                
            except ValueError:
                print("Invalid input. Point not added.")
            
            # Reset last click
            last_click['time'] = None
            last_click['pos'] = None
        else:
            # Update last click info
            last_click['time'] = current_time
            last_click['pos'] = current_pos
            points.append(current_pos)

def main():
    calibration = CalibrationManager()
    has_calibration = calibration.validate_points()
    
    if not has_calibration:
        print(f"Warning: Not enough calibration points. Need at least {MIN_CALIBRATION_POINTS}.")
        print(f"Running in preview mode without transformation.")
        print(f"Please add calibration points to {CALIBRATION_FILE}")

    # Initialize RealSense camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)

    # Create visualization window and mouse callback
    cv2.namedWindow('Interpolation Test')
    points = []
    param = {
        'points': points,
        'calibration': calibration,
        'tps': None,
        'last_click': {'time': None, 'pos': None}
    }
    cv2.setMouseCallback('Interpolation Test', mouse_callback, param)

    # Create a blank visualization image
    vis_width = 1280  # 640 * 2 for side by side
    vis_height = 480
    
    # Define workspace bounds (adjust these values based on your setup)
    workspace_bounds = {
        'x_fixed': 100,
        'y_fixed': 50,
        'box_width': 440,
        'box_height': 380
    }

    # Initialize TPS only if we have calibration points
    if has_calibration:
        source_points = np.array(list(calibration.points.keys()))
        target_points = np.array(list(calibration.points.values()))
        param['tps'] = ThinPlateSpline(source_points, target_points)

    while True:
        # Get frames
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        
        if not color_frame:
            continue

        # Get color frame
        frame = np.asanyarray(color_frame.get_data())

        # Create visualization image
        vis_image = np.zeros((vis_height, vis_width, 3), dtype=np.uint8)
        
        # Copy original frame to both sides
        vis_image[:, :640] = frame
        vis_image[:, 640:] = frame

        # Draw workspace boundaries and grid on both sides
        draw_workspace(vis_image, workspace_bounds)
        draw_workspace(vis_image, workspace_bounds, 640)

        # Draw transformed workspace only if we have calibration
        if param['tps']:
            draw_transformed_workspace(vis_image, workspace_bounds, param['tps'], 640)
            
            # Draw calibration points - reduced from 3 to 2
            for (src_x, src_y), (tgt_x, tgt_y) in calibration.points.items():
                # Draw source points on left side
                cv2.circle(vis_image, (int(src_x), int(src_y)), 1, (255, 255, 0), -1)
                # Draw target points on right side
                cv2.circle(vis_image, (int(src_x) + 640, int(src_y)), 1, (255, 255, 0), -1)

        # Draw clicked points and their interpolated positions
        for x, y in points:
            # Draw original point on left side (red) - reduced from 5 to 3
            cv2.circle(vis_image, (int(x), int(y)), 3, (0, 0, 255), -1)
            cv2.putText(vis_image, f"({x:.1f}, {y:.1f})", (int(x) + 10, int(y)),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # Transform point only if we have calibration
            if param['tps']:
                transformed = param['tps'].transform(np.array([x, y]))
                robot_x, robot_y = transformed[0]
                
                # Draw interpolated point on right side (green) - reduced from 5 to 3
                interpolated_x = int(x) + 640
                cv2.circle(vis_image, (interpolated_x, int(y)), 3, (0, 255, 0), -1)
                
                # Draw coordinate information
                text = f"TPS({robot_x:.1f}, {robot_y:.1f})"
                cv2.putText(vis_image, text, (interpolated_x + 10, int(y)),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Draw dividing line
        cv2.line(vis_image, (640, 0), (640, vis_height), (255, 255, 255), 2)

        # Add labels with more information
        cv2.putText(vis_image, "Click to add points", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(vis_image, "Double-click to add calibration point", (10, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(vis_image, "TPS Transform", (650, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Show legend
        cv2.putText(vis_image, "Press 'c' to clear points", (10, vis_height - 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(vis_image, "Yellow: Calibration Points", (10, vis_height - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(vis_image, "Red: Input Point", (10, vis_height - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(vis_image, "Green: Transformed Point", (10, vis_height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Show result
        cv2.imshow('Interpolation Test', vis_image)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            points.clear()

    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
