import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Store clicked points with timestamps
clicked_points = []

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        distance = depth_frame.get_distance(x, y)
        clicked_points.append({
            'x': x,
            'y': y,
            'distance': distance,
            'time': time.time()
        })
        print(f'Depth at ({x}, {y}): {distance:.3f} meters')

# Configure depth and color streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipe = rs.pipeline()
profile = pipe.start(config)

# Create align object to align depth frames to color frames
align = rs.align(rs.stream.color)

# Register mouse callback
cv2.namedWindow('RealSense Color')
cv2.namedWindow('RealSense Depth')
cv2.setMouseCallback('RealSense Color', mouse_callback)
cv2.setMouseCallback('RealSense Depth', mouse_callback)

try:
    while True:
        frames = pipe.wait_for_frames()
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        
        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue
            
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Draw circles and text for points less than 5 seconds old
        current_time = time.time()
        clicked_points[:] = [p for p in clicked_points if current_time - p['time'] < 5.0]
        
        for point in clicked_points:
            # Draw on color image
            cv2.circle(color_image, (point['x'], point['y']), 4, (0, 0, 255), -1)
            cv2.putText(color_image, f"({point['x']},{point['y']}) {point['distance']:.3f}m", 
                       (point['x']+10, point['y']), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # Draw on depth image
            cv2.circle(depth_colormap, (point['x'], point['y']), 4, (0, 0, 255), -1)
            cv2.putText(depth_colormap, f"({point['x']},{point['y']}) {point['distance']:.3f}m", 
                       (point['x']+10, point['y']), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Show images
        cv2.imshow('RealSense Color', color_image)
        cv2.imshow('RealSense Depth', depth_colormap)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()
