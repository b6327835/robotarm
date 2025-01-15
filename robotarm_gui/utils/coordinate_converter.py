import numpy as np
from scipy.interpolate import LinearNDInterpolator

class CoordinateConverter:
    # Calibration points mapping (after conversion) -> (desired position)
    # Format: (current_x, current_y): (desired_x, desired_y)
    CALIBRATION_POINTS = {
        (276, 176): (286, 178),   # Top-left region
        (278, 11): (288, 30),     # Bottom-left region
        (142, 179): (160, 180),   # Top-right region
        # Add more calibration points as needed
    }

    @classmethod
    def _initialize_interpolator(cls):
        """Initialize the interpolator with calibration points"""
        current_points = np.array(list(cls.CALIBRATION_POINTS.keys()))
        desired_points = np.array(list(cls.CALIBRATION_POINTS.values()))
        return LinearNDInterpolator(current_points, desired_points, fill_value=None)

    @classmethod
    def _correct_coordinates(cls, x, y):
        """Correct coordinates using nearest calibration point offset"""
        if x is None or y is None:
            return x, y

        # Find nearest calibration point
        min_distance = float('inf')
        nearest_current = None
        nearest_desired = None

        for current, desired in cls.CALIBRATION_POINTS.items():
            distance = ((current[0] - x) ** 2 + (current[1] - y) ** 2) ** 0.5
            if distance < min_distance:
                min_distance = distance
                nearest_current = current
                nearest_desired = desired

        if nearest_current is None or nearest_desired is None:
            return x, y

        # Apply offset from nearest point
        dx = nearest_desired[0] - nearest_current[0]
        dy = nearest_desired[1] - nearest_current[1]
        
        return x + dx, y + dy

    @staticmethod
    def to_robot_coordinates(target_x, target_y, x_fixed, y_fixed, box_width, box_height):
        """
        Convert camera coordinates to robot coordinates with correction
        
        Args:
            target_x (float): Target x coordinate in camera space
            target_y (float): Target y coordinate in camera space
            x_fixed (float): Fixed x coordinate of workspace
            y_fixed (float): Fixed y coordinate of workspace
            box_width (float): Width of workspace
            box_height (float): Height of workspace
            
        Returns:
            tuple: (x, y) coordinates in robot space
        """
        if any(v is None for v in [target_x, target_y, x_fixed, y_fixed, box_width, box_height]):
            return None, None

        try:
            y_relative = (target_y - y_fixed) / box_height
            x_relative = (target_x - x_fixed) / box_width

            tar_x = (169 - (y_relative * 169)) - 4  # Changed from 135 to 169
            tar_y = (193 - (x_relative * 193)) - 0  # Changed from 145 to 193
            
            if tar_x < 0:
                tar_x = 0
                
            # Apply correction using calibration points
            return CoordinateConverter._correct_coordinates(tar_x, tar_y)
        except (ZeroDivisionError, TypeError):
            return None, None

    @staticmethod
    def grid_to_robot_coordinates(x, y, workspace_bounds):
        """
        Convert grid coordinates to robot coordinates
        
        Args:
            x (float): X coordinate in grid space
            y (float): Y coordinate in grid space
            workspace_bounds (dict): Dictionary containing workspace boundaries
            
        Returns:
            tuple: (x, y) coordinates in robot space
        """
        if not workspace_bounds or any(v is None for v in [x, y]):
            return None, None

        try:
            y_relative = (y - workspace_bounds['y_fixed']) / workspace_bounds['box_height']
            x_relative = (x - workspace_bounds['x_fixed']) / workspace_bounds['box_width']
            
            tar_x = (169 - (y_relative * 169)) - 4  # Changed from 135 to 169
            tar_y = (193 - (x_relative * 193)) - 0  # Changed from 145 to 193
            
            if tar_x < 0:
                tar_x = 0
                
            # Apply correction using calibration points
            return CoordinateConverter._correct_coordinates(tar_x, tar_y)
        except (KeyError, ZeroDivisionError, TypeError):
            return None, None
