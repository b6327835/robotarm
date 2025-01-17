import numpy as np
from scipy.interpolate import RBFInterpolator

class CoordinateConverter:
    # Update calibration points with new coordinate pairs
    CALIBRATION_POINTS = {
        (19.0, 180.0): (35.0, 180.0),
        (16.5, 106.0): (33.0, 116.0),
        (15.0, 7.5): (32.0, 25.0),
        (146.89, 175.81): (160.0, 178.0),
        (159.5, 83.0): (172.0, 95.0),
        (144.0, 9.7): (155.0, 23.0),
        (252.5, 182.0): (260.0, 183.0),
        (245.8, 96.5): (254.0, 105.0),
        (252.0, 11.7): (260.0, 28.0),
    }
    
    # Updated scaling factors based on the new coordinate pairs
    SCALE_X = 0.9635
    SCALE_Y = 0.9138
    OFFSET_X = 17.24
    OFFSET_Y = 17.11
    
    # Initialize interpolators as class variables
    _rbf_x = None
    _rbf_y = None
    
    @classmethod
    def _initialize_interpolators(cls):
        """Initialize RBF interpolators for x and y coordinates"""
        if cls._rbf_x is None or cls._rbf_y is None:
            current_points = np.array(list(cls.CALIBRATION_POINTS.keys()))
            desired_points = np.array(list(cls.CALIBRATION_POINTS.values()))
            
            # Create separate interpolators for x and y coordinates
            cls._rbf_x = RBFInterpolator(
                current_points, 
                desired_points[:, 0],
                kernel='thin_plate_spline',
                epsilon=0.1
            )
            cls._rbf_y = RBFInterpolator(
                current_points,
                desired_points[:, 1],
                kernel='thin_plate_spline',
                epsilon=0.1
            )

    @classmethod
    def _correct_coordinates(cls, x, y):
        """Correct coordinates using RBF interpolation with smoothing"""
        if x is None or y is None:
            return x, y

        cls._initialize_interpolators()
        point = np.array([[x, y]])

        # Get interpolated coordinates
        corrected_x = float(cls._rbf_x(point))
        corrected_y = float(cls._rbf_y(point))

        # Add boundary checks
        corrected_x = np.clip(corrected_x, 0, 300)  # Adjust bounds as needed
        corrected_y = np.clip(corrected_y, 0, 300)  # Adjust bounds as needed

        # Calculate confidence based on distance to nearest calibration point
        min_distance = float('inf')
        for current in cls.CALIBRATION_POINTS.keys():
            distance = np.sqrt((current[0] - x)**2 + (current[1] - y)**2)
            min_distance = min(min_distance, distance)

        # If too far from calibration points, fall back to linear correction
        if min_distance > 50:  # Adjust threshold as needed
            nearest_current, nearest_desired = cls._get_nearest_point(x, y)
            dx = nearest_desired[0] - nearest_current[0]
            dy = nearest_desired[1] - nearest_current[1]
            corrected_x = x + dx
            corrected_y = y + dy

        return corrected_x, corrected_y

    @classmethod
    def _correct_coordinates_simple(cls, x, y):
        """Apply simple scaling and offset correction"""
        if x is None or y is None:
            return x, y
            
        # Apply scaling and offset
        corrected_x = (x * cls.SCALE_X) + cls.OFFSET_X
        corrected_y = (y * cls.SCALE_Y) + cls.OFFSET_Y
        
        return corrected_x, corrected_y

    @classmethod
    def _get_nearest_point(cls, x, y):
        """Find nearest calibration point"""
        min_distance = float('inf')
        nearest_current = None
        nearest_desired = None

        for current, desired in cls.CALIBRATION_POINTS.items():
            distance = np.sqrt((current[0] - x)**2 + (current[1] - y)**2)
            if distance < min_distance:
                min_distance = distance
                nearest_current = current
                nearest_desired = desired

        return nearest_current, nearest_desired

    @staticmethod
    def to_robot_coordinates(target_x, target_y, x_fixed, y_fixed, box_width, box_height, 
                           use_raw=False, use_interpolation=True):
        """
        Convert camera coordinates to robot coordinates
        
        Args:
            target_x (float): Target x coordinate in camera space
            target_y (float): Target y coordinate in camera space
            x_fixed (float): Fixed x coordinate of workspace
            y_fixed (float): Fixed y coordinate of workspace
            box_width (float): Width of workspace
            box_height (float): Height of workspace
            use_raw (bool): If True, use raw coordinates without any correction
            use_interpolation (bool): If True, use RBF interpolation, otherwise use simple scaling
            
        Returns:
            tuple: (x, y) coordinates in robot space
        """
        if any(v is None for v in [target_x, target_y, x_fixed, y_fixed, box_width, box_height]):
            return None, None

        try:
            # Basic conversion
            y_relative = (target_y - y_fixed) / box_height
            x_relative = (target_x - x_fixed) / box_width

            tar_x = (169 - (y_relative * 169))
            tar_y = (193 - (x_relative * 193))
            
            if tar_x < 0:
                tar_x = 0

            # Return raw coordinates if requested
            if use_raw:
                return tar_x, tar_y

            # Choose correction method
            if use_interpolation:
                return CoordinateConverter._correct_coordinates(tar_x, tar_y)
            else:
                return CoordinateConverter._correct_coordinates_simple(tar_x, tar_y)

        except (ZeroDivisionError, TypeError):
            return None, None

    @staticmethod
    def grid_to_robot_coordinates(x, y, workspace_bounds, use_raw=False):
        """
        Convert grid coordinates to robot coordinates
        
        Args:
            x (float): X coordinate in grid space
            y (float): Y coordinate in grid space
            workspace_bounds (dict): Dictionary containing workspace boundaries
            use_raw (bool): If True, use raw coordinates without calibration correction
            
        Returns:
            tuple: (x, y) coordinates in robot space
        """
        if not workspace_bounds or any(v is None for v in [x, y]):
            return None, None

        try:
            y_relative = (y - workspace_bounds['y_fixed']) / workspace_bounds['box_height']
            x_relative = (x - workspace_bounds['x_fixed']) / workspace_bounds['box_width']
            
            tar_x = (169 - (y_relative * 169)) - 4
            tar_y = (193 - (x_relative * 193)) - 0
            
            if tar_x < 0:
                tar_x = 0
                
            # Apply correction only if not using raw coordinates
            if use_raw:
                return tar_x, tar_y
            return CoordinateConverter._correct_coordinates(tar_x, tar_y)
        except (KeyError, ZeroDivisionError, TypeError):
            return None, None
