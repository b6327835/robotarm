import numpy as np
from scipy.interpolate import RBFInterpolator
import json
import os

class CoordinateConverter:
    _rbf_x = None
    _rbf_y = None
    _calibration_points = None
    
    @classmethod
    def load_calibration_points(cls):
        """Load calibration points from JSON file"""
        if cls._calibration_points is None:
            try:
                json_path = os.path.join(os.path.dirname(__file__), '..', 'calibration_points.json')
                with open(json_path, 'r') as f:
                    points = json.load(f)
                    # Create arrays of source (camera) and target (robot) coordinates
                    source_points = []
                    target_points = []
                    for point in points:
                        source_points.append(point['cam'])
                        target_points.append(point['real'])
                    cls._calibration_points = {
                        'source': np.array(source_points),
                        'target': np.array(target_points)
                    }
            except (FileNotFoundError, json.JSONDecodeError) as e:
                print(f"Error loading calibration points: {e}")
                cls._calibration_points = {'source': None, 'target': None}
        return cls._calibration_points

    @classmethod
    def _initialize_interpolators(cls):
        """Initialize RBF interpolators for x and y coordinates"""
        if cls._rbf_x is None or cls._rbf_y is None:
            points = cls.load_calibration_points()
            if points['source'] is None or points['target'] is None:
                return
            
            # Create separate interpolators for x and y coordinates
            cls._rbf_x = RBFInterpolator(
                points['source'],
                points['target'][:, 0],
                kernel='thin_plate_spline',
                epsilon=0.1
            )
            cls._rbf_y = RBFInterpolator(
                points['source'],
                points['target'][:, 1],
                kernel='thin_plate_spline',
                epsilon=0.1
            )

    @classmethod
    def transform_coordinates(cls, x, y):
        """Transform coordinates using thin plate spline interpolation"""
        if x is None or y is None:
            return x, y

        cls._initialize_interpolators()
        if cls._rbf_x is None or cls._rbf_y is None:
            return x, y

        point = np.array([[x, y]])
        transformed_x = float(cls._rbf_x(point))
        transformed_y = float(cls._rbf_y(point))

        return transformed_x, transformed_y

    @staticmethod
    def to_robot_coordinates(target_x, target_y, x_fixed, y_fixed, box_width, box_height, 
                           use_raw=False, use_interpolation=True):
        """Convert camera coordinates to robot coordinates"""
        if any(v is None for v in [target_x, target_y, x_fixed, y_fixed, box_width, box_height]):
            return None, None

        try:
            # Return raw coordinates if requested
            if use_raw:
                return target_x, target_y

            # Apply transformation if interpolation is enabled
            if use_interpolation:
                return CoordinateConverter.transform_coordinates(target_x, target_y)
            
            # Fall back to raw coordinates if no interpolation
            return target_x, target_y

        except (ZeroDivisionError, TypeError):
            return None, None

    @staticmethod
    def grid_to_robot_coordinates(x, y, workspace_bounds, use_raw=False):
        """Convert grid coordinates to robot coordinates"""
        if not workspace_bounds or any(v is None for v in [x, y]):
            return None, None

        try:
            if use_raw:
                return x, y
            return CoordinateConverter.transform_coordinates(x, y)
        except (KeyError, ZeroDivisionError, TypeError):
            return None, None
