import numpy as np
import cv2
from utils.coordinate_converter import CoordinateConverter

class PnP2Operations:
    def __init__(self, thread, workspace_bounds):
        self.thread = thread
        self.workspace_bounds = workspace_bounds
        self.coordinate_converter = CoordinateConverter()
        self.remembered_positions = {
            'objects': [],           # Objects to move from left
            'existing_top': [],      # Objects already in top area
            'placed_top': [],        # Objects we've moved to top
            'baskets': {
                'left': [],
                'right': []
            },
            'basket_cells': {},      # Map basket center to its cells
            'basket_filled_cells': {} # Track filled cells per basket
        }
        self.current_stage = 'move_objects_to_top'
        self.object_spacing = -15    # 15mm spacing between objects
        self.basket_capacity = 12
        self.top_area_y_threshold = None
        self.stage_sequence = [
            'move_objects_to_top',   # First move objects to top
            'check_baskets',         # Then check and move baskets if needed
            'fill_right_baskets',    # Finally fill right baskets
            'complete'               # Operation complete
        ]
        self.current_stage_index = 0

    def start_operation(self):
        """Initialize PnP2 operation"""
        if self.workspace_bounds:
            self.top_area_y_threshold = self.workspace_bounds['y_fixed'] - 10

            # Remember initial object positions
            for obj_pos in self.thread.available_positions['pickable_objects']:
                if obj_pos[1] >= self.top_area_y_threshold:
                    # Objects in bottom-left area to be moved
                    if (obj_pos[0] < 320):  # Left side of image
                        self.remembered_positions['objects'].append(obj_pos)
                else:
                    # Objects already in top area
                    self.remembered_positions['existing_top'].append(obj_pos)

            # Remember basket positions
            if hasattr(self.thread.basket_detector, 'basket_infos'):
                for basket_info in self.thread.basket_detector.basket_infos:
                    basket_center = basket_info['center']
                    basket_cells = self.get_basket_cells(basket_info)
                    
                    # Classify basket as left or right
                    if basket_center[0] < 320:  # Left side of image
                        self.remembered_positions['baskets']['left'].append(basket_info)
                    else:
                        self.remembered_positions['baskets']['right'].append(basket_info)
                    
                    # Store cells for this basket
                    self.remembered_positions['basket_cells'][basket_center] = basket_cells
                    self.remembered_positions['basket_filled_cells'][basket_center] = []

            return len(self.remembered_positions['objects']) > 0 or len(self.remembered_positions['baskets']['left']) > 0
        return False

    def get_next_operation(self):
        """Get next operation based on current stage"""
        current_stage = self.stage_sequence[self.current_stage_index]
        
        if current_stage == 'move_objects_to_top':
            result = self.get_next_object_move()
            if not result:
                self.current_stage_index += 1
            return result
            
        elif current_stage == 'check_baskets':
            if self.check_and_move_baskets():
                self.current_stage_index += 1
            return None  # This stage just checks and updates state
            
        elif current_stage == 'fill_right_baskets':
            result = self.get_next_basket_fill()
            if not result:
                self.current_stage_index += 1
            return result
            
        return None  # Operation complete

    def get_next_object_move(self):
        """Move next object from left to top"""
        if not self.remembered_positions['objects']:
            return None

        # Get next object from left side
        cam_x, cam_y = self.remembered_positions['objects'].pop(0)
        
        # Convert source position to robot coordinates
        robot_x, robot_y = self.coordinate_converter.to_robot_coordinates(
            cam_x, cam_y, 
            self.workspace_bounds['x_fixed'],
            self.workspace_bounds['y_fixed'],
            self.workspace_bounds['box_width'],
            self.workspace_bounds['box_height']
        )
        
        # Calculate safe placement position in top area
        dest_coords = self.find_safe_top_position()
        if dest_coords:
            dest_robot_x, dest_robot_y = self.coordinate_converter.to_robot_coordinates(
                dest_coords[0], dest_coords[1],
                self.workspace_bounds['x_fixed'],
                self.workspace_bounds['y_fixed'],
                self.workspace_bounds['box_width'],
                self.workspace_bounds['box_height']
            )
            self.remembered_positions['placed_top'].append((dest_robot_x, dest_robot_y))
            return robot_x, robot_y, dest_robot_x, dest_robot_y
        
        return None

    def find_safe_top_position(self):
        """Find safe position in top area considering existing objects"""
        if not self.workspace_bounds:
            return None
        
        # Start position in top area in camera coordinates
        base_x = self.workspace_bounds['x_fixed'] + (self.workspace_bounds['box_width'] * 0.25)
        base_y = self.top_area_y_threshold - (self.workspace_bounds['box_height'] / 4)
        
        # Convert base position to robot coordinates
        robot_base_x, robot_base_y = self.coordinate_converter.to_robot_coordinates(
            base_x, base_y,
            self.workspace_bounds['x_fixed'],
            self.workspace_bounds['y_fixed'],
            self.workspace_bounds['box_width'],
            self.workspace_bounds['box_height']
        )
        
        # Calculate Y offset based on number of objects already placed
        # object_spacing is in real robot coordinates (mm)
        y_offset = len(self.remembered_positions['placed_top']) * self.object_spacing
        target_y = robot_base_y + y_offset
        
        # Convert back to camera coordinates for collision checking
        camera_x, camera_y = base_x, base_y + (y_offset * 0.7)  # 0.7 is approximate camera-to-robot ratio
        
        # Get all objects to avoid (in camera coordinates)
        all_top_objects = (
            self.remembered_positions['existing_top'] + 
            [(self.coordinate_converter.to_robot_coordinates(
                x, y,
                self.workspace_bounds['x_fixed'],
                self.workspace_bounds['y_fixed'],
                self.workspace_bounds['box_width'],
                self.workspace_bounds['box_height']
            )) for x, y in self.remembered_positions['placed_top']]
        )
        
        # Check if position is safe
        if self.is_position_safe(camera_x, camera_y, all_top_objects):
            return camera_x, camera_y
        
        # If position not safe, try nearby positions
        for offset in range(5, 50, 5):  # Try positions up to 50mm away
            for direction in [-1, 1]:
                test_y = camera_y + (direction * offset)
                if self.is_position_safe(camera_x, test_y, all_top_objects):
                    return camera_x, test_y
        
        return None

    def check_and_move_baskets(self):
        """Check if baskets need to be moved and update state"""
        # If no left baskets or right side is full, skip basket movement
        if (not self.remembered_positions['baskets']['left'] or 
            len(self.remembered_positions['baskets']['right']) >= 2):
            return True
        
        # Move left basket to right if space available
        basket_info = self.remembered_positions['baskets']['left'].pop(0)
        self.remembered_positions['baskets']['right'].append(basket_info)
        return True

    def get_next_basket_fill(self):
        """Get parameters for filling right side baskets"""
        # Check right baskets for empty cells
        for basket_info in self.remembered_positions['baskets']['right']:
            basket_center = basket_info['center']
            cells = self.remembered_positions['basket_cells'][basket_center]
            filled_cells = self.remembered_positions['basket_filled_cells'][basket_center]
            
            # Find empty cell
            for cell in cells:
                if cell not in filled_cells and len(filled_cells) < self.basket_capacity:
                    # Find object to place
                    if self.remembered_positions['placed_top']:
                        obj_pos = self.remembered_positions['placed_top'].pop(0)
                        robot_x, robot_y = obj_pos
                        
                        # Convert cell position to robot coordinates
                        dest_robot_x, dest_robot_y = self.coordinate_converter.to_robot_coordinates(
                            cell[0], cell[1],
                            self.workspace_bounds['x_fixed'],
                            self.workspace_bounds['y_fixed'],
                            self.workspace_bounds['box_width'],
                            self.workspace_bounds['box_height']
                        )
                        
                        # Mark cell as filled
                        self.remembered_positions['basket_filled_cells'][basket_center].append(cell)
                        return robot_x, robot_y, dest_robot_x, dest_robot_y

        # If no more right baskets to fill, check if we can move left baskets
        self.current_stage = 'move_left_basket'
        return None

    def is_position_safe(self, x, y, existing_positions):
        """Check if position is safe from collisions"""
        min_distance = abs(self.object_spacing) * 0.8  # 80% of spacing as minimum distance
        
        for pos in existing_positions:
            dist = np.sqrt((x - pos[0])**2 + (y - pos[1])**2)
            if dist < min_distance:
                return False
        return True

    def get_basket_cells(self, basket_info):
        """Get grid cell positions for a basket"""
        cells = []
        center_x, center_y = basket_info['center']
        width, height = basket_info['dimensions']
        grid_angle = basket_info['angle']
        grid_rows = basket_info['grid_params']['rows']
        grid_cols = basket_info['grid_params']['cols']
        
        step_x = width / grid_cols
        step_y = height / grid_rows
        M = cv2.getRotationMatrix2D((center_x, center_y), grid_angle, 1.0)
        
        for row in range(grid_rows):
            for col in range(grid_cols):
                x_offset = (col * step_x + step_x/2) - (width / 2)
                y_offset = (row * step_y + step_y/2) - (height / 2)
                point = (center_x + x_offset, center_y + y_offset)
                rotated_point = self.thread.basket_detector.rotate_point(point,(center_x, center_y),M)
                cells.append(rotated_point)

        return cells