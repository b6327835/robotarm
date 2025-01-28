import numpy as np
from utils.coordinate_converter import CoordinateConverter

class PnP2Operations:
    def __init__(self, video_thread, workspace_bounds):
        self.video_thread = video_thread
        self.workspace_bounds = workspace_bounds
        self.remembered_positions = {
            'objects': [],
            'placed_top': [], 
            'baskets': {
                'left': [],
                'right': []
            },
            'basket_cells': {},
            'occupied_cells': set(),
            'grid_positions': {}  # Add grid positions tracking
        }
        self.stage_sequence = ['move_to_top', 'fill_baskets', 'move_baskets', 'complete']
        self.current_stage_index = 0
        self.object_spacing = 15  # 15mm spacing between objects
        self.basket_cell_capacity = 12  # Maximum cells per basket

    def start_operation(self):
        """Initialize operation by capturing current positions"""
        if not self.video_thread.available_positions:
            return False

        # Remember initial object positions
        self.remembered_positions['objects'] = [
            (x, y) for x, y in self.video_thread.available_positions.get('pickable_objects', [])
            if y >= self.workspace_bounds['y_fixed']  # Only bottom objects
        ]

        # Remember basket positions and initialize cell tracking
        basket_positions = self.video_thread.basket_detector.basket_infos
        if not basket_positions:
            return False

        # Track initially occupied cells
        grid_positions = self.video_thread.available_positions.get('grid', {})
        for grid_id in grid_positions:
            if grid_id.startswith('occupied_'):
                original_id = grid_id.replace('occupied_', '')
                self.remembered_positions['occupied_cells'].add(original_id)

        # Initialize baskets
        for basket in basket_positions:
            center = basket['center']
            if center[0] < self.workspace_bounds['x_fixed'] + self.workspace_bounds['box_width']/2:
                self.remembered_positions['baskets']['left'].append(basket)
            else:
                self.remembered_positions['baskets']['right'].append(basket)
            self.remembered_positions['basket_cells'][center] = []

        return True

    def is_position_safe(self, x, y):
        return True
        """Check if position is safe (not too close to other objects)"""
        for placed_obj in self.remembered_positions['placed_top']:
            placed_x, placed_y = placed_obj['camera']  # Access camera coordinates from dictionary
            dist = np.sqrt((x - placed_x)**2 + (y - placed_y)**2)
            if dist < self.object_spacing:
                return False
        return True

    def find_next_safe_position(self, base_x):
        """Find next safe position in top area"""
        current_y = self.workspace_bounds['y_fixed'] - 60  # Initial Y position
        
        while current_y > self.workspace_bounds['y_fixed'] - self.workspace_bounds['box_height']/2:
            if self.is_position_safe(float(base_x), float(current_y)):  # Ensure values are float
                return base_x, current_y
            current_y -= self.object_spacing
        return None

    def get_next_operation(self):
        """Get next pick and place operation based on current stage"""
        current_stage = self.stage_sequence[self.current_stage_index]
        
        if current_stage == 'move_to_top':
            return self._handle_move_to_top()
        elif current_stage == 'fill_baskets':
            return self._handle_fill_baskets()
        elif current_stage == 'move_baskets':
            return self._handle_move_baskets()
        elif current_stage == 'complete':
            return None
            
        return None

    def _handle_move_to_top(self):
        """Handle moving objects to top area"""
        # Store grid positions when starting this stage
        if not self.remembered_positions['grid_positions']:
            self.remembered_positions['grid_positions'] = self.video_thread.available_positions.get('grid', {})
            # Update occupied cells from grid positions
            for grid_id in self.remembered_positions['grid_positions']:
                if grid_id.startswith('occupied_'):
                    original_id = grid_id.replace('occupied_', '')
                    self.remembered_positions['occupied_cells'].add(original_id)

        if not self.remembered_positions['objects']:
            self.current_stage_index += 1
            return None

        # Rest of move_to_top logic remains the same
        obj_x, obj_y = self.remembered_positions['objects'][0]
        next_pos = self.find_next_safe_position(obj_x)
        if not next_pos:
            self.current_stage_index += 1
            return None

        dest_cam_x, dest_cam_y = next_pos
        dest_x, dest_y = CoordinateConverter.to_robot_coordinates(
            dest_cam_x, dest_cam_y,
            self.workspace_bounds['x_fixed'],
            self.workspace_bounds['y_fixed'],
            self.workspace_bounds['box_width'],
            self.workspace_bounds['box_height']
        )

        source_x, source_y = CoordinateConverter.to_robot_coordinates(
            obj_x, obj_y,
            self.workspace_bounds['x_fixed'],
            self.workspace_bounds['y_fixed'],
            self.workspace_bounds['box_width'],
            self.workspace_bounds['box_height']
        )

        # Update remembered positions
        self.remembered_positions['objects'].pop(0)
        self.remembered_positions['placed_top'].append({
            'camera': (dest_cam_x, dest_cam_y),
            'robot': (dest_x, dest_y),
            'original': (obj_x, obj_y)
        })

        return source_x, source_y, dest_x, dest_y

    def _handle_fill_baskets(self):
        """Handle filling baskets with objects"""
        if not self.remembered_positions['placed_top']:
            print(f"No objects in top area to place in baskets. Placed top: {self.remembered_positions['placed_top']}")
            return None  # Return None without incrementing stage

        print(f"Objects available for basket placement: {len(self.remembered_positions['placed_top'])}")
        
        # Use stored grid positions instead of getting them again
        grid_positions = self.remembered_positions['grid_positions']
        
        # Check right baskets first
        for basket in self.remembered_positions['baskets']['right']:
            basket_center = basket['center']
            print(f"Checking basket at {basket_center}")
            print(f"Available grid positions: {len(grid_positions)}")
            print(f"Occupied cells: {self.remembered_positions['occupied_cells']}")

            for i in range(1, self.basket_cell_capacity + 1):
                if not self.remembered_positions['placed_top']:
                    break
                
                cell_id = f"R{i}"
                print(f"Checking cell {cell_id}")
                
                if (cell_id in self.remembered_positions['occupied_cells'] or
                    f"occupied_{cell_id}" in grid_positions or
                    cell_id not in grid_positions):
                    print(f"Cell {cell_id} is unavailable, skipping")
                    continue

                # Process available cell
                next_cell = grid_positions[cell_id]
                obj_data = self.remembered_positions['placed_top'].pop(0)
                source_x, source_y = obj_data['robot']
                dest_x, dest_y = CoordinateConverter.to_robot_coordinates(
                    next_cell[0], next_cell[1],
                    self.workspace_bounds['x_fixed'],
                    self.workspace_bounds['y_fixed'],
                    self.workspace_bounds['box_width'],
                    self.workspace_bounds['box_height']
                )

                # Update tracking
                self.remembered_positions['basket_cells'][basket_center] = (
                    self.remembered_positions['basket_cells'].get(basket_center, []) + [cell_id]
                )
                self.remembered_positions['occupied_cells'].add(cell_id)
                self.remembered_positions['grid_positions'][f"occupied_{cell_id}"] = next_cell
                
                return source_x, source_y, dest_x, dest_y

        print("No more available cells in right baskets")
        self.current_stage_index += 1
        return None

    def _handle_move_baskets(self):
        """Handle moving left baskets to right side"""
        if not self._can_move_baskets():
            self.current_stage_index += 1
            return None

        # Get next basket to move
        source_basket = self.remembered_positions['baskets']['left'][0]
        target_position = self._find_right_side_space()
        
        if target_position:
            source_x, source_y = CoordinateConverter.to_robot_coordinates(
                source_basket['center'][0], source_basket['center'][1],
                self.workspace_bounds['x_fixed'],
                self.workspace_bounds['y_fixed'],
                self.workspace_bounds['box_width'],
                self.workspace_bounds['box_height']
            )
            
            dest_x, dest_y = CoordinateConverter.to_robot_coordinates(
                target_position[0], target_position[1],
                self.workspace_bounds['x_fixed'],
                self.workspace_bounds['y_fixed'],
                self.workspace_bounds['box_width'],
                self.workspace_bounds['box_height']
            )

            # Update basket positions
            self.remembered_positions['baskets']['left'].pop(0)
            self.remembered_positions['baskets']['right'].append({
                'center': target_position,
                'cells': []
            })
            
            return source_x, source_y, dest_x, dest_y

        self.current_stage_index += 1
        return None

    def _get_next_basket_cell(self, basket, filled_count):
        """Calculate position of next empty cell in basket"""
        if filled_count >= self.basket_cell_capacity:
            return None

        rows = basket['grid_params']['rows']
        cols = basket['grid_params']['cols']
        cell_width = basket['dimensions'][0] / cols
        cell_height = basket['dimensions'][1] / rows

        row = filled_count // cols
        col = filled_count % cols

        if row < rows and col < cols:
            x = basket['center'][0] + (col - (cols-1)/2) * cell_width
            y = basket['center'][1] + (row - (rows-1)/2) * cell_height
            return (x, y)

        return None

    def _can_move_baskets(self):
        """Check if there's space to move baskets"""
        return (len(self.remembered_positions['baskets']['left']) > 0 and
                self._find_right_side_space() is not None)

    def _find_right_side_space(self):
        """Find available space on right side for a basket"""
        right_x = self.workspace_bounds['x_fixed'] + self.workspace_bounds['box_width'] * 0.75
        right_y = self.workspace_bounds['y_fixed'] + self.workspace_bounds['box_height'] * 0.25
        
        # Check if position is far enough from existing right baskets
        for basket in self.remembered_positions['baskets']['right']:
            dist = np.sqrt((right_x - basket['center'][0])**2 + 
                         (right_y - basket['center'][1])**2)
            if dist < 50:  # 50mm minimum separation
                return None
        
        return (right_x, right_y)
