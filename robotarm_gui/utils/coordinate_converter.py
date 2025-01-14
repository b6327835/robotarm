class CoordinateConverter:
    @staticmethod
    def to_robot_coordinates(target_x, target_y, x_fixed, y_fixed, box_width, box_height):
        """
        Convert camera coordinates to robot coordinates
        
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
        y_relative = (target_y - y_fixed) / box_height
        x_relative = (target_x - x_fixed) / box_width

        tar_x = (169 - (y_relative * 169)) - 4  # Changed from 135 to 169
        tar_y = (193 - (x_relative * 193)) - 0  # Changed from 145 to 193
        
        if tar_x < 0:
            tar_x = 0
            
        return tar_x, tar_y

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
        y_relative = (y - workspace_bounds['y_fixed']) / workspace_bounds['box_height']
        x_relative = (x - workspace_bounds['x_fixed']) / workspace_bounds['box_width']
        
        tar_x = (169 - (y_relative * 169)) - 4  # Changed from 135 to 169
        tar_y = (193 - (x_relative * 193)) - 0  # Changed from 145 to 193
        
        if tar_x < 0:
            tar_x = 0
            
        return tar_x, tar_y
