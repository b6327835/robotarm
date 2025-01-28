import cv2
import numpy as np

class BasketDetector:
    def __init__(self):
        # Red basket detection parameters
        self.params = {
            'low_h1': 0, 'high_h1': 10,
            'low_h2': 170, 'high_h2': 180,
            'low_s': 18, 'high_s': 255,
            'low_v': 150, 'high_v': 255,
            'min_area': 2500,
            'kernel_size': 1,
            'erosion_iter': 1, 
            'dilation_iter': 1
        }
        self.occupied_cells = {}  # Track occupied grid cells

    def detect_basket(self, hsv_img, x_fixed, box_width, box_height, y_fixed):
        # Create red mask using two HSV ranges
        red_mask1 = cv2.inRange(hsv_img, 
            np.array([self.params['low_h1'], self.params['low_s'], self.params['low_v']]),
            np.array([self.params['high_h1'], self.params['high_s'], self.params['high_v']]))

        red_mask2 = cv2.inRange(hsv_img,
            np.array([self.params['low_h2'], self.params['low_s'], self.params['low_v']]),
            np.array([self.params['high_h2'], self.params['high_s'], self.params['high_v']]))

        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Apply morphological operations
        if self.params['kernel_size'] > 0:
            kernel = np.ones((self.params['kernel_size'], self.params['kernel_size']), np.uint8)
        if self.params['erosion_iter'] > 0:
            red_mask = cv2.erode(red_mask, kernel, iterations=self.params['erosion_iter'])
        if self.params['dilation_iter'] > 0:
            red_mask = cv2.dilate(red_mask, kernel, iterations=self.params['dilation_iter'])

        # Find red basket contours
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        basket_infos = []  # Changed to list to store multiple baskets
        
        for contour in red_contours:
            area = cv2.contourArea(contour)
            if area > self.params['min_area']:
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int64(box)
                
                (center_x, center_y), (width, height), angle = rect
                
                # Adjust dimensions to target inner area
                inner_margin = 10  # pixels to shrink from outer edge
                if width < height:
                    width, height = height - inner_margin, width - inner_margin
                    angle += 90
                else:
                    width, height = width - inner_margin, height - inner_margin
                grid_angle = -angle

                # Check if basket is in bottom rectangle
                if (x_fixed <= center_x <= x_fixed + box_width and 
                    y_fixed <= center_y <= y_fixed + box_height):
                    
                    basket_info = {
                        'box': box,
                        'center': (center_x, center_y),
                        'dimensions': (width, height),
                        'angle': grid_angle,
                        'grid_params': {
                            'rows': 3,
                            'cols': 4
                        }
                    }
                    basket_infos.append(basket_info)  # Append instead of break

        return basket_infos  # Return list of all baskets

    @staticmethod
    def check_cell_occupied(point, detected_objects, threshold=10):
        """Check if a grid cell contains any detected object"""
        if not detected_objects:
            return False
        for obj_x, obj_y in detected_objects:
            distance = np.sqrt((point[0] - obj_x)**2 + (point[1] - obj_y)**2)
            if distance < threshold:
                return True
        return False

    @staticmethod
    def rotate_point(point, center, M):
        """Rotate a point around a center using transformation matrix M"""
        px, py = point
        cx, cy = center
        # Translate point to origin
        px_t = px - cx
        py_t = py - cy
        # Apply rotation
        px_r = M[0][0] * px_t + M[0][1] * py_t
        py_r = M[1][0] * px_t + M[1][1] * py_t
        # Translate back
        return (int(px_r + cx), int(py_r + cy))

    @staticmethod
    def draw_basket_grid(img, basket_infos, detected_objects=[], show_label=False):
        """Draw basket grid with optional label display"""
        # Handle single basket info being passed (backward compatibility)
        if not isinstance(basket_infos, list):
            basket_infos = [basket_infos]

        # Split baskets into left and right sides
        left_baskets = []
        right_baskets = []
        img_center_x = img.shape[1] / 2

        for basket_info in basket_infos:
            if not basket_info:
                continue
            
            if basket_info['center'][0] < img_center_x:
                left_baskets.append(basket_info)
            else:
                right_baskets.append(basket_info)

        # Sort baskets by y-coordinate within their respective sides
        left_baskets.sort(key=lambda x: x['center'][1])
        right_baskets.sort(key=lambda x: x['center'][1])

        # Counter for dots on each side
        left_counter = 1
        right_counter = 1

        grid_positions = {}  # Add this at the start of the method

        # Process left baskets
        for basket_info in left_baskets:
            box = basket_info['box']
            center_x, center_y = basket_info['center']
            width, height = basket_info['dimensions']
            grid_angle = basket_info['angle']
            grid_rows = basket_info['grid_params']['rows']
            grid_cols = basket_info['grid_params']['cols']

            # Skip drawing the yellow contour since we're focusing on inner grid
            # Instead, draw a slightly smaller rectangle
            inner_box = np.copy(box)
            # Shrink the box towards center
            # for i in range(len(inner_box)):
            #     vec = inner_box[i] - np.array([center_x, center_y])
            #     scale = 0.8  # Scale factor to shrink the box
            #     inner_box[i] = np.array([center_x, center_y]) + vec * scale
            # cv2.drawContours(img, [inner_box.astype(np.int32)], 0, (255, 255, 255), 1)

            # Calculate grid parameters and draw grid lines (existing code)
            step_x = width / grid_cols
            step_y = height / grid_rows
            M = cv2.getRotationMatrix2D((center_x, center_y), grid_angle, 1.0)

            # Draw vertical grid lines
            for i in range(grid_cols + 1):
                x_offset = (i * step_x) - (width / 2)
                start_point = (center_x + x_offset, center_y - height/2)
                end_point = (center_x + x_offset, center_y + height/2)
                
                start_rotated = BasketDetector.rotate_point(start_point, (center_x, center_y), M)
                end_rotated = BasketDetector.rotate_point(end_point, (center_x, center_y), M)
                
                cv2.line(img, start_rotated, end_rotated, (0, 255, 255), 1)

            # Draw horizontal grid lines
            for i in range(grid_rows + 1):
                y_offset = (i * step_y) - (height / 2)
                start_point = (center_x - width/2, center_y + y_offset)
                end_point = (center_x + width/2, center_y + y_offset)
                
                start_rotated = BasketDetector.rotate_point(start_point, (center_x, center_y), M)
                end_rotated = BasketDetector.rotate_point(end_point, (center_x, center_y), M)
                
                cv2.line(img, start_rotated, end_rotated, (0, 255, 255), 1)

            # Draw numbered dots
            for row in range(grid_rows):
                for col in range(grid_cols):
                    x_offset = (col * step_x + step_x/2) - (width / 2)
                    y_offset = (row * step_y + step_y/2) - (height / 2)
                    point = (center_x + x_offset, center_y + y_offset)
                    
                    # Rotate point
                    px = M[0][0] * (point[0] - center_x) + M[0][1] * (point[1] - center_y) + center_x
                    py = M[1][0] * (point[0] - center_x) + M[1][1] * (point[1] - center_y) + center_y
                    point_rotated = (int(px), int(py))
                    
                    # Always use red for left side (no occupation check needed)
                    cv2.circle(img, point_rotated, 2, (0, 0, 255), -1)
                    if show_label:
                        label = f"L{left_counter}"
                        cv2.putText(img, label, 
                                  (point_rotated[0] + 5, point_rotated[1] + 5),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
                    
                    # Add to grid positions
                    grid_positions[f"L{left_counter}"] = (px, py)
                    left_counter += 1

        # Process right baskets
        for basket_info in right_baskets:
            # Same drawing code as above, but with right-side numbering
            box = basket_info['box']
            center_x, center_y = basket_info['center']
            width, height = basket_info['dimensions']
            grid_angle = basket_info['angle']
            grid_rows = basket_info['grid_params']['rows']
            grid_cols = basket_info['grid_params']['cols']

            # Skip drawing the yellow contour since we're focusing on inner grid
            # Instead, draw a slightly smaller rectangle
            inner_box = np.copy(box)
            # Shrink the box towards center
            # for i in range(len(inner_box)):
            #     vec = inner_box[i] - np.array([center_x, center_y])
            #     scale = 0.8  # Scale factor to shrink the box
            #     inner_box[i] = np.array([center_x, center_y]) + vec * scale
            # cv2.drawContours(img, [inner_box.astype(np.int32)], 0, (0, 255, 255), 1)

            # Calculate grid parameters and draw grid lines
            step_x = width / grid_cols
            step_y = height / grid_rows
            M = cv2.getRotationMatrix2D((center_x, center_y), grid_angle, 1.0)

            # Draw vertical grid lines
            for i in range(grid_cols + 1):
                x_offset = (i * step_x) - (width / 2)
                start_point = (center_x + x_offset, center_y - height/2)
                end_point = (center_x + x_offset, center_y + height/2)
                
                start_rotated = BasketDetector.rotate_point(start_point, (center_x, center_y), M)
                end_rotated = BasketDetector.rotate_point(end_point, (center_x, center_y), M)
                
                cv2.line(img, start_rotated, end_rotated, (0, 255, 255), 1)

            # Draw horizontal grid lines
            for i in range(grid_rows + 1):
                y_offset = (i * step_y) - (height / 2)
                start_point = (center_x - width/2, center_y + y_offset)
                end_point = (center_x + width/2, center_y + y_offset)
                
                start_rotated = BasketDetector.rotate_point(start_point, (center_x, center_y), M)
                end_rotated = BasketDetector.rotate_point(end_point, (center_x, center_y), M)
                
                cv2.line(img, start_rotated, end_rotated, (0, 255, 255), 1)

            # Draw numbered dots
            for row in range(grid_rows):
                for col in range(grid_cols):
                    x_offset = (col * step_x + step_x/2) - (width / 2)
                    y_offset = (row * step_y + step_y/2) - (height / 2)
                    point = (center_x + x_offset, center_y + y_offset)
                    
                    # Rotate point
                    px = M[0][0] * (point[0] - center_x) + M[0][1] * (point[1] - center_y) + center_x
                    py = M[1][0] * (point[0] - center_x) + M[1][1] * (point[1] - center_y) + center_y
                    
                    # Before drawing grid points, check if cell is occupied
                    point_rotated = (int(px), int(py))
                    is_occupied = False
                    
                    # Check if this cell contains a detected object
                    is_occupied = BasketDetector.check_cell_occupied(
                        point_rotated, 
                        detected_objects,
                        threshold=10
                    )
                    
                    # Draw dot and label with different color based on occupation
                    color = (255, 0, 0) if is_occupied else (0, 0, 255)  # Blue if occupied, red if free
                    cv2.circle(img, point_rotated, 2, color, -1)
                    if show_label:
                        label = f"R{right_counter}"
                        cv2.putText(img, label, 
                                  (point_rotated[0] + 5, point_rotated[1] + 5),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)
                    
                    # Add position to grid positions with occupation status
                    if is_occupied:
                        grid_positions[f"occupied_R{right_counter}"] = (px, py)
                    else:
                        grid_positions[f"R{right_counter}"] = (px, py)
                    
                    right_counter += 1

        return grid_positions  # Return the grid positions dictionary

    def get_grid_position_by_id(self, img, basket_infos, target_id):
        """Get the coordinates of a specific grid position by its ID (e.g., 'L3' or 'R5')"""
        if not isinstance(basket_infos, list):
            basket_infos = [basket_infos]

        img_center_x = img.shape[1] / 2
        left_baskets = []
        right_baskets = []

        # Split baskets by side
        for basket_info in basket_infos:
            if not basket_info:
                continue
            if basket_info['center'][0] < img_center_x:
                left_baskets.append(basket_info)
            else:
                right_baskets.append(basket_info)

        # Sort baskets by y-coordinate
        left_baskets.sort(key=lambda x: x['center'][1])
        right_baskets.sort(key=lambda x: x['center'][1])

        side = target_id[0].upper()
        try:
            position_number = int(target_id[1:])
        except ValueError:
            return None

        baskets = left_baskets if side == 'L' else right_baskets
        current_count = 0

        for basket_info in baskets:
            center_x, center_y = basket_info['center']
            width, height = basket_info['dimensions']
            grid_angle = basket_info['angle']
            grid_rows = basket_info['grid_params']['rows']
            grid_cols = basket_info['grid_params']['cols']

            # Calculate step sizes
            step_x = width / grid_cols
            step_y = height / grid_rows
            M = cv2.getRotationMatrix2D((center_x, center_y), grid_angle, 1.0)

            # Check each grid position
            for row in range(grid_rows):
                for col in range(grid_cols):
                    current_count += 1
                    if current_count == position_number:
                        # Calculate position
                        x_offset = (col * step_x + step_x/2) - (width / 2)
                        y_offset = (row * step_y + step_y/2) - (height / 2)
                        point = (center_x + x_offset, center_y + y_offset)
                        
                        # Rotate point
                        px = M[0][0] * (point[0] - center_x) + M[0][1] * (point[1] - center_y) + center_x
                        py = M[1][0] * (point[0] - center_x) + M[1][1] * (point[1] - center_y) + center_y
                        
                        return (px, py)
        return None
