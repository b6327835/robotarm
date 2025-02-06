from PyQt5.QtCore import QThread
import time
import queue

class AutoPnPThread(QThread):
    def __init__(self, grid_rows, grid_cols, cell_size, parent, grid_start_x, grid_start_y):
        super().__init__()
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        self.cell_size = cell_size
        self.grid_start_x = grid_start_x  # Starting X position
        self.grid_start_y = grid_start_y  # Starting Y position
        self.parent = parent  # Reference to the main UI class

    def run(self):
        self.parent.placed_count = 0
        max_placements = self.grid_rows * self.grid_cols
        print(f"AutoPnPThread started. Max placements: {max_placements}")
        while self.parent.placed_count < max_placements:
            print("AutoPnPThread loop started.")
            try:
                target_x, target_y = self.parent.object_queue.get_nowait()
                print(f"Got object: ({target_x}, {target_y})")
                dest_x = self.grid_start_x + (self.parent.placed_count % self.grid_cols) * self.cell_size
                dest_y = self.grid_start_y + (self.parent.placed_count // self.grid_cols) * self.cell_size
                # Update tar_x and tar_y for the move thread
                self.parent.tar_x = target_x
                self.parent.tar_y = target_y
                self.parent.start_move_thread("auto_pnp", dest_x, dest_y)
                while self.parent.is_move_running:
                    time.sleep(0.1)
                print("Move completed. Moving to next object.")
                self.parent.placed_count += 1
            except queue.Empty:
                print("No objects in queue. Waiting...")
                time.sleep(0.1)
        print("AutoPnPThread completed.")