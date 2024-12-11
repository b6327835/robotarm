from PyQt5.QtCore import QThread, pyqtSignal
import serial
import time

class MoveRobotThread(QThread):
    movement_status = pyqtSignal(str)  # Signal to update UI with movement status

    def __init__(self, x, y, z, mode, parent=None, dest_x=0.0, dest_y=0.0):
        super().__init__(parent)
        # Assign positions directly without validation
        self.x = round(x, 3)
        self.y = round(y, 3)
        self.z = round(z, 3)
        self.dest_x = round(dest_x, 3)
        self.dest_y = round(dest_y, 3)
        self.mode = mode
        self.serial_port = None
        self._connect_serial()

    def _connect_serial(self):
        """Try to connect to various common serial ports"""
        possible_ports = [
            '/dev/ttyUSB0',
            '/dev/ttyACM0',
            'COM1', 'COM2', 'COM3', 'COM4', 'COM5', 'COM6', 'COM7', 'COM8', 'COM9' , 'COM10', 'COM11', 'COM12',
        ]
        
        for port in possible_ports:
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=1)
                self.movement_status.emit(f"Connected to {port}")
                return True
            except serial.SerialException:
                continue
        
        self.movement_status.emit("Failed to connect to any serial port")
        raise Exception("Could not connect to any available serial port")

    def _wait_for_completion(self, timeout=10):
        """Wait for 'Movement complete' response from ESP32"""
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            try:
                response = self.serial_port.readline().decode().strip()
                if response == "Movement complete":
                    return True
                elif response:  # Log any other responses
                    self.movement_status.emit(f"Got response: {response}")
            except Exception as e:
                self.movement_status.emit(f"Error reading response: {str(e)}")
                return False
        return False

    def _send_command(self, command):
        """Send command to the robot arm via serial."""
        try:
            if not self.serial_port or not self.serial_port.is_open:
                self._connect_serial()
            
            # Format command for your specific robot protocol
            formatted_command = f"{command}\n"
            self.serial_port.write(formatted_command.encode())
            
            # Wait for acknowledgment
            response = self.serial_port.readline().decode().strip()
            self.movement_status.emit(f"Response: {response}")
            
            return response == "OK"
        except Exception as e:
            self.movement_status.emit(f"Error sending command: {str(e)}")
            return False

    def _execute_movement(self, positions, vacuum=0, description="Moving", max_attempts=2):
        """Execute movement with error handling, configurable retries."""
        command = f"x{positions[0]},y{positions[1]},z{positions[2]},v{vacuum}"
        for attempt in range(max_attempts):
            self.movement_status.emit(f"{description}: {positions} (Attempt {attempt + 1})")
            if self._send_command(command):
                # Add wait for completion after successful command send
                if self._wait_for_completion():
                    return True
                self.movement_status.emit("Movement timeout")
            self.movement_status.emit(f"Movement failed on attempt {attempt + 1}")
            time.sleep(0.1)  # Wait before retrying
        self.movement_status.emit(f"Failed to execute movement after {max_attempts} attempts.")
        return False

    def run(self):
        try:
            def move():
                return self._execute_movement([self.x, self.y, 0.0], vacuum=0, description="Moving to position")
            def pnp():
                pnp_x = self.x
                pnp_y = self.y
                # Approach position from above
                if not self._execute_movement([pnp_x, pnp_y, 0.0], vacuum=0, description="Approaching position"):
                    return
                # Pick with vacuum on
                if not self._execute_movement([pnp_x, pnp_y, 0.112], vacuum=0, description="move z"):
                    return
                if not self._execute_movement([pnp_x, pnp_y, 0.112], vacuum=1, description="Picking"):
                    return
                # Lift with item
                if not self._execute_movement([pnp_x, pnp_y, 0.0], vacuum=1, description="Lifting"):
                    return
                # Move to placement position
                if not self._execute_movement([self.dest_x, self.dest_y, 0.0], vacuum=1, description="Moving to placement"):
                    return
                # Place with vacuum release
                if not self._execute_movement([self.dest_x, self.dest_y, 0.112], vacuum=1, description="move z"):
                    return
                if not self._execute_movement([self.dest_x, self.dest_y, 0.112], vacuum=2, description="Placing object"):
                    return
                # Lift after placing
                if not self._execute_movement([self.dest_x, self.dest_y, 0.0], vacuum=0, description="Lifting after placing"):
                    return
                # Return to home position
                if not self._execute_movement([0.0, 0.0, 0.0], vacuum=0, description="Returning home"):
                    return
                print("Completed pnp movement.")

            def pick():
                if not self._execute_movement([self.x, self.y, 0.0], vacuum=0, description="Approaching pick position"):
                    return
                self._execute_movement([self.x, self.y, 0.112], vacuum=1, description="Picking")

            def home():
                self._execute_movement([0.0, 0.0, 0.0], vacuum=0, description="Returning home")

            def auto_pnp():
                # Use self.x and self.y for the object's position
                pick_x = self.x
                pick_y = self.y
                # Pick
                if not self._execute_movement([self.x, self.y, 0.0], vacuum=0, description="Approaching object"):
                    return
                if not self._execute_movement([self.x, self.y, 0.112], vacuum=1, description="Picking object"):
                    return
                # Lift
                if not self._execute_movement([self.x, self.y, 0.0], vacuum=1, description="Lifting object"):
                    return
                # Move to placement position
                if not self._execute_movement([self.dest_x, self.dest_y, 0.0], vacuum=1, description="Moving to placement"):
                    return
                if not self._execute_movement([self.dest_x, self.dest_y, 0.112], vacuum=2, description="Placing object"):
                    return
                # Lift after placing
                if not self._execute_movement([self.dest_x, self.dest_y, 0.0], vacuum=0, description="Lifting after placing"):
                    return
                # Return to home position
                if not self._execute_movement([0.0, 0.0, 0.0], vacuum=0, description="Returning home"):
                    return
                print("Completed auto_pnp movement.")

            # Execute requested movement mode
            movement_functions = {
                "move": move,
                "pnp": pnp,
                "pick": pick,
                "home": home,
                "auto_pnp": auto_pnp
            }
            
            print(f"Starting movement in mode: {self.mode}")
            if self.mode in movement_functions:
                movement_functions[self.mode]()
            else:
                self.movement_status.emit(f"Invalid movement mode: {self.mode}")
            print("Movement function completed.")
        except Exception as e:
            self.movement_status.emit(f"Error: {str(e)}")
            print(f"Exception in MoveRobotThread: {e}")
        finally:
            self.serial_port.close()
            print("MoveRobotThread completed.")
