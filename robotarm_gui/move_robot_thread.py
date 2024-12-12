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
            '/dev/ttyUSB0', '/dev/ttyUSB1',
            '/dev/ttyACM0', '/dev/ttyACM1',
            'COM1', 'COM2', 'COM3', 'COM4', 'COM5'
        ]
        
        for port in possible_ports:
            self.movement_status.emit(f"Trying port {port}...")
            try:
                self.serial_port = serial.Serial(
                    port=port,
                    baudrate=115200,
                    timeout=2,
                    write_timeout=2
                )
                # Clear any pending data
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                
                # Test communication
                self.serial_port.write(b"test\n")
                time.sleep(0.5)  # Give device time to respond
                
                if self.serial_port.in_waiting:
                    response = self.serial_port.readline().decode().strip()
                    if response:  # Only accept if we get actual response
                        self.movement_status.emit(f"Successfully connected to {port}")
                        return True
                
                self.serial_port.close()
                self.movement_status.emit(f"No response from {port}")
                
            except Exception as e:
                self.movement_status.emit(f"Port {port} error: {str(e)}")
                if hasattr(self.serial_port, 'close'):
                    self.serial_port.close()
                continue
        
        self.movement_status.emit("Failed to connect to any serial port")
        raise Exception("Could not connect to any available serial port")

    def _wait_for_completion(self, timeout=10):
        """Wait for 'Movement complete' response from ESP32"""
        valid_responses = ["Movement complete", "OK"]
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            try:
                if self.serial_port.in_waiting:
                    response = self.serial_port.readline().decode().strip()
                    if response in valid_responses:
                        self.movement_status.emit(f"Movement confirmed: {response}")
                        return True
                    elif response:  # Log any other responses
                        self.movement_status.emit(f"Got response: {response}")
            except Exception as e:
                self.movement_status.emit(f"Error reading response: {str(e)}")
                return False
            time.sleep(0.1)
        return False

    def _send_command(self, command):
        """Send command to the robot arm via serial with enhanced error handling."""
        max_retries = 3
        valid_responses = ["OK", "Movement complete"]
        
        for attempt in range(max_retries):
            try:
                if not self.serial_port or not self.serial_port.is_open:
                    self._connect_serial()
                
                # Clear buffers before sending
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                
                # Format and send command
                formatted_command = f"{command}\n"
                self.movement_status.emit(f"Sending command: {formatted_command.strip()}")
                self.serial_port.write(formatted_command.encode())
                
                # Wait for response with timeout
                start_time = time.time()
                while (time.time() - start_time) < 2:  # 2 second timeout
                    if self.serial_port.in_waiting:
                        response = self.serial_port.readline().decode().strip()
                        self.movement_status.emit(f"Got response: {response}")
                        if response in valid_responses:
                            return True
                    time.sleep(0.1)
                
                self.movement_status.emit(f"Timeout waiting for response (attempt {attempt + 1})")
            except Exception as e:
                self.movement_status.emit(f"Error sending command (attempt {attempt + 1}): {str(e)}")
            
            # Wait before retry
            if attempt < max_retries - 1:
                time.sleep(1)
        
        return False

    def _execute_movement(self, positions, vacuum=0, description="Moving", max_attempts=3):
        """Execute movement with enhanced error handling and logging."""
        # Format command with 3 decimal precision
        command = f"x{positions[0]:.3f},y{positions[1]:.3f},z{positions[2]:.3f},v{vacuum}"
        
        for attempt in range(max_attempts):
            self.movement_status.emit(f"{description}: {positions} (Attempt {attempt + 1})")
            
            if self._send_command(command):
                if self._wait_for_completion(timeout=5):  # Increased timeout
                    self.movement_status.emit(f"Movement completed successfully: {command}")
                    return True
                self.movement_status.emit("Movement timeout - no completion confirmation")
            
            if attempt < max_attempts - 1:
                self.movement_status.emit(f"Retrying movement in 1 second... (Attempt {attempt + 1}/{max_attempts})")
                time.sleep(1)
        
        self.movement_status.emit(f"Failed to execute movement after {max_attempts} attempts")
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
