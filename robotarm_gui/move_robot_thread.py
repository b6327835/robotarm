from PyQt5.QtCore import QThread, pyqtSignal
import serial
import time

class MoveRobotThread(QThread):
    movement_status = pyqtSignal(str)  # Signal to update UI with movement status
    positions_update = pyqtSignal(float, float, float)

    def __init__(self, x, y, z, mode, parent=None, dest_x=0.0, dest_y=0.0):
        super().__init__(parent)
        # Remove the *100 multiplication since values are already in correct scale
        self.offsetx = 0
        self.offsety = 0
        self.x = x + self.offsetx
        self.y = y + self.offsety
        self.z = z
        self.dest_x = dest_x
        self.dest_y = dest_y
        self.mode = mode
        self.serial_port = None
        self.z_top = 170
        self.z_bottom = 148

    def _connect_serial(self):
        """Only connect if no existing connection"""
        if self.serial_port and self.serial_port.is_open:
            return True
            
        possible_ports = [
            '/dev/ttyUSB0', '/dev/ttyUSB1',
            '/dev/ttyACM0', '/dev/ttyACM1',
            'COM1', 'COM2', 'COM3', 'COM4', 'COM5'
        ]
        
        for port in possible_ports:
            self.movement_status.emit(f"Trying port {port}...")
            try:
                self.serial_port = serial.Serial()
                self.serial_port.port = port
                self.serial_port.baudrate = 115200
                self.serial_port.timeout = 2
                self.serial_port.write_timeout = 2
                
                # Disable flow control before opening
                self.serial_port.dsrdtr = False
                self.serial_port.rtscts = False
                
                # Open port
                self.serial_port.open()
                
                # Set DTR/RTS after opening
                self.serial_port.dtr = False
                self.serial_port.rts = False
                time.sleep(0.1)  # Allow signals to stabilize
                
                # Clear buffers
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                
                # Test communication
                self.serial_port.write(b"test\n")
                time.sleep(0.2)
                
                if self.serial_port.in_waiting:
                    response = self.serial_port.readline().decode().strip()
                    if response:
                        self.movement_status.emit(f"Successfully connected to {port}")
                        return True
                
                # Proper closing sequence
                self.serial_port.dtr = False
                self.serial_port.rts = False
                time.sleep(0.1)
                self.serial_port.close()
                self.movement_status.emit(f"No response from {port}")
                
            except Exception as e:
                self.movement_status.emit(f"Port {port} error: {str(e)}")
                if hasattr(self.serial_port, 'close'):
                    self.serial_port.dtr = False
                    self.serial_port.rts = False
                    time.sleep(0.1)
                    self.serial_port.close()
                continue

        self.movement_status.emit("Failed to connect to any serial port")
        raise Exception("Could not connect to any available serial port")

    def _wait_for_completion(self, timeout=10):
        """Wait for 'Movement complete' response from ESP32."""
        valid_responses = ["Movement complete", "OK"]
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            try:
                if self.serial_port.in_waiting:
                    raw_response = self.serial_port.readline()
                    response = raw_response.decode('utf-8', errors='ignore').strip()
                    print(f"[SERIAL RECV] <<< {repr(response)}")
                    self.movement_status.emit(f"Got response: {response}")
                    # Ensure exact match
                    if any(response.lower() == valid_resp.lower() for valid_resp in valid_responses):
                        self.movement_status.emit(f"Movement confirmed: {response}")
                        return True
                    # Parse other responses
                    self._parse_response(response)
                time.sleep(0.07)
            except Exception as e:
                print(f"[SERIAL ERROR] {str(e)}")
                self.movement_status.emit(f"Error reading response: {str(e)}")
                return False
        print("[SERIAL TIMEOUT] Movement completion timeout")
        return False

    def _send_command(self, command):
        """Send command to the robot arm via serial with enhanced error handling."""
        max_retries = 2
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
                print(f"\n[SERIAL SEND] >>> {formatted_command.strip()}")
                self.movement_status.emit(f"Sending command: {formatted_command.strip()}")
                self.serial_port.write(formatted_command.encode())
                
                # Wait for response with timeout
                start_time = time.time()
                while (time.time() - start_time) < 5:  # 5 second timeout
                    if self.serial_port.in_waiting:
                        raw_response = self.serial_port.readline()
                        response = raw_response.decode('utf-8', errors='ignore').strip()
                        print(f"[SERIAL RECV] <<< {response}")
                        self.movement_status.emit(f"Got response: {response}")
                        if response in valid_responses:
                            return True
                        # Parse other responses
                        self._parse_response(response)
                    time.sleep(0.07)
                
                print("[SERIAL TIMEOUT] No response received")
                self.movement_status.emit(f"Timeout waiting for response (attempt {attempt + 1})")
            except Exception as e:
                print(f"[SERIAL ERROR] {str(e)}")
                self.movement_status.emit(f"Error sending command (attempt {attempt + 1}): {str(e)}")
            
            # Wait before retry
            if attempt < max_retries - 1:
                time.sleep(0.5)
        
        return False

    def _execute_movement(self, positions, vacuum=0, description="Moving", max_attempts=2):
        """Execute movement with enhanced error handling and logging."""
        # Special handling for HOME command
        if positions == "INIT":
            command = "INIT"
        else:
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
                time.sleep(0.5)
        
        self.movement_status.emit(f"Failed to execute movement after {max_attempts} attempts")
        return False

    def _parse_response(self, response):
        """Parse 'Current positions - X:8 Y:0 Z:7136' and emit signal."""
        if "Current positions" in response:
            try:
                # Example format: "Current positions - X:8 Y:0 Z:7136"
                parts = response.split("X:")[1].split("Y:")
                x_val = float(parts[0].strip())
                parts2 = parts[1].split("Z:")
                y_val = float(parts2[0].strip())
                z_val = float(parts2[1].strip())
                self.positions_update.emit(x_val, y_val, z_val)
            except:
                pass

    def run(self):
        serial_owned = False
        try:
            if not self.serial_port:
                self._connect_serial()
                serial_owned = True
            
            def move():
                print("[DEBUG] Executing move command...")
                return self._execute_movement([self.x, self.y, self.z], vacuum=0, description="Moving to position", max_attempts=3)
            
            def pnp():
                pnp_x = self.x
                pnp_y = self.y
                # Calculate pick and place heights separately
                pick_z = self.z_top if pnp_x > 169 else self.z_bottom
                place_z = self.z_top if self.dest_x > 169 else self.z_bottom
                
                # Approach position from above
                if not self._execute_movement([pnp_x, pnp_y, 0.0], vacuum=0, description="Approaching position"):
                    return
                # Pick with vacuum on then off
                if not self._execute_movement([pnp_x, pnp_y, pick_z], vacuum=0, description="move z"):
                    return
                if not self._execute_movement([pnp_x, pnp_y, pick_z], vacuum=1, description="Picking"):
                    return
                if not self._execute_movement([pnp_x, pnp_y, pick_z], vacuum=0, description="Picking"):
                    return
                # Lift with item
                if not self._execute_movement([pnp_x, pnp_y, 0.0], vacuum=0, description="Lifting"):
                    return
                # Move to placement position
                if not self._execute_movement([self.dest_x, self.dest_y, 0.0], vacuum=0, description="Moving to placement"):
                    return
                # Place with vacuum release
                if not self._execute_movement([self.dest_x, self.dest_y, place_z], vacuum=0, description="move z"):
                    return
                if not self._execute_movement([self.dest_x, self.dest_y, place_z], vacuum=2, description="Placing object"):
                    return
                if not self._execute_movement([self.dest_x, self.dest_y, place_z], vacuum=0, description="Placing object"):
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
                self._execute_movement([self.x, self.y, self.z_bottom], vacuum=1, description="Picking")

            def home():
                self._execute_movement([0,0,0], vacuum=0, description="Returning home")
            def initial():
                self._execute_movement("HOME", vacuum=0, description="Initial position")
            def auto_pnp():
                # Use self.x and self.y for the object's position
                pick_x = self.x
                pick_y = self.y
                # Pick
                if not self._execute_movement([self.x, self.y, 0.0], vacuum=0, description="Approaching object"):
                    return
                if not self._execute_movement([self.x, self.y, self.z_bottom], vacuum=1, description="Picking object"):
                    return
                # Lift
                if not self._execute_movement([self.x, self.y, 0.0], vacuum=1, description="Lifting object"):
                    return
                # Move to placement position
                if not self._execute_movement([self.dest_x, self.dest_y, 0.0], vacuum=1, description="Moving to placement"):
                    return
                if not self._execute_movement([self.dest_x, self.dest_y, self.z_bottom], vacuum=2, description="Placing object"):
                    return
                # Lift after placing
                if not self._execute_movement([self.dest_x, self.dest_y, 0.0], vacuum=0, description="Lifting after placing"):
                    return
                # Return to home position
                if not self._execute_movement([0.0, 0.0, 0.0], vacuum=0, description="Returning home"):
                    return
                print("Completed auto_pnp movement.")

            # Add new pnp2 movement function
            def pnp2():
                """Optimized pick and place without returning home between operations"""
                pnp_x = self.x
                pnp_y = self.y
                # Calculate pick and place heights based on position
                pick_z = self.z_top if pnp_x > 169 else self.z_bottom
                place_z = self.z_top if self.dest_x > 169 else self.z_bottom
                
                # Approach position from above
                if not self._execute_movement([pnp_x, pnp_y, 0.0], vacuum=0, description="Approaching position"):
                    return False
                # Lower to pick
                if not self._execute_movement([pnp_x, pnp_y, pick_z], vacuum=0, description="Lowering to pick"):
                    return False
                # Pick with vacuum
                if not self._execute_movement([pnp_x, pnp_y, pick_z], vacuum=1, description="Picking"):
                    return False
                # Release vacuum hold but maintain position
                if not self._execute_movement([pnp_x, pnp_y, pick_z], vacuum=0, description="Securing grip"):
                    return False
                # Lift object
                if not self._execute_movement([pnp_x, pnp_y, 0.0], vacuum=0, description="Lifting"):
                    return False
                # Move to placement position
                if not self._execute_movement([self.dest_x, self.dest_y, 0.0], vacuum=0, description="Moving to placement"):
                    return False
                # Lower to place
                if not self._execute_movement([self.dest_x, self.dest_y, place_z], vacuum=0, description="Lowering to place"):
                    return False
                # Release object
                if not self._execute_movement([self.dest_x, self.dest_y, place_z], vacuum=2, description="Releasing"):
                    return False
                # Confirm release
                if not self._execute_movement([self.dest_x, self.dest_y, place_z], vacuum=0, description="Confirming release"):
                    return False
                # Lift after placing
                if not self._execute_movement([self.dest_x, self.dest_y, 0.0], vacuum=0, description="Lifting after place"):
                    return False
                return True

            # Execute requested movement mode
            movement_functions = {
                "move": move,
                "pnp": pnp,
                "pick": pick,
                "home": home,
                "init": initial,
                "auto_pnp": auto_pnp,
                "pnp2": pnp2  # Add new movement mode
            }
            
            print(f"Starting movement in mode: {self.mode}")
            if self.mode in movement_functions:
                result = movement_functions[self.mode]()
                print(f"Movement result: {result}")
            else:
                self.movement_status.emit(f"Invalid movement mode: {self.mode}")
            print("Movement function completed.")
            
        except Exception as e:
            self.movement_status.emit(f"Error: {str(e)}")
            print(f"Exception in MoveRobotThread: {e}")
        finally:
            # Only close if we created the connection
            if serial_owned and self.serial_port and self.serial_port.is_open:
                print("Closing thread-owned serial connection")
                # Do not set DTR/RTS when closing; just close the port
                self.serial_port.close()
            print("MoveRobotThread completed.")
