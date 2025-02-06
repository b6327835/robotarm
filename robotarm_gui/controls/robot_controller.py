import serial
import time

class RobotController:
    def __init__(self):
        self.serial_port = None
        self.connect()
    
    def connect(self):
        """Connect to the robot's serial port"""
        if self.serial_port and self.serial_port.is_open:
            return True
            
        port = '/dev/ttyUSB0'  # Your CP2102 port
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
            
            # Set DTR/RTS after opening to prevent reset
            self.serial_port.dtr = False
            self.serial_port.rts = False
            time.sleep(0.1)  # Allow signals to stabilize
            
            # Clear buffers
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def close(self):
        """Close the serial connection"""
        if self.serial_port and self.serial_port.is_open:
            # Set DTR/RTS False before closing to prevent reset
            self.serial_port.dtr = False
            self.serial_port.rts = False
            time.sleep(0.1)  # Allow signals to stabilize
            self.serial_port.close()
