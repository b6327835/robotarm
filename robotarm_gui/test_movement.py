from robot_controller import RobotController
from move_robot_thread import MoveRobotThread
import time
import serial.tools.list_ports

def list_available_ports():
    """List all available serial ports"""
    ports = serial.tools.list_ports.comports()
    print("\nAvailable ports:")
    for port in ports:
        print(f"- {port.device}: {port.description}")
    print()

def main():
    try:
        # Print available ports for debugging
        list_available_ports()
        
        print("Attempting to connect to robot...")
        
        # Initialize robot controller
        controller = RobotController()
        print("Serial connection opened:", controller.serial_port.port if controller.serial_port else "Not connected")
        
        # Move to test position
        print("\n=== First Movement ===")
        robot = MoveRobotThread(x=100.0, y=100.0, z=100.0, mode="move")
        robot.serial_port = controller.serial_port
        robot.start()
        robot.wait()
        print("First movement completed")
        
        time.sleep(0.5)
        
        # Move back home
        print("\n=== Second Movement ===")
        robot = MoveRobotThread(x=0.0, y=0.0, z=0.0, mode="move")
        robot.serial_port = controller.serial_port
        robot.start()
        robot.wait()
        print("Second movement completed")
        
        print("\nTest complete!")
        
    except Exception as e:
        print(f"\nError occurred: {e}")
        print("Please check:")
        print("1. The robot arm is properly connected")
        print("2. You have permission to access serial ports (try 'sudo chmod 666 /dev/ttyUSB0')")
        print("3. The correct port is available")
    finally:
        if 'controller' in locals():
            print("Closing serial connection...")
            controller.close()
            print("Serial connection closed")

if __name__ == "__main__":
    main()
