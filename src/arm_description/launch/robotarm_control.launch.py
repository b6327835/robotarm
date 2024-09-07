from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=['/home/mark/robotarm/src/robotarm_description/config/controllers.yaml']
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawner_x_joint_position_controller',
            output='screen',
            arguments=['x_joint_position_controller', '--controller-manager', '/controller_manager']
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawner_y_joint_position_controller',
            output='screen',
            arguments=['y_joint_position_controller', '--controller-manager', '/controller_manager']
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawner_z_joint_position_controller',
            output='screen',
            arguments=['z_joint_position_controller', '--controller-manager', '/controller_manager']
        ),
    ])

