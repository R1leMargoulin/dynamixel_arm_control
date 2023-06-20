from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamixel_arm_control',
            executable='hardware_manager_node',
            name='hardware_manager_node'
        ),
        Node(
            package='dynamixel_arm_control',
            executable='control_node',
            name='control_node'
        ),
        Node(
            package='dynamixel_arm_control',
            executable='positions_node',
            name='positions_node'
        ),
        Node(
            package='dynamixel_arm_control',
            executable='moveit_controller_bridge',
            name='moveit_controller_bridge'
        )
    ])
