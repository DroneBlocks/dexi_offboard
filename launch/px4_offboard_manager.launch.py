from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dexi_offboard',
            namespace='dexi',
            executable='px4_offboard_manager',
            name='px4_offboard_manager',
            output='screen',
            emulate_tty=True,
        )
    ]) 