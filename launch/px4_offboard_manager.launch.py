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
            parameters=[{
                'keyboard_control_enabled': True
            }],
            emulate_tty=True,
        )
    ]) 