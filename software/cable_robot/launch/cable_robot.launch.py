from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('cable_robot'),
        'config',
        'single_dof_controller.yaml'
    )

    return LaunchDescription([
        Node(
            package='cable_robot',
            executable='cable_robot_node',
            name='cable_robot_node',
            output='screen',
            parameters=[config],
            emulate_tty=True
        ),
    ])
