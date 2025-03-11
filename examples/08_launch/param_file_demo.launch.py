import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # u83b7u53d6u53c2u6570u6587u4ef6u8defu5f84
    config_file = os.path.join(
        get_package_share_directory('my_robot_pkg'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='basic_param_node',
            name='custom_param_node',
            parameters=[config_file]
        )
    ])
