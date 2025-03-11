from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='basic_param_node',
            name='custom_param_node',
            parameters=[
                {'my_str_param': 'from_launch_file'},
                {'my_int_param': 999},
                {'my_double_param': 2.718},
                {'my_bool_param': False}
            ]
        )
    ])
