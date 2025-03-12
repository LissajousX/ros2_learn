#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    # 获取当前包路径
    pkg_dir = os.path.dirname(os.path.realpath(__file__))
    
    # 定义参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_file = os.path.join(pkg_dir, 'simple_robot.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'simple_world.sdf')
    
    # 解析XACRO文件
    robot_description_content = Command(
        ['xacro ', urdf_file]
    )
    
    # 机器人状态发布节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    # 启动Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # 生成URDF模型的SDF格式
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                  '-entity', 'simple_robot',
                  '-x', '0.0',
                  '-y', '0.0',
                  '-z', '0.1'],
        output='screen'
    )
    
    # 启动RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'robot_simulation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=LaunchConfiguration('rviz', default='true')
    )
    
    # 确保在Gazebo完全启动后再生成机器人模型
    spawn_entity_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo,
            on_exit=[spawn_entity]
        )
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity_event,
        rviz
    ])
