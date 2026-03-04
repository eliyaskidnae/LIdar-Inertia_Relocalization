#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch.conditions

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('lidar_localization_ros2')
    
    # Default paths
    default_params_file = os.path.join(pkg_dir, 'config', 'lio_ndt_fusion_params.yaml')
    default_rviz_config = os.path.join(pkg_dir, 'rviz', 'fusion.rviz')
    
    # Launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to parameters file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz automatically'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    
    # Fusion node
    fusion_node = Node(
        package='lidar_localization_ros2',
        executable='fusion_node',
        name='lio_ndt_fusion_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=launch.conditions.IfCondition(LaunchConfiguration('start_rviz'))
    )
    
    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        start_rviz_arg,
        log_level_arg,
        fusion_node,
        rviz_node
    ])