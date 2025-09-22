

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    agv_nav_dir = get_package_share_directory('agv_nav')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')

    return LaunchDescription([

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[os.path.join(agv_nav_dir, 'config', 'mapper_params_online_sync.yaml')],
            remappings=[('/scan', '/scan')],
        ),

        # YDLidar Driver
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            parameters=[os.path.join(ydlidar_dir, 'params', 'X4-Pro.yaml')],
        ),

        # Static TF: base_link → laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame'],
        ),

        # Odom Node
        Node(
            package='agv_nav',
            executable='odom_controller',
            name='odom_controller',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
