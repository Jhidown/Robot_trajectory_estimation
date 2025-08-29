from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to your ekf_launch.py
    bringup_dir = get_package_share_directory('my_robot_bringup')
    ekf_launch = os.path.join(bringup_dir, 'launch', 'ekf_launch.py')

    return LaunchDescription([
        # CAN bridge
        Node(
            package='can_bridge',
            executable='can_bridge',
            name='can_bridge',
            output='screen'
        ),

        # Camera
        Node(
            package='camera_node',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),

        # LiDAR
        Node(
            package='lidar_reader',
            executable='lidar_node',
            name='lidar_node',
            output='screen'
        ),

        # Motor controller
        Node(
            package='motor_controller',
            executable='motor_node',
            name='motor_node',
            output='screen'
        ),

        # Odometry
        Node(
            package='odom_node',
            executable='odom_node',
            name='odom_node',
            output='screen'
        ),

        # Visual odometry
        Node(
            package='vo_node',
            executable='vo_node',
            name='vo_node',
            output='screen'
        ),

        # EKF launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch)
        )
    ])

