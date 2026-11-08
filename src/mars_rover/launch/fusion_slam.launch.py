"""Terminal 2: sensor fusion (EKF: wheel odom + IMU gyro) and lidar SLAM (/map)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mars_rover')
    ekf = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
               parameters=[os.path.join(pkg, 'config', 'ekf.yaml'), {'use_sim_time': True}],
               output='screen')
    slam = Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                parameters=[os.path.join(pkg, 'config', 'slam_toolbox.yaml'), {'use_sim_time': True}],
                output='screen')
    return LaunchDescription([ekf, slam])
