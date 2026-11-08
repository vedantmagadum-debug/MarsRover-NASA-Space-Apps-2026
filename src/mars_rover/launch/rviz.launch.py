"""Terminal 4 (optional): RViz with map, lidar, camera, robot model, fused odometry."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('mars_rover'), 'rviz', 'mars_rover.rviz')
    return LaunchDescription([Node(package='rviz2', executable='rviz2', arguments=['-d', cfg],
                                   parameters=[{'use_sim_time': True}], output='screen')])
