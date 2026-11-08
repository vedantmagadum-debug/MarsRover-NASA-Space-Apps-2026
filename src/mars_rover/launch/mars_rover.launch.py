"""Everything in one command:  ros2 launch mars_rover mars_rover.launch.py [mode:=auto|manual|stop]
                                                                      [rviz:=false] [data_server:=false] [port:=8080]
For separate terminals use sim / fusion_slam / control / rviz launch files instead."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    d = os.path.join(get_package_share_directory('mars_rover'), 'launch')

    def inc(name, **kw):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(d, name)), **kw)

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='auto', description='auto | manual | stop'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('data_server', default_value='true'),
        DeclareLaunchArgument('port', default_value='8080'),
        inc('sim.launch.py'),
        inc('fusion_slam.launch.py'),
        inc('control.launch.py', launch_arguments={
            'mode': LaunchConfiguration('mode'),
            'data_server': LaunchConfiguration('data_server'),
            'port': LaunchConfiguration('port')}.items()),
        inc('rviz.launch.py', condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
