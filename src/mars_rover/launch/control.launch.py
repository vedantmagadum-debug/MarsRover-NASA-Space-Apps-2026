"""Terminal 3: drive control + data.  Everything starts immediately (no timers).

  mode:=auto    obstacle_avoider drives            (default)
  mode:=manual  you drive (keyboard or web buttons)
  mode:=stop    rover held still
You can switch at any time:  ros2 run mars_rover set_mode auto|manual|stop
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    sim = {'use_sim_time': True}

    mux = Node(package='mars_rover', executable='cmd_vel_mux', name='cmd_vel_mux',
               parameters=[sim, {'initial_mode': mode}], output='screen')
    avoider = Node(package='mars_rover', executable='obstacle_avoider', name='obstacle_avoider',
                   parameters=[sim], output='screen')
    server = Node(package='mars_rover', executable='data_server', name='data_server',
                  parameters=[sim, {'port': LaunchConfiguration('port'),
                                    'open_browser': LaunchConfiguration('open_browser'),
                                    'host': LaunchConfiguration('host'),
                                    'allow_control': LaunchConfiguration('allow_control')}],
                  condition=IfCondition(LaunchConfiguration('data_server')), output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='auto', description='auto | manual | stop'),
        DeclareLaunchArgument('data_server', default_value='true'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('host', default_value='127.0.0.1',
                              description='127.0.0.1 = this computer only, 0.0.0.0 = other devices on the network too'),
        DeclareLaunchArgument('allow_control', default_value='true',
                              description='false = read-only data server (safe for a public web page)'),
        DeclareLaunchArgument('open_browser', default_value='true',
                              description='open the dashboard in your browser automatically'),
        mux, avoider, server,
    ])
