"""Terminal 1: Gazebo Fortress + Mars world + rover spawn + ROS bridge + robot_state_publisher."""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def _first_installed(candidates):
    """Humble + Fortress ships the bridge as ros_gz_* or ros_ign_*; use whichever exists."""
    for name in candidates:
        try:
            get_package_share_directory(name)
            return name
        except PackageNotFoundError:
            continue
    raise RuntimeError(f'None of {candidates} installed. Try: sudo apt install ros-humble-ros-gz '
                       '(or ros-humble-ros-ign)')


def generate_launch_description():
    pkg = get_package_share_directory('mars_rover')
    gz_sim_pkg = _first_installed(('ros_gz_sim', 'ros_ign_gazebo'))
    gz_bridge_pkg = _first_installed(('ros_gz_bridge', 'ros_ign_bridge'))

    with open(os.path.join(pkg, 'urdf', 'mars_rover.urdf'), 'r') as f:
        robot_description = f.read()

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '2', os.path.join(pkg, 'worlds', 'mars_world.sdf')],
        output='screen')

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
               output='screen')

    spawn = TimerAction(period=5.0, actions=[Node(
        package=gz_sim_pkg, executable='create',
        arguments=['-world', 'mars_world', '-name', 'mars_rover', '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '0.5'],
        output='screen')])

    bridge = Node(
        package=gz_bridge_pkg, executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        output='screen')

    return LaunchDescription([gazebo, rsp, bridge, spawn])
