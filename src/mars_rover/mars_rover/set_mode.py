#!/usr/bin/env python3
"""Switch the drive mode:  ros2 run mars_rover set_mode auto|manual|stop"""
import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String


def main(args=None):
    argv = remove_ros_args(sys.argv)[1:]
    if len(argv) != 1 or argv[0].lower() not in ('auto', 'manual', 'stop'):
        print('usage: ros2 run mars_rover set_mode auto|manual|stop')
        return 1
    rclpy.init(args=args)
    node = rclpy.create_node('set_mode')
    pub = node.create_publisher(String, '/drive_mode', 10)
    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print('cmd_vel_mux is not running (nobody listens on /drive_mode)')
        node.destroy_node()
        rclpy.try_shutdown()
        return 1
    for _ in range(3):
        pub.publish(String(data=argv[0].lower()))
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f'drive mode set to: {argv[0].lower()}')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
