#!/usr/bin/env python3
"""Velocity multiplexer: chooses who drives the rover.

  /cmd_vel_auto    (from obstacle_avoider)  -+
  /cmd_vel_manual  (keyboard / web dashboard)+--> mux --> /cmd_vel  (Gazebo diff drive)

Mode is set by publishing a String to /drive_mode:  auto | manual | stop
The current mode is published on /drive_mode_status (2 Hz and on every change).
Switching mode always sends one zero command first, so the rover never lurches.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

MODES = ('auto', 'manual', 'stop')


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        self.declare_parameter('initial_mode', 'auto')
        self.declare_parameter('auto_timeout', 1.0)     # s, stale autonomous command -> stop
        self.declare_parameter('manual_timeout', 0.0)   # s, 0 = keep last manual command (keyboard teleop)
        self.declare_parameter('rate', 20.0)

        mode = self.get_parameter('initial_mode').value
        self.mode = mode if mode in MODES else 'auto'
        self.auto_cmd, self.auto_t = Twist(), None
        self.man_cmd, self.man_t = Twist(), None

        self.create_subscription(Twist, '/cmd_vel_auto', self.on_auto, 10)
        self.create_subscription(Twist, '/cmd_vel_manual', self.on_manual, 10)
        self.create_subscription(String, '/drive_mode', self.on_mode, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/drive_mode_status', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.tick)
        self.create_timer(0.5, self.publish_mode)
        self.get_logger().info(f'cmd_vel_mux started in "{self.mode}" mode')

    def now(self):
        return self.get_clock().now()

    def age(self, t):
        return float('inf') if t is None else (self.now() - t).nanoseconds * 1e-9

    def on_auto(self, msg):
        self.auto_cmd, self.auto_t = msg, self.now()

    def on_manual(self, msg):
        self.man_cmd, self.man_t = msg, self.now()

    def on_mode(self, msg):
        new = msg.data.strip().lower()
        if new not in MODES:
            self.get_logger().warn(f'unknown mode "{msg.data}", use one of {MODES}')
            return
        if new != self.mode:
            self.get_logger().info(f'mode: {self.mode} -> {new}')
            self.mode = new
            self.man_cmd = Twist()       # forget old manual command
            self.pub.publish(Twist())    # zero once
        self.publish_mode()

    def publish_mode(self):
        self.mode_pub.publish(String(data=self.mode))

    def tick(self):
        cmd = Twist()
        if self.mode == 'auto':
            if self.age(self.auto_t) <= float(self.get_parameter('auto_timeout').value):
                cmd = self.auto_cmd
        elif self.mode == 'manual':
            mt = float(self.get_parameter('manual_timeout').value)
            if mt <= 0.0 or self.age(self.man_t) <= mt:
                cmd = self.man_cmd
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
