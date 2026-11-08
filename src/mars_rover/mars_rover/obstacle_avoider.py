#!/usr/bin/env python3
"""Reactive obstacle avoidance (autonomous driving) for the Mars rover.

Sensor roles
  /scan                 (2D lidar)  -> distance to obstacles in front / left / right
  /camera/image_raw     (camera)    -> "dark rock" pixels in the ground area ahead; early warning
  /imu/data             (IMU)       -> roll/pitch tilt guard
  /odometry/filtered    (EKF)       -> heading hold + goal check (falls back to raw /odom)

Output
  /cmd_vel_auto         Twist       (the cmd_vel_mux decides if it reaches /cmd_vel)
  /obstacle_status      String/JSON (state + all readings, 2 Hz)

The node starts working the moment it is launched: it simply stays idle (state WAITING)
until the first lidar scan and odometry message arrive.
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import String


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        params = {
            'cmd_topic': '/cmd_vel_auto',
            'cruise_speed': 0.35,      # m/s
            'turn_speed': 0.8,         # rad/s
            'stop_dist': 0.9,          # m (from lidar centre; nose is ~0.26 m ahead of it)
            'slow_dist': 2.0,          # m
            'clear_dist': 1.6,         # m, front must be this clear before a turn ends
            'front_half_angle': 20.0,  # deg
            'side_angle': 100.0,       # deg
            'goal_x': 30.0,            # m in the odom frame (change live with ros2 param set)
            'goal_yaw': 0.0,           # rad, preferred heading (+X)
            'tilt_limit': 0.35,        # rad (~20 deg)
            'cam_range': 4.0,          # m, camera only matters if lidar sees something this close
            'cam_sat': 0.55,           # HSV saturation below this = grey (rock); sand is ~0.7
            'cam_val': 0.60,           # brightness below this = dark
            'cam_ratio': 0.10,         # fraction of ROI pixels that must look like rock
        }
        for k, v in params.items():
            self.declare_parameter(k, v)
        self.p = lambda k: self.get_parameter(k).value

        self.front = self.left = self.right = None
        self.scan_time = None
        self.tilt = 0.0
        self.cam_flag = False
        self.cam_frac = 0.0
        self.x = self.yaw = None
        self.filtered_time = None
        self.turning = 0
        self.state = 'WAITING'
        self.last_cmd = Twist()
        self._img_n = 0

        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10)
        self.create_subscription(Odometry, '/odom', self.on_raw_odom, 10)
        self.pub = self.create_publisher(Twist, self.p('cmd_topic'), 10)
        self.status_pub = self.create_publisher(String, '/obstacle_status', 10)
        self.create_timer(0.1, self.control)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(f'obstacle_avoider started -> publishing {self.p("cmd_topic")}, waiting for sensors')

    # ------------------------------------------------------------ callbacks
    def on_scan(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=np.float32)
        ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
        # no return (inf/nan) or too close = treat as free space
        r = np.where(np.isfinite(r) & (r >= msg.range_min), r, msg.range_max)
        f = math.radians(self.p('front_half_angle'))
        s = math.radians(self.p('side_angle'))
        big = float(msg.range_max)
        self.front = float(r[(ang >= -f) & (ang <= f)].min(initial=big))
        self.left = float(r[(ang > f) & (ang <= s)].min(initial=big))
        self.right = float(r[(ang < -f) & (ang >= -s)].min(initial=big))
        self.scan_time = self.get_clock().now()

    def on_imu(self, msg: Imu):
        q = msg.orientation
        if q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 0.5:
            return  # no valid orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self.tilt = max(abs(roll), abs(pitch))

    def _set_pose(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_odom(self, msg: Odometry):
        self._set_pose(msg)
        self.filtered_time = self.get_clock().now()

    def on_raw_odom(self, msg: Odometry):
        # fallback so the rover still drives if the EKF is not running
        if self.filtered_time is None or \
                (self.get_clock().now() - self.filtered_time).nanoseconds * 1e-9 > 1.0:
            self._set_pose(msg)

    def on_image(self, msg: Image):
        self._img_n += 1
        if self._img_n % 3:           # ~5 Hz is plenty
            return
        if msg.encoding not in ('rgb8', 'bgr8') or msg.width == 0:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        h, w = msg.height, msg.width
        # region just below the horizon, centre of the image = where the rover will drive
        roi = img[int(h * 0.55):int(h * 0.85), int(w * 0.30):int(w * 0.70)].astype(np.float32) / 255.0
        mx, mn = roi.max(axis=2), roi.min(axis=2)
        sat = (mx - mn) / np.maximum(mx, 1e-3)
        rock = (sat < self.p('cam_sat')) & (mx < self.p('cam_val'))
        self.cam_frac = float(rock.mean())
        self.cam_flag = self.cam_frac > self.p('cam_ratio')

    # ------------------------------------------------------------- control
    def send(self, cmd: Twist):
        self.last_cmd = cmd
        self.pub.publish(cmd)

    def publish_status(self):
        def r(v, n=2):
            return None if v is None else round(float(v), n)
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'x': r(self.x), 'front': r(self.front), 'left': r(self.left), 'right': r(self.right),
            'cam_rock_fraction': r(self.cam_frac), 'cam_flag': bool(self.cam_flag),
            'tilt_deg': r(math.degrees(self.tilt), 1),
            'turning': self.turning,
            'v': r(self.last_cmd.linear.x), 'w': r(self.last_cmd.angular.z),
            'goal_x': self.p('goal_x'),
        })
        self.status_pub.publish(msg)

    def control(self):
        if self.scan_time is None or self.x is None:
            self.state = 'WAITING'
            return  # still waiting for lidar / odometry
        age = (self.get_clock().now() - self.scan_time).nanoseconds * 1e-9
        if age > 1.0:
            self.state = 'STALE_LIDAR'
            self.get_logger().warn('lidar data is stale -> stopping', throttle_duration_sec=2.0)
            return self.send(Twist())
        if self.x >= self.p('goal_x'):
            if self.state != 'GOAL_REACHED':
                self.get_logger().info(f'goal reached at x={self.x:.1f} m, stopping')
            self.state = 'GOAL_REACHED'
            return self.send(Twist())
        if self.tilt > self.p('tilt_limit'):
            self.state = 'TILT_STOP'
            self.get_logger().warn(f'IMU tilt {math.degrees(self.tilt):.0f} deg too high -> stopping',
                                   throttle_duration_sec=2.0)
            return self.send(Twist())

        cmd = Twist()
        stop_d, slow_d = self.p('stop_dist'), self.p('slow_dist')
        turn_speed, cruise = self.p('turn_speed'), self.p('cruise_speed')

        # sticky avoidance turn: pick the freer side, keep turning until the front is clear
        if self.turning and self.front > self.p('clear_dist'):
            self.turning = 0
        elif not self.turning and self.front < stop_d:
            self.turning = 1 if self.left >= self.right else -1

        if self.turning:
            cmd.linear.x = -0.05 if self.front < 0.5 else 0.0
            cmd.angular.z = self.turning * turn_speed
            self.state = 'AVOID'
        else:
            speed = cruise
            if self.front < slow_d:
                speed *= max(0.3, min(1.0, (self.front - stop_d) / (slow_d - stop_d)))
            self.state = 'CRUISE'
            # camera sees rock ahead AND lidar sees something within range -> be careful
            if self.cam_flag and self.front < self.p('cam_range'):
                speed = min(speed, 0.6 * cruise)
                self.state = 'CAUTION'
            # steer toward the more open side
            steer = 0.7 * (min(self.left, 3.0) - min(self.right, 3.0)) / 3.0
            # when the path ahead is clear, come back to the preferred heading
            if self.front > slow_d and self.yaw is not None:
                steer += max(-0.5, min(0.5, 0.8 * wrap(self.p('goal_yaw') - self.yaw)))
            cmd.linear.x = speed
            cmd.angular.z = max(-turn_speed, min(turn_speed, steer))

        self.send(cmd)
        self.get_logger().info(
            f'{self.state:8s} x={self.x:5.1f}  front={self.front:4.1f} L={self.left:4.1f} R={self.right:4.1f} '
            f'cam={self.cam_frac:4.2f} tilt={math.degrees(self.tilt):4.1f}deg v={cmd.linear.x:4.2f} w={cmd.angular.z:5.2f}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
