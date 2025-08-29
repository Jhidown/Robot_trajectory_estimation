#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, PoseStamped
import time
import math


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Converts yaw angle (rad) to ROS quaternion"""
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.subscription = self.create_subscription(
            Frame,
            '/can_rx',
            self.can_callback,
            10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/odom_path', 10)

        # Robot parameters
        self.wheel_radius = 0.075  # Wheel radius (m)
        self.wheel_base = 0.3      # Distance between wheels (m)
        
        # Previous states
        self.last_pos_l = None
        self.last_pos_r = None
        self.last_time = self.get_clock().now()
        
        # Pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.accum_distance = 0.0
        self.last_publish_time = time.time()
        self.publish_distance_threshold = 0.3  # 30 cm
        self.publish_time_guard = 9  # 9 s

        # Store latest wheel speeds (rad/s)
        self.speed_l = 0.0
        self.speed_r = 0.0

        # Covariances
        self.pose_covariance = [
            0.01,     0.0,     0.0,     0.0,     0.0,    0.0,
             0.0,    0.01,     0.0,     0.0,     0.0,    0.0,
             0.0,     0.0, 99999.0,     0.0,     0.0,    0.0,
             0.0,     0.0,     0.0, 99999.0,     0.0,    0.0,
             0.0,     0.0,     0.0,     0.0, 99999.0,    0.0,
             0.0,     0.0,     0.0,     0.0,     0.0,   0.05
        ]
        self.twist_covariance = [
            0.01,     0.0,     0.0,     0.0,     0.0,    0.0,
             0.0,    0.01,     0.0,     0.0,     0.0,    0.0,
             0.0,     0.0, 99999.0,     0.0,     0.0,    0.0,
             0.0,     0.0,     0.0, 99999.0,     0.0,    0.0,
             0.0,     0.0,     0.0,     0.0, 99999.0,    0.0,
             0.0,     0.0,     0.0,     0.0,     0.0,   0.05
        ]

        # Path storage : frame = "map" (comme VO)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

    def can_callback(self, msg: Frame):
        if msg.id not in [0x381, 0x382]:
            return

        # Decode data
        pos = int.from_bytes(msg.data[0:4], byteorder='little', signed=True)
        speed_raw = int.from_bytes(msg.data[4:8], byteorder='little', signed=True)

        # Convert speed to m/s
        speed_m_s = speed_raw * self.wheel_radius

        if msg.id == 0x381:  # Right wheel
            delta_r = 0 if self.last_pos_r is None else pos - self.last_pos_r
            self.last_pos_r = pos
            self.delta_r = delta_r
            self.speed_r = speed_m_s
        elif msg.id == 0x382:  # Left wheel
            delta_l = 0 if self.last_pos_l is None else pos - self.last_pos_l
            self.last_pos_l = pos
            self.delta_l = delta_l
            self.speed_l = speed_m_s

        # Odometry update if both deltas exist
        if hasattr(self, 'delta_l') and hasattr(self, 'delta_r'):
            dl_m = self.delta_l * self.wheel_radius
            dr_m = self.delta_r * self.wheel_radius
            dc = (dr_m + dl_m) / 2.0
            dtheta = (dr_m - dl_m) / self.wheel_base

            self.x += dc * math.cos(self.theta + dtheta/2)
            self.y += dc * math.sin(self.theta + dtheta/2)
            self.theta += dtheta
            self.accum_distance += abs(dc)

            # Linear & angular velocities
            v = (self.speed_r + self.speed_l) / 2.0
            w = (self.speed_r - self.speed_l) / self.wheel_base

            # Publish odometry based on distance or time
            now_time = time.time()
            if self.accum_distance >= self.publish_distance_threshold or (now_time - self.last_publish_time) > self.publish_time_guard:
                now = self.get_clock().now().to_msg()

                odom_msg = Odometry()
                odom_msg.header.stamp = now
                odom_msg.header.frame_id = 'odom'        # toujours "odom" pour Odometry
                odom_msg.child_frame_id = 'base_link'

                # Pose
                odom_msg.pose.pose.position.x = self.x
                odom_msg.pose.pose.position.y = self.y
                odom_msg.pose.pose.position.z = 0.0
                odom_msg.pose.pose.orientation = yaw_to_quaternion(self.theta)
                odom_msg.pose.covariance = self.pose_covariance

                # Twist
                odom_msg.twist.twist.linear.x = v
                odom_msg.twist.twist.angular.z = w
                odom_msg.twist.covariance = self.twist_covariance

                self.odom_pub.publish(odom_msg)
                self.accum_distance = 0.0
                self.last_publish_time = now_time

                # --- Path en frame "map" pour debug / visualisation
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = now
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose = odom_msg.pose.pose

                self.path_msg.header.stamp = now
                self.path_msg.poses.append(pose_stamped)
                self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

