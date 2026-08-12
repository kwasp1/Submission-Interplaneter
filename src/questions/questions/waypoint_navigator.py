#!/usr/bin/env python3
"""
Navigation node: connects to the WebSocket waypoint broadcaster,
parses (x, y, yaw) waypoints, and drives the TurtleBot4 to each one
sequentially, with a LIDAR-based reactive obstacle escape maneuver.
"""
import math
import json
import threading
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

import websockets


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.waypoints = []
        self.waypoints_lock = threading.Lock()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.pose_received = False

        self.current_waypoint_index = 0
        self.mission_complete = False

        self.distance_tolerance = 0.15
        self.yaw_tolerance = 0.08
        self.max_linear_speed = 0.45
        self.max_angular_speed = 1.0
        self.turn_first_threshold = 0.3

        self.front_obstacle_distance = float('inf')
        self.left_clearance = float('inf')
        self.right_clearance = float('inf')
        self.contact_floor = 0.3  # meters - obstacle detection trigger distance

        # Escape maneuver: back up -> turn 90 right -> drive forward -> resume.
        # Driven by measured odometry (position/yaw change), not elapsed
        # time, so it works correctly regardless of sim speed.
        self.avoidance_phase = None  # None, 'backing_up', 'turning', 'translating'
        self.avoidance_start_x = None
        self.avoidance_start_y = None
        self.avoidance_start_yaw = None
        self.BACKUP_DISTANCE = 0.1
        self.BACKUP_SPEED = -0.15
        self.TURN_TARGET_RAD = math.radians(88.0)
        self.TRANSLATE_DISTANCE = 0.5
        self.TRANSLATE_SPEED = 0.3

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, '/diffdrive_controller/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/sim_ground_truth_pose', self.odom_callback,
            qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback,
            qos_profile_sensor_data)
        self.nav_complete_pub = self.create_publisher(
            Bool, '/navigation_complete', 10)

        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.ws_thread = threading.Thread(target=self.run_ws_client, daemon=True)
        self.ws_thread.start()

        self.get_logger().info('Waypoint navigator started.')

    def run_ws_client(self):
        asyncio.run(self.ws_client_loop())

    async def ws_client_loop(self):
        uri = "ws://localhost:8765"
        while rclpy.ok():
            try:
                async with websockets.connect(uri) as websocket:
                    self.get_logger().info(f'Connected to {uri}')
                    async for message in websocket:
                        data = json.loads(message)
                        wps = data.get('waypoints', [])
                        with self.waypoints_lock:
                            if not self.waypoints:
                                processed = list(wps)
                                # Extra approach point centered on the sphere
                                # cluster, ~1m back, so all three spheres are
                                # roughly equidistant from the camera for
                                # accurate size comparison.
                                processed.append(
                                    {"x": 4.9, "y": 2.0, "yaw": 1.5708})
                                self.waypoints = processed
                                self.get_logger().info(
                                    f'Received {len(wps)} waypoints '
                                    f'({len(processed)} total).')
            except Exception as e:
                self.get_logger().warn(f'WebSocket error: {e}. Retrying in 2s...')
                await asyncio.sleep(2.0)

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.pose_received = True

    def scan_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0:
            return

        def index_for_angle(angle_rad):
            idx = int(round((angle_rad - msg.angle_min) / msg.angle_increment))
            return max(0, min(n - 1, idx))

        def min_range_in_window(center_angle_rad, width_deg):
            center_idx = index_for_angle(center_angle_rad)
            half_width = max(1, int(math.radians(width_deg) / msg.angle_increment))
            lo = max(0, center_idx - half_width)
            hi = min(n, center_idx + half_width + 1)
            window = msg.ranges[lo:hi]
            valid = [r for r in window
                     if msg.range_min <= r <= msg.range_max and not math.isnan(r)]
            return min(valid) if valid else float('inf')

        # LiDAR is mounted with a +90 deg yaw offset from the robot's base
        # frame (confirmed via tf2_echo). Correct for it here.
        LIDAR_YAW_OFFSET_DEG = 90.0

        def base_angle_to_lidar_angle(base_deg):
            return math.radians(base_deg - LIDAR_YAW_OFFSET_DEG)

        self.front_obstacle_distance = min_range_in_window(
            base_angle_to_lidar_angle(0.0), 20.0)
        self.left_clearance = min_range_in_window(
            base_angle_to_lidar_angle(90.0), 25.0)
        self.right_clearance = min_range_in_window(
            base_angle_to_lidar_angle(-90.0), 25.0)

    def control_loop(self):
        if self.mission_complete:
            return
        if not self.pose_received:
            return

        with self.waypoints_lock:
            waypoints = self.waypoints

        if not waypoints:
            return

        if self.current_waypoint_index >= len(waypoints):
            self.stop_robot()
            self.mission_complete = True
            self.get_logger().info('All waypoints reached. Mission complete.')
            settle_timer = self.create_timer(1.5, self.publish_nav_complete_once)
            self.settle_timer = settle_timer
            return

        # --- Obstacle escape maneuver ---
        if self.avoidance_phase is None and self.front_obstacle_distance < self.contact_floor:
            self.avoidance_phase = 'backing_up'
            self.avoidance_start_x = self.current_x
            self.avoidance_start_y = self.current_y
            self.get_logger().info(
                f'Object detected at {self.front_obstacle_distance:.2f}m ahead - '
                f'starting escape maneuver.')

        if self.avoidance_phase is not None:
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = 'base_link'

            if self.avoidance_phase == 'backing_up':
                traveled = math.hypot(self.current_x - self.avoidance_start_x,
                                       self.current_y - self.avoidance_start_y)
                if traveled < self.BACKUP_DISTANCE:
                    twist.twist.linear.x = self.BACKUP_SPEED
                    twist.twist.angular.z = 0.0
                else:
                    self.avoidance_phase = 'turning'
                    self.avoidance_start_yaw = self.current_yaw
                    twist.twist.linear.x = 0.0
                    twist.twist.angular.z = -self.max_angular_speed

            elif self.avoidance_phase == 'turning':
                rotated = abs(normalize_angle(self.current_yaw - self.avoidance_start_yaw))
                if rotated < self.TURN_TARGET_RAD:
                    twist.twist.linear.x = 0.0
                    twist.twist.angular.z = -self.max_angular_speed
                else:
                    self.avoidance_phase = 'translating'
                    self.avoidance_start_x = self.current_x
                    self.avoidance_start_y = self.current_y
                    twist.twist.linear.x = self.TRANSLATE_SPEED
                    twist.twist.angular.z = 0.0

            elif self.avoidance_phase == 'translating':
                traveled = math.hypot(self.current_x - self.avoidance_start_x,
                                       self.current_y - self.avoidance_start_y)
                if traveled < self.TRANSLATE_DISTANCE:
                    twist.twist.linear.x = self.TRANSLATE_SPEED
                    twist.twist.angular.z = 0.0
                else:
                    self.avoidance_phase = None
                    self.get_logger().info(
                        f'Escape maneuver complete - resuming navigation '
                        f'to waypoint {self.current_waypoint_index}.')
                    twist.twist.linear.x = 0.0
                    twist.twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)
            return

        # --- Normal go-to-goal ---
        target = waypoints[self.current_waypoint_index]
        target_x = target['x']
        target_y = target['y']
        target_yaw = target.get('yaw', None)

        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.hypot(dx, dy)
        heading_to_target = math.atan2(dy, dx)
        heading_error = normalize_angle(heading_to_target - self.current_yaw)

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'

        if distance > self.distance_tolerance:
            if abs(heading_error) > self.turn_first_threshold:
                twist.twist.angular.z = max(-self.max_angular_speed,
                                       min(self.max_angular_speed, 2.0 * heading_error))
                twist.twist.linear.x = 0.0
            else:
                if distance > 0.5:
                    twist.twist.linear.x = self.max_linear_speed
                else:
                    twist.twist.linear.x = max(0.08, min(self.max_linear_speed, 1.2 * distance))
                twist.twist.angular.z = max(-self.max_angular_speed,
                                       min(self.max_angular_speed, 1.5 * heading_error))
        elif target_yaw is not None and abs(normalize_angle(target_yaw - self.current_yaw)) > self.yaw_tolerance:
            yaw_error = normalize_angle(target_yaw - self.current_yaw)
            twist.twist.angular.z = max(-self.max_angular_speed,
                                   min(self.max_angular_speed, 2.0 * yaw_error))
            twist.twist.linear.x = 0.0
        else:
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_index}: '
                f'({target_x:.2f}, {target_y:.2f})')
            self.current_waypoint_index += 1
            self.stop_robot()
            return

        slow_zone = self.contact_floor * 2.0
        if self.front_obstacle_distance < slow_zone:
            scale = ((self.front_obstacle_distance - self.contact_floor) /
                     (slow_zone - self.contact_floor))
            twist.twist.linear.x *= max(0.0, min(1.0, scale))

        self.cmd_vel_pub.publish(twist)

    def publish_nav_complete_once(self):
        msg = Bool()
        msg.data = True
        self.nav_complete_pub.publish(msg)
        self.get_logger().info('Published navigation_complete=True')
        self.settle_timer.cancel()

    def stop_robot(self):
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        self.cmd_vel_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
