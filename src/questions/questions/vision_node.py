#!/usr/bin/env python3
"""
Vision node: subscribes to the TurtleBot4's camera feed, segments the
three colored spheres (red, green, yellow) using HSV thresholding,
and reports the color of the LARGEST by TRUE physical size (not just
pixel area), using the depth image + camera intrinsics to convert
pixel radius -> real-world radius. This makes the result correct
regardless of the robot's exact stopping distance from each sphere.

Only starts processing once it receives navigation_complete=True from
the navigator, so it never locks onto a premature/wrong view.
"""
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge


class SphereColorDetector(Node):
    def __init__(self):
        super().__init__('sphere_color_detector')

        self.bridge = CvBridge()

        self.min_area = 150  # minimum pixel area to count as a real detection
        self.reported = False
        self.navigation_complete = False

        # Camera intrinsics (filled in once camera_info arrives)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.latest_depth = None  # most recent depth frame, as float32 meters

        # HSV color ranges for the three known sphere colors.
        self.color_ranges = {
            'red': [
                (np.array([0, 100, 80]), np.array([10, 255, 255])),
                (np.array([170, 100, 80]), np.array([179, 255, 255])),
            ],
            'green': [
                (np.array([40, 70, 60]), np.array([85, 255, 255])),
            ],
            'yellow': [
                (np.array([20, 100, 100]), np.array([35, 255, 255])),
            ],
        }

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/oakd/rgb/preview/camera_info',
            self.camera_info_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/oakd/rgb/preview/depth', self.depth_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.nav_complete_sub = self.create_subscription(
            Bool, '/navigation_complete', self.nav_complete_callback, 10)

        self.get_logger().info('Sphere color detector started, waiting for navigation to complete...')

    def nav_complete_callback(self, msg: Bool):
        if msg.data and not self.navigation_complete:
            self.navigation_complete = True
            self.get_logger().info('Navigation complete signal received - now watching for spheres.')

    def camera_info_callback(self, msg: CameraInfo):
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')
            return

        depth = np.array(depth, dtype=np.float32)
        # Some depth encodings are in millimeters (uint16) - normalize to meters.
        if msg.encoding in ('16UC1', 'mono16'):
            depth = depth / 1000.0
        self.latest_depth = depth

    def get_depth_at(self, px, py):
        """Sample depth at a pixel, using a small median-filtered patch
        to avoid noise/holes at a single pixel."""
        if self.latest_depth is None:
            return None
        h, w = self.latest_depth.shape[:2]
        px = int(max(0, min(w - 1, px)))
        py = int(max(0, min(h - 1, py)))
        patch = self.latest_depth[max(0, py - 3):py + 4, max(0, px - 3):px + 4]
        valid = patch[(patch > 0.05) & (patch < 20.0) & np.isfinite(patch)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def image_callback(self, msg: Image):
        if not self.navigation_complete:
            return  # ignore frames until the robot has actually arrived
        if self.reported:
            return
        if self.fx is None:
            return  # wait for camera intrinsics

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        detections = {}  # color_name -> (real_radius_m, pixel_area, depth)

        for color_name, ranges in self.color_ranges.items():
            mask = None
            for lower, upper in ranges:
                m = cv2.inRange(hsv, lower, upper)
                mask = m if mask is None else cv2.bitwise_or(mask, m)

            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area < self.min_area:
                continue

            (px, py), pixel_radius = cv2.minEnclosingCircle(largest)
            depth = self.get_depth_at(px, py)

            if depth is None:
                continue  # can't compute real size without depth here

            # Pinhole camera model: real_radius = pixel_radius * depth / focal_length
            focal = (self.fx + self.fy) / 2.0
            real_radius = pixel_radius * depth / focal

            detections[color_name] = (real_radius, area, depth)

        if len(detections) >= 3:
            largest_color = max(detections, key=lambda c: detections[c][0])
            summary = {c: round(v[0], 4) for c, v in detections.items()}
            self.get_logger().info(f'Estimated real radii (m): {summary}')
            self.get_logger().info(
                f'>>> LARGEST SPHERE COLOR: {largest_color.upper()} <<<')
            self.reported = True
        elif detections:
            summary = {c: round(v[0], 4) for c, v in detections.items()}
            self.get_logger().info(
                f'Partial detection so far: {summary}',
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = SphereColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
