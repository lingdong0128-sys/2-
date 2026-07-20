#!/usr/bin/env python3
import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from pyzbar import pyzbar
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class SceneStateGuard(Node):
    """Use the camera image to infer the current mission scene."""

    def __init__(self):
        super().__init__('scene_state_guard')

        self.declare_parameter('image_topic', '/aurora/rgb/image_raw')
        self.declare_parameter('process_interval_sec', 0.20)
        self.declare_parameter('blue_ratio_threshold', 0.010)
        self.declare_parameter('yellow_ratio_threshold', 0.080)
        self.declare_parameter('white_ratio_threshold', 0.030)
        self.declare_parameter('line_roi_start_ratio', 0.55)
        self.declare_parameter('scale_factor', 0.5)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.process_interval_sec = float(self.get_parameter('process_interval_sec').value)
        self.blue_ratio_threshold = float(self.get_parameter('blue_ratio_threshold').value)
        self.yellow_ratio_threshold = float(self.get_parameter('yellow_ratio_threshold').value)
        self.white_ratio_threshold = float(self.get_parameter('white_ratio_threshold').value)
        self.line_roi_start_ratio = float(self.get_parameter('line_roi_start_ratio').value)
        self.scale_factor = float(self.get_parameter('scale_factor').value)

        self.bridge = CvBridge()
        self.last_process_time = 0.0

        self.state_pub = self.create_publisher(String, '/mission/scene_state', 10)
        self.create_subscription(Image, self.image_topic, self._image_callback, 10)

        self.get_logger().info(f'场景守卫已启动，图像输入：{self.image_topic}')

    def _image_callback(self, msg):
        now = time.monotonic()
        if (now - self.last_process_time) < self.process_interval_sec:
            return

        self.last_process_time = now
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().warn(f'图像转换失败：{exc}', throttle_duration_sec=5.0)
            return

        if self.scale_factor < 1.0:
            image = cv2.resize(
                image,
                None,
                fx=self.scale_factor,
                fy=self.scale_factor,
                interpolation=cv2.INTER_AREA,
            )

        result = self._classify_scene(image)
        result['stamp_ns'] = int(self.get_clock().now().nanoseconds)

        state_msg = String()
        state_msg.data = json.dumps(result, ensure_ascii=False)
        self.state_pub.publish(state_msg)

    def _classify_scene(self, image):
        height, width = image.shape[:2]
        roi_start = int(height * self.line_roi_start_ratio)
        bottom_roi = image[roi_start:, :]

        hsv = cv2.cvtColor(bottom_roi, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        blue_mask = cv2.inRange(hsv, np.array([85, 40, 40]), np.array([140, 255, 255]))
        yellow_mask = cv2.inRange(hsv, np.array([15, 50, 60]), np.array([45, 255, 255]))
        white_mask = cv2.inRange(hsv, np.array([0, 0, 180]), np.array([180, 70, 255]))

        total_pixels = max(bottom_roi.shape[0] * bottom_roi.shape[1], 1)
        blue_ratio = float(np.count_nonzero(blue_mask)) / float(total_pixels)
        yellow_ratio = float(np.count_nonzero(yellow_mask)) / float(total_pixels)
        white_ratio = float(np.count_nonzero(white_mask)) / float(total_pixels)

        qr_text = ''
        qr_visible = False
        decoded = pyzbar.decode(gray)
        if decoded:
            qr_visible = True
            try:
                qr_text = decoded[0].data.decode('utf-8')
            except Exception:
                qr_text = ''

        line_visible = blue_ratio >= self.blue_ratio_threshold
        yellow_nav_floor_visible = yellow_ratio >= self.yellow_ratio_threshold
        waypoint_marker_visible = white_ratio >= self.white_ratio_threshold

        if qr_visible:
            scene_label = 'QR_SCAN'
        elif yellow_nav_floor_visible:
            scene_label = 'NAV_RUNNING'
        elif line_visible:
            scene_label = 'LINE_FOLLOW'
        else:
            scene_label = 'UNKNOWN'

        return {
            'scene_label': scene_label,
            'line_visible': line_visible,
            'qr_visible': qr_visible,
            'yellow_nav_floor_visible': yellow_nav_floor_visible,
            'waypoint_marker_visible': waypoint_marker_visible,
            'blue_ratio': round(blue_ratio, 4),
            'yellow_ratio': round(yellow_ratio, 4),
            'white_ratio': round(white_ratio, 4),
            'qr_text': qr_text,
            'image_width': width,
            'image_height': height,
        }


def main(args=None):
    rclpy.init(args=args)
    node = SceneStateGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
