#!/usr/bin/env python3
import json
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class TrackMapRecorder(Node):
    """Record odometry during line-follow phases and export path samples."""

    def __init__(self):
        super().__init__('track_map_recorder')

        self.declare_parameter(
            'output_file',
            '/userdata/racing_ws/recordings/recorded_path.yaml',
        )
        self.declare_parameter('min_distance_m', 0.03)
        self.declare_parameter('flush_every_n_points', 20)

        self.output_file = Path(str(self.get_parameter('output_file').value))
        self.min_distance_m = float(self.get_parameter('min_distance_m').value)
        self.flush_every_n_points = int(self.get_parameter('flush_every_n_points').value)

        self.current_state = 'UNKNOWN'
        self.origin_pose = None
        self.last_saved_pose = None
        self.samples = []

        self.create_subscription(String, '/mission/state', self._state_callback, 10)
        self.create_subscription(Odometry, '/odom_combined', self._odom_callback, 10)

        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'轨迹记录器已启动，输出文件：{self.output_file}')

    def _state_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.current_state = str(payload.get('state', 'UNKNOWN'))

    def _odom_callback(self, msg):
        if self.current_state not in ('LINE_FOLLOW', 'LINE_RETURN'):
            return

        x_value = float(msg.pose.pose.position.x)
        y_value = float(msg.pose.pose.position.y)

        if self.origin_pose is None:
            self.origin_pose = (x_value, y_value)

        current_pose = (x_value, y_value)
        if self.last_saved_pose is not None:
            dx_value = current_pose[0] - self.last_saved_pose[0]
            dy_value = current_pose[1] - self.last_saved_pose[1]
            if (dx_value * dx_value + dy_value * dy_value) < (self.min_distance_m ** 2):
                return

        origin_x, origin_y = self.origin_pose
        relative_x = current_pose[0] - origin_x
        relative_y = current_pose[1] - origin_y
        sample = {
            'x': round(relative_x, 4),
            'y': round(relative_y, 4),
            'pixel_x': int(round(relative_x * 100.0)),
            'pixel_y': int(round(relative_y * 100.0)),
        }
        self.samples.append(sample)
        self.last_saved_pose = current_pose

        if len(self.samples) == 1 or (len(self.samples) % self.flush_every_n_points) == 0:
            self._flush_to_disk()

    def _flush_to_disk(self):
        data = {
            'recording': {
                'resolution_m_per_pixel': 0.01,
                'origin_world': {
                    'x': round(self.origin_pose[0], 4) if self.origin_pose else 0.0,
                    'y': round(self.origin_pose[1], 4) if self.origin_pose else 0.0,
                },
                'point_count': len(self.samples),
                'points': self.samples,
            }
        }

        temp_path = self.output_file.with_suffix('.tmp')
        with temp_path.open('w', encoding='utf-8') as handle:
            yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False)
        temp_path.replace(self.output_file)


def main(args=None):
    rclpy.init(args=args)
    node = TrackMapRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node._flush_to_disk()
    finally:
        node._flush_to_disk()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
