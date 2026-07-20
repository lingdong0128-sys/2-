import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ScanMerger(Node):
    def __init__(self) -> None:
        super().__init__('scan_merger')

        self.front_scan_topic = self.declare_parameter('front_scan_topic', '/scan').value
        self.rear_scan_topic = self.declare_parameter('rear_scan_topic', '/scan_rear').value
        self.output_scan_topic = self.declare_parameter('output_scan_topic', '/scan_merged').value
        self.target_frame = self.declare_parameter('target_frame', 'base_footprint').value
        self.source_timeout_sec = float(self.declare_parameter('source_timeout_sec', 0.5).value)
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 12.0).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.front_scan: Optional[LaserScan] = None
        self.rear_scan: Optional[LaserScan] = None
        self.front_received_at = None
        self.rear_received_at = None
        self.last_tf_warning_ns = 0

        self.publisher = self.create_publisher(LaserScan, self.output_scan_topic, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.front_scan_topic, self.on_front_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, self.rear_scan_topic, self.on_rear_scan, qos_profile_sensor_data
        )
        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.publish_merged_scan)

        self.get_logger().info(
            f'Merging {self.front_scan_topic} + {self.rear_scan_topic} into '
            f'{self.output_scan_topic} in frame {self.target_frame}'
        )

    def on_front_scan(self, msg: LaserScan) -> None:
        self.front_scan = msg
        self.front_received_at = self.get_clock().now()

    def on_rear_scan(self, msg: LaserScan) -> None:
        self.rear_scan = msg
        self.rear_received_at = self.get_clock().now()

    def publish_merged_scan(self) -> None:
        active_scans = self.get_active_scans()
        if not active_scans:
            return

        angle_increment = min(
            scan.angle_increment for scan, _received_at in active_scans if scan.angle_increment > 0.0
        )
        angle_min = -math.pi
        angle_max = math.pi
        count = int(round((angle_max - angle_min) / angle_increment)) + 1
        merged_ranges = [math.inf] * count

        range_min = min(scan.range_min for scan, _received_at in active_scans)
        range_max = max(scan.range_max for scan, _received_at in active_scans)
        scan_time = max(scan.scan_time for scan, _received_at in active_scans)
        time_increment = max(scan.time_increment for scan, _received_at in active_scans)
        stamp = self.select_output_stamp(active_scans)

        for scan, _received_at in active_scans:
            transform = self.lookup_scan_transform(scan)
            if transform is None:
                continue
            self.project_scan_into_bins(
                scan=scan,
                transform=transform,
                merged_ranges=merged_ranges,
                angle_min=angle_min,
                angle_increment=angle_increment,
                count=count,
                range_min=range_min,
                range_max=range_max,
            )

        if all(math.isinf(distance) for distance in merged_ranges):
            return

        merged_scan = LaserScan()
        merged_scan.header.frame_id = self.target_frame
        merged_scan.header.stamp = stamp
        merged_scan.angle_min = angle_min
        merged_scan.angle_max = angle_max
        merged_scan.angle_increment = angle_increment
        merged_scan.time_increment = time_increment
        merged_scan.scan_time = scan_time
        merged_scan.range_min = range_min
        merged_scan.range_max = range_max
        merged_scan.ranges = merged_ranges
        merged_scan.intensities = []
        self.publisher.publish(merged_scan)

    def select_output_stamp(self, active_scans: List[Tuple[LaserScan, object]]):
        latest_scan = max(
            active_scans,
            key=lambda item: (item[0].header.stamp.sec, item[0].header.stamp.nanosec),
        )[0]
        return latest_scan.header.stamp

    def get_active_scans(self) -> List[Tuple[LaserScan, object]]:
        now = self.get_clock().now()
        active_scans: List[Tuple[LaserScan, object]] = []

        if self.front_scan is not None and self.front_received_at is not None:
            if (now - self.front_received_at) <= Duration(seconds=self.source_timeout_sec):
                active_scans.append((self.front_scan, self.front_received_at))

        if self.rear_scan is not None and self.rear_received_at is not None:
            if (now - self.rear_received_at) <= Duration(seconds=self.source_timeout_sec):
                active_scans.append((self.rear_scan, self.rear_received_at))

        return active_scans

    def lookup_scan_transform(self, scan: LaserScan):
        if scan.header.frame_id == self.target_frame:
            return None

        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                scan.header.frame_id,
                Time.from_msg(scan.header.stamp),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_tf_warning_ns > 5_000_000_000:
                self.get_logger().warning(
                    f'Cannot transform {scan.header.frame_id} -> {self.target_frame}: {exc}'
                )
                self.last_tf_warning_ns = now_ns
            return None

    def project_scan_into_bins(
        self,
        scan: LaserScan,
        transform,
        merged_ranges: List[float],
        angle_min: float,
        angle_increment: float,
        count: int,
        range_min: float,
        range_max: float,
    ) -> None:
        if transform is None:
            tx = 0.0
            ty = 0.0
            yaw = 0.0
        else:
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            rotation = transform.transform.rotation
            yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            source_angle = scan.angle_min + index * scan.angle_increment
            local_x = distance * math.cos(source_angle)
            local_y = distance * math.sin(source_angle)

            target_x = tx + cos_yaw * local_x - sin_yaw * local_y
            target_y = ty + sin_yaw * local_x + cos_yaw * local_y
            target_distance = math.hypot(target_x, target_y)
            if target_distance < range_min or target_distance > range_max:
                continue

            target_angle = math.atan2(target_y, target_x)
            bin_index = int(round((target_angle - angle_min) / angle_increment))
            if 0 <= bin_index < count and target_distance < merged_ranges[bin_index]:
                merged_ranges[bin_index] = target_distance


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanMerger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
