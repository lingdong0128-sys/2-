#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    """把平面偏航角转换成四元数的 z/w 分量。"""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(z_value, w_value):
    """从二维平面用到的 z/w 四元数分量恢复 yaw。"""
    return math.atan2(2.0 * w_value * z_value, 1.0 - 2.0 * z_value * z_value)


def inverse_transform_2d(x_value, y_value, yaw_value):
    """对二维刚体变换求逆。"""
    cos_yaw = math.cos(yaw_value)
    sin_yaw = math.sin(yaw_value)
    inverse_x = -(cos_yaw * x_value + sin_yaw * y_value)
    inverse_y = sin_yaw * x_value - cos_yaw * y_value
    inverse_yaw = -yaw_value
    return inverse_x, inverse_y, inverse_yaw


def compose_transform_2d(first_x, first_y, first_yaw, second_x, second_y, second_yaw):
    """组合两个二维刚体变换。"""
    cos_yaw = math.cos(first_yaw)
    sin_yaw = math.sin(first_yaw)
    composed_x = first_x + cos_yaw * second_x - sin_yaw * second_y
    composed_y = first_y + sin_yaw * second_x + cos_yaw * second_y
    composed_yaw = first_yaw + second_yaw
    return composed_x, composed_y, composed_yaw


class FixedStartManager(Node):
    """统一管理起点归零与二维码定位点矫正的坐标对齐节点。"""

    def __init__(self):
        super().__init__('fixed_start_manager')

        # 这些参数定义了地图系、里程计系以及整场任务起点。
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom_combined')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('perform_initial_reset', False)
        self.declare_parameter('reset_count', 5)
        self.declare_parameter('reset_interval_sec', 0.3)
        self.declare_parameter('publish_frequency', 20.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.start_yaw = float(self.get_parameter('start_yaw').value)
        self.perform_initial_reset = bool(self.get_parameter('perform_initial_reset').value)
        self.reset_count = int(self.get_parameter('reset_count').value)
        self.reset_interval_sec = float(self.get_parameter('reset_interval_sec').value)
        self.publish_frequency = float(self.get_parameter('publish_frequency').value)

        # /set_pose 用来把 EKF 的局部世界归零；/map_alignment/correct_pose 用来做二维码点位矫正。
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/set_pose', 10)
        self.create_subscription(Odometry, f'/{self.odom_frame}', self._odom_callback, 10)
        self.create_subscription(PoseStamped, '/map_alignment/correct_pose', self._correct_pose_callback, 10)

        # 改成动态广播器，后续二维码矫正时可以实时更新 map -> odom_combined。
        self.transform_broadcaster = TransformBroadcaster(self)

        self.latest_odom_x = 0.0
        self.latest_odom_y = 0.0
        self.latest_odom_yaw = 0.0
        self.has_odom = False

        # 当前维护的就是 map -> odom_combined 变换。
        self.map_to_odom_x = self.start_x
        self.map_to_odom_y = self.start_y
        self.map_to_odom_yaw = self.start_yaw

        self.transform_timer = self.create_timer(
            1.0 / max(self.publish_frequency, 1.0),
            self._broadcast_transform,
        )

        self.publish_count = 0
        self.reset_timer = None
        if self.perform_initial_reset:
            self.reset_timer = self.create_timer(self.reset_interval_sec, self._publish_reset_pose)
            self.get_logger().info(
                '已开启整场任务起点归零：'
                f'启动时会把当前位置视为地图 ({self.start_x:.3f}, {self.start_y:.3f}, {self.start_yaw:.3f})'
            )
        else:
            self.get_logger().info('当前仅保持 map->odom 对齐并等待二维码矫正，不执行启动归零')

    def _odom_callback(self, msg):
        """缓存当前 odom_combined -> base_footprint 的局部位姿。"""
        self.latest_odom_x = float(msg.pose.pose.position.x)
        self.latest_odom_y = float(msg.pose.pose.position.y)
        self.latest_odom_yaw = quaternion_to_yaw(
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.has_odom = True

    def _publish_reset_pose(self):
        """整场任务最开始时重复发布零位姿，让 EKF 把当前位置当成本地零点。"""
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.odom_frame
        pose.pose.pose.position.x = 0.0
        pose.pose.pose.position.y = 0.0
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = 0.0
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = 1e-6
        pose.pose.covariance[7] = 1e-6
        pose.pose.covariance[35] = 1e-6

        self.pose_pub.publish(pose)
        self.publish_count += 1
        self.map_to_odom_x = self.start_x
        self.map_to_odom_y = self.start_y
        self.map_to_odom_yaw = self.start_yaw

        self.get_logger().info(
            f'已向 /set_pose 发布第 {self.publish_count}/{self.reset_count} 次起点归零命令'
        )

        if self.publish_count >= self.reset_count and self.reset_timer is not None:
            self.reset_timer.cancel()
            self.get_logger().info('整场任务起点归零完成，后续继续等待二维码定位点矫正')

    def _correct_pose_callback(self, msg):
        """收到二维码定位点后，把当前位置重新对齐到地图中的已知地标。"""
        if not self.has_odom:
            self.get_logger().warn('尚未收到 odom_combined，暂时无法根据二维码位置做地图矫正')
            return

        desired_map_x = float(msg.pose.position.x)
        desired_map_y = float(msg.pose.position.y)
        desired_map_yaw = quaternion_to_yaw(msg.pose.orientation.z, msg.pose.orientation.w)

        inverse_odom_x, inverse_odom_y, inverse_odom_yaw = inverse_transform_2d(
            self.latest_odom_x,
            self.latest_odom_y,
            self.latest_odom_yaw,
        )

        corrected_x, corrected_y, corrected_yaw = compose_transform_2d(
            desired_map_x,
            desired_map_y,
            desired_map_yaw,
            inverse_odom_x,
            inverse_odom_y,
            inverse_odom_yaw,
        )

        self.map_to_odom_x = corrected_x
        self.map_to_odom_y = corrected_y
        self.map_to_odom_yaw = corrected_yaw

        self.get_logger().info(
            '已根据二维码定位点更新 map->odom：'
            f'目标地图位姿=({desired_map_x:.3f}, {desired_map_y:.3f}, {desired_map_yaw:.3f})，'
            f'当前 odom 位姿=({self.latest_odom_x:.3f}, {self.latest_odom_y:.3f}, {self.latest_odom_yaw:.3f})'
        )

    def _broadcast_transform(self):
        """持续广播 map -> odom_combined，让 Nav2 始终工作在地图坐标系下。"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = self.map_to_odom_x
        transform.transform.translation.y = self.map_to_odom_y
        transform.transform.translation.z = 0.0

        quaternion_z, quaternion_w = yaw_to_quaternion(self.map_to_odom_yaw)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = quaternion_z
        transform.transform.rotation.w = quaternion_w

        self.transform_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = FixedStartManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
