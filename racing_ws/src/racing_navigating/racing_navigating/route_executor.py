#!/usr/bin/env python3
import math
import threading
import time
from pathlib import Path

import rclpy
import yaml
from ai_msgs.msg import PerceptionTargets
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
try:
    from origincar_msg.msg import Sign
except Exception:
    Sign = None
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Int32
from std_msgs.msg import String


class RouteExecutor(Node):
    """根据二维码结果执行固定路线的任务节点。"""

    MODE_NAVIGATING = 'navigating'
    MODE_LOCAL_AVOIDING = 'local_avoiding'
    MODE_LOCAL_RECOVERING = 'local_recovering'

    def __init__(self):
        super().__init__('route_executor')

        # 这里的参数全部围绕“二维码触发路线”和“复用现有障碍检测结果”展开。
        self.declare_parameter('routes_file', '')
        self.declare_parameter('landmarks_file', '')
        self.declare_parameter('start_route_sign_value', 5)
        self.declare_parameter('finish_route_sign_value', 6)
        self.declare_parameter('obstacle_confidence_threshold', 0.5)
        self.declare_parameter('obstacle_bottom_threshold', 320)
        self.declare_parameter('obstacle_clear_hold_sec', 1.0)
        self.declare_parameter('route_feedback_period_sec', 0.1)
        self.declare_parameter('local_avoidance_enabled', True)
        self.declare_parameter('local_avoid_linear_speed', 0.10)
        self.declare_parameter('local_avoid_angular_ratio', 0.20)
        self.declare_parameter('local_recover_linear_speed', 0.70)
        self.declare_parameter('local_recover_angular_ratio', 0.80)
        self.declare_parameter('local_recover_duration_sec', 0.8)
        self.declare_parameter('local_avoid_timeout_sec', 5.0)
        self.declare_parameter('image_center_x', 320.0)
        self.declare_parameter('local_avoid_min_offset_px', 5.0)

        self.routes_file = self.get_parameter('routes_file').value
        self.landmarks_file = self.get_parameter('landmarks_file').value
        self.start_route_sign_value = int(self.get_parameter('start_route_sign_value').value)
        self.finish_route_sign_value = int(self.get_parameter('finish_route_sign_value').value)
        self.obstacle_confidence_threshold = float(
            self.get_parameter('obstacle_confidence_threshold').value
        )
        self.obstacle_bottom_threshold = int(self.get_parameter('obstacle_bottom_threshold').value)
        self.obstacle_clear_hold_sec = float(self.get_parameter('obstacle_clear_hold_sec').value)
        self.route_feedback_period_sec = float(
            self.get_parameter('route_feedback_period_sec').value
        )
        self.local_avoidance_enabled = bool(
            self.get_parameter('local_avoidance_enabled').value
        )
        self.local_avoid_linear_speed = float(
            self.get_parameter('local_avoid_linear_speed').value
        )
        self.local_avoid_angular_ratio = float(
            self.get_parameter('local_avoid_angular_ratio').value
        )
        self.local_recover_linear_speed = float(
            self.get_parameter('local_recover_linear_speed').value
        )
        self.local_recover_angular_ratio = float(
            self.get_parameter('local_recover_angular_ratio').value
        )
        self.local_recover_duration_sec = float(
            self.get_parameter('local_recover_duration_sec').value
        )
        self.local_avoid_timeout_sec = float(
            self.get_parameter('local_avoid_timeout_sec').value
        )
        self.image_center_x = float(self.get_parameter('image_center_x').value)
        self.local_avoid_min_offset_px = float(
            self.get_parameter('local_avoid_min_offset_px').value
        )

        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.nav2_ready = False
        self.active_goal_handle = None
        self.active_goal_result_future = None

        self.routes = self._load_routes(self.routes_file)
        self.landmarks = self._load_landmarks(self.landmarks_file)

        sign4return_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.sign4return_pub = self.create_publisher(
            Int32, '/sign4return', sign4return_qos
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.correct_pose_pub = self.create_publisher(PoseStamped, '/map_alignment/correct_pose', 10)

        self.create_subscription(String, 'qr_code', self._qr_callback, 10)
        if Sign is not None:
            self.create_subscription(Sign, '/sign_switch', self._sign_switch_callback, 10)
        self.create_subscription(
            Int32, '/sign4return', self._sign4return_callback, sign4return_qos
        )
        self.create_subscription(
            PerceptionTargets, '/racing_obstacle_detection', self._obstacle_callback, 10
        )

        self.state_lock = threading.Lock()
        self.pending_route_id = None
        self.last_qr_code = ''
        self.last_sign_switch = None
        self.last_sign4return = None
        self.route_thread = None
        self.route_running = False
        self.obstacle_blocked = False
        self.last_obstacle_seen_time = 0.0
        self.latest_obstacle = None
        self.current_feedback_waypoint = 0
        self.route_mode = self.MODE_NAVIGATING
        self.last_local_avoidance_direction = 0.0
        self.last_local_avoidance_angular_z = 0.0

        self.get_logger().info(f'路线执行节点已启动，已加载 {len(self.routes)} 条路线')

    def _follow_waypoints_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        if hasattr(feedback, 'current_waypoint'):
            self.current_feedback_waypoint = int(feedback.current_waypoint)

    def _load_routes(self, routes_file):
        """从 YAML 中读取二维码与路线表。"""
        if not routes_file:
            self.get_logger().warn('未提供 routes_file，路线表为空')
            return {}

        routes_path = Path(routes_file)
        if not routes_path.exists():
            self.get_logger().error(f'路线表文件不存在：{routes_file}')
            return {}

        with routes_path.open('r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}

        routes = data.get('routes', {})
        normalized = {}
        for route_id, route_info in routes.items():
            route_key = str(route_id)
            if isinstance(route_info, dict):
                normalized[route_key] = route_info.get('waypoints', []) or []
            else:
                # 为了兼容旧版 routes.yaml，这里仍然允许直接写成点位列表。
                normalized[route_key] = route_info or []
        return normalized

    def _load_landmarks(self, landmarks_file):
        """读取起点和二维码地标配置。"""
        if not landmarks_file:
            self.get_logger().warn('未提供 landmarks_file，二维码矫正功能将保持关闭')
            return {}

        landmarks_path = Path(landmarks_file)
        if not landmarks_path.exists():
            self.get_logger().warn(f'地标配置文件不存在：{landmarks_file}，二维码矫正功能将保持关闭')
            return {}

        with landmarks_path.open('r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        return data

    def _build_pose_message(self, x, y, yaw):
        """把二维位姿转换成 PoseStamped，供 Nav2 和地图矫正共用。"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return pose

    def _resolve_qr_correction_pose(self, qr_code):
        """根据二维码内容查找地图中的已知定位点。"""
        qr_corrections = self.landmarks.get('qr_corrections', {})
        correction = qr_corrections.get(str(qr_code))
        if correction is None:
            correction = qr_corrections.get('default')
        return correction

    def _publish_qr_correction(self, qr_code):
        """在扫码时用已知地标位置对 map->odom_combined 做一次矫正。"""
        correction = self._resolve_qr_correction_pose(qr_code)
        if correction is None:
            self.get_logger().info(f'二维码 {qr_code} 没有配置定位点，本次不执行地图矫正')
            return

        correction_pose = self._build_pose_message(
            correction['x'],
            correction['y'],
            correction.get('yaw', 0.0),
        )
        self.correct_pose_pub.publish(correction_pose)
        self.get_logger().info(
            f'已根据二维码 {qr_code} 发布地图矫正请求：'
            f"({float(correction['x']):.3f}, {float(correction['y']):.3f}, {float(correction.get('yaw', 0.0)):.3f})"
        )

    def _qr_callback(self, msg):
        """收到二维码原始结果后，优先按二维码内容匹配路线。"""
        qr_code = msg.data.strip()
        if not qr_code:
            return

        self.last_qr_code = qr_code
        self._publish_qr_correction(qr_code)
        if qr_code in self.routes:
            self.pending_route_id = qr_code
            self.get_logger().info(f'二维码 {qr_code} 命中路线表，等待启动导航')
        else:
            if Sign is not None:
                self.get_logger().warn(f'二维码 {qr_code} 未直接命中路线表，将尝试等待 /sign_switch 回退映射')
            else:
                self.get_logger().warn(f'二维码 {qr_code} 未直接命中路线表，且 /sign_switch 消息类型不可用，将忽略本次任务')

        self._try_start_route()

    def _sign_switch_callback(self, msg):
        """二维码节点已把奇偶结果映射为 sign_switch，这里做回退路线选择。"""
        self.last_sign_switch = int(msg.sign_data)

        # 当二维码内容未直接命中路线表时，默认用左/右分支映射到预设路线。
        if self.pending_route_id is None:
            if self.last_sign_switch == 3 and '1' in self.routes:
                # 某些现场情况下可能先收到 sign_switch，二维码原文尚未送达。
                # 这时仍按“已经到达二维码点”处理，补发一次默认地标矫正，
                # 避免 Nav2 在未对齐 map 坐标时直接开跑。
                if not self.last_qr_code:
                    self._publish_qr_correction('sign_switch_fallback')
                self.pending_route_id = '1'
                self.get_logger().info('收到 /sign_switch=3，回退选择路线 1')
            elif self.last_sign_switch == 4 and '2' in self.routes:
                if not self.last_qr_code:
                    self._publish_qr_correction('sign_switch_fallback')
                self.pending_route_id = '2'
                self.get_logger().info('收到 /sign_switch=4，回退选择路线 2')

        self._try_start_route()

    def _sign4return_callback(self, msg):
        """仅当原流程已经发出挂起信号后，才允许 Nav2 接管。"""
        self.last_sign4return = int(msg.data)
        if self.last_sign4return == self.start_route_sign_value:
            self._try_start_route()

    def _extract_relevant_obstacle(self, msg):
        """挑选当前最值得关注的锥桶，优先取图像中最靠近车体的目标。"""
        relevant = None
        for target in msg.targets:
            if target.type != 'construction_cone' or not target.rois:
                continue

            roi = target.rois[0]
            if roi.confidence < self.obstacle_confidence_threshold:
                continue

            bottom = int(roi.rect.y_offset + roi.rect.height)
            center_x = float(roi.rect.x_offset + roi.rect.width / 2.0)
            candidate = {
                'bottom': bottom,
                'center_x': center_x,
                'confidence': float(roi.confidence),
            }
            if relevant is None or candidate['bottom'] > relevant['bottom']:
                relevant = candidate
        return relevant

    def _obstacle_callback(self, msg):
        """复用现有障碍检测结果，发现近距离锥桶时暂停导航。"""
        obstacle = self._extract_relevant_obstacle(msg)
        blocked = obstacle is not None and obstacle['bottom'] >= self.obstacle_bottom_threshold

        with self.state_lock:
            self.latest_obstacle = obstacle
            self.obstacle_blocked = blocked
            if blocked:
                self.last_obstacle_seen_time = time.monotonic()

    def _try_start_route(self):
        """满足“已有路线 + 控制已挂起”后，启动独立线程执行任务。"""
        with self.state_lock:
            if self.route_running:
                return
            if self.pending_route_id is None:
                return
            if self.last_sign4return != self.start_route_sign_value:
                return

            route_id = self.pending_route_id
            if route_id not in self.routes:
                self.get_logger().warn(f'路线 {route_id} 不存在，忽略本次任务')
                self.pending_route_id = None
                return

            self.route_running = True
            self.pending_route_id = None
            self.current_feedback_waypoint = 0
            self.route_thread = threading.Thread(
                target=self._run_route, args=(route_id,), daemon=True
            )
            self.route_thread.start()

    def _ensure_nav2_ready(self):
        """等待 map_server 与 bt_navigator 进入可用状态。"""
        if self.nav2_ready:
            return

        self.get_logger().info('等待 Nav2 激活...')
        while rclpy.ok():
            if self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
                break
            self.get_logger().info('等待 /follow_waypoints 动作服务...')
        self.nav2_ready = True
        self.get_logger().info('Nav2 已激活，可以执行路线任务')

    def _send_follow_waypoints_goal(self, poses):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = list(poses)
        send_goal_future = self.follow_waypoints_client.send_goal_async(
            goal_msg,
            feedback_callback=self._follow_waypoints_feedback_cb,
        )

        while rclpy.ok() and not send_goal_future.done():
            time.sleep(self.route_feedback_period_sec)

        if not send_goal_future.done():
            raise RuntimeError('FollowWaypoints goal 未能完成发送')

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('FollowWaypoints goal 被拒绝')

        result_future = goal_handle.get_result_async()
        self.active_goal_handle = goal_handle
        self.active_goal_result_future = result_future
        return goal_handle, result_future

    def _cancel_active_goal(self):
        goal_handle = self.active_goal_handle
        if goal_handle is None:
            return

        cancel_future = goal_handle.cancel_goal_async()
        while rclpy.ok() and not cancel_future.done():
            time.sleep(self.route_feedback_period_sec)

        self.active_goal_handle = None
        self.active_goal_result_future = None

    def _build_pose(self, x, y, yaw):
        """把 YAML 中的二维位姿转换成 PoseStamped。"""
        return self._build_pose_message(x, y, yaw)

    def _build_route_poses(self, route_id):
        """把路线表中的点位转换成 Nav2 可直接执行的航点列表。"""
        route_points = self.routes.get(route_id, [])
        poses = []
        for point in route_points:
            poses.append(self._build_pose(point['x'], point['y'], point.get('yaw', 0.0)))
        return poses

    def _publish_sign4return(self, value):
        """沿用现有总线，不改变其他节点对 /sign4return 的理解。"""
        msg = Int32()
        msg.data = int(value)
        self.sign4return_pub.publish(msg)

    def _publish_stop_cmd(self):
        """导航取消或完成时主动发送零速，避免底盘残留旧速度。"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def _publish_cmd(self, linear_x, angular_z):
        """统一发布速度控制，便于在本地避障阶段复用。"""
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(cmd)

    def _get_latest_obstacle(self):
        """线程安全地读取最近一次障碍物检测结果。"""
        with self.state_lock:
            if self.latest_obstacle is None:
                return None
            return dict(self.latest_obstacle)

    def _is_obstacle_blocking(self):
        """把最近一小段时间内的障碍检测视为“仍然阻塞”。"""
        with self.state_lock:
            if self.obstacle_blocked:
                return True
            if self.last_obstacle_seen_time <= 0.0:
                return False
            return (time.monotonic() - self.last_obstacle_seen_time) < self.obstacle_clear_hold_sec

    def _compute_local_avoidance_cmd(self, obstacle):
        """根据锥桶在图像中的水平偏移量，计算一个远离障碍物的转向速度。"""
        offset = float(obstacle['center_x']) - self.image_center_x
        if -self.local_avoid_min_offset_px < offset < self.local_avoid_min_offset_px:
            if self.last_local_avoidance_direction != 0.0:
                offset = self.local_avoid_min_offset_px * self.last_local_avoidance_direction
            else:
                offset = self.local_avoid_min_offset_px

        normalized_offset = max(-1.0, min(1.0, offset / max(self.image_center_x, 1.0)))
        angular_z = (
            -1.0
            * self.local_avoid_angular_ratio
            * (1.0 - abs(normalized_offset))
            * math.copysign(1.0, normalized_offset)
        )

        current_direction = 1.0 if angular_z > 0.0 else -1.0
        if self.last_local_avoidance_direction == 0.0:
            self.last_local_avoidance_direction = current_direction
        elif self.last_local_avoidance_direction * current_direction < 0.0:
            angular_z = abs(angular_z) * self.last_local_avoidance_direction

        self.last_local_avoidance_angular_z = angular_z
        return self.local_avoid_linear_speed, angular_z

    def _run_local_avoidance(self, route_id, route_offset):
        """Nav2 任务暂停后，临时接管 /cmd_vel 做近距离绕障，再恢复导航。"""
        mode = self.MODE_LOCAL_AVOIDING
        recovery_start_time = None
        start_time = time.monotonic()
        self.last_local_avoidance_direction = 0.0
        self.route_mode = mode

        self.get_logger().warn(
            f'路线 {route_id} 在第 {route_offset} 个航点附近进入本地避障状态机'
        )

        while rclpy.ok():
            now = time.monotonic()
            if (now - start_time) >= self.local_avoid_timeout_sec:
                self.get_logger().error(
                    f'路线 {route_id} 本地避障超时 {self.local_avoid_timeout_sec:.1f}s，保持停车等待人工处理'
                )
                self._publish_stop_cmd()
                self.route_mode = self.MODE_NAVIGATING
                return False

            obstacle_blocking = self._is_obstacle_blocking()
            obstacle = self._get_latest_obstacle()

            if mode == self.MODE_LOCAL_AVOIDING:
                if obstacle_blocking and obstacle is not None:
                    linear_x, angular_z = self._compute_local_avoidance_cmd(obstacle)
                    self._publish_cmd(linear_x, angular_z)
                    time.sleep(self.route_feedback_period_sec)
                    continue

                mode = self.MODE_LOCAL_RECOVERING
                self.route_mode = mode
                recovery_start_time = now
                self.get_logger().info('近距离锥桶已离开主阻塞区，进入本地恢复阶段')
                continue

            if obstacle_blocking and obstacle is not None:
                mode = self.MODE_LOCAL_AVOIDING
                self.route_mode = mode
                self.get_logger().warn('本地恢复过程中再次检测到锥桶，切回本地避障阶段')
                continue

            if recovery_start_time is not None and (
                now - recovery_start_time
            ) >= self.local_recover_duration_sec:
                self._publish_stop_cmd()
                self.route_mode = self.MODE_NAVIGATING
                self.get_logger().info('本地避障完成，准备把剩余航点重新交给 Nav2 规划')
                return True

            angular_z = 0.0
            if self.last_local_avoidance_angular_z != 0.0:
                angular_z = -math.copysign(
                    self.local_recover_angular_ratio,
                    self.last_local_avoidance_angular_z,
                )
            self._publish_cmd(self.local_recover_linear_speed, angular_z)
            time.sleep(self.route_feedback_period_sec)

    def _run_route(self, route_id):
        """后台线程：负责执行、暂停和恢复单条固定路线。"""
        try:
            poses = self._build_route_poses(route_id)
            if not poses:
                self.get_logger().error(f'路线 {route_id} 没有可执行航点')
                return

            self._ensure_nav2_ready()

            # 再次发布挂起信号，确保 racing_control 继续让出 /cmd_vel。
            self._publish_sign4return(self.start_route_sign_value)

            remaining_poses = poses
            route_offset = 0

            while rclpy.ok() and remaining_poses:
                if self._is_obstacle_blocking():
                    if self.local_avoidance_enabled:
                        if not self._run_local_avoidance(route_id, route_offset):
                            return
                    else:
                        self.get_logger().warn('检测到近距离锥桶，当前未启用本地避障，等待障碍清除')
                        while rclpy.ok() and self._is_obstacle_blocking():
                            self._publish_stop_cmd()
                            time.sleep(self.route_feedback_period_sec)

                self.current_feedback_waypoint = 0
                self.route_mode = self.MODE_NAVIGATING
                self.get_logger().info(
                    f'开始执行路线 {route_id}，剩余航点数：{len(remaining_poses)}'
                )
                self._send_follow_waypoints_goal(remaining_poses)

                canceled_for_obstacle = False
                while rclpy.ok():
                    result_future = self.active_goal_result_future
                    if result_future is not None and result_future.done():
                        break
                    if self._is_obstacle_blocking():
                        resume_index = min(
                            max(self.current_feedback_waypoint, 0),
                            len(remaining_poses) - 1,
                        )
                        self.get_logger().warn(
                            f'路线 {route_id} 在第 {route_offset + resume_index} 个航点前被障碍打断，取消当前任务'
                        )
                        self._cancel_active_goal()
                        self._publish_stop_cmd()
                        route_offset += resume_index
                        remaining_poses = remaining_poses[resume_index:]
                        canceled_for_obstacle = True
                        if self.local_avoidance_enabled:
                            if not self._run_local_avoidance(route_id, route_offset):
                                return
                        else:
                            self.get_logger().warn(
                                '当前未启用本地避障，等待障碍离开后继续执行剩余航点'
                            )
                            while rclpy.ok() and self._is_obstacle_blocking():
                                self._publish_stop_cmd()
                                time.sleep(self.route_feedback_period_sec)
                        break

                    time.sleep(self.route_feedback_period_sec)

                if canceled_for_obstacle:
                    continue

                result_future = self.active_goal_result_future
                result = result_future.result() if result_future is not None else None
                status = int(result.status) if result is not None else GoalStatus.STATUS_UNKNOWN
                self.active_goal_handle = None
                self.active_goal_result_future = None

                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info(f'路线 {route_id} 执行成功，恢复原有流程')
                    self._publish_stop_cmd()
                    self._publish_sign4return(self.finish_route_sign_value)
                    return

                if status == GoalStatus.STATUS_CANCELED:
                    self.get_logger().warn(f'路线 {route_id} 被外部取消，保持停止等待人工处理')
                else:
                    self.get_logger().error(f'路线 {route_id} 执行失败，保持停止等待人工处理')

                self._publish_stop_cmd()
                return

        except Exception as exc:
            self.get_logger().error(f'执行路线 {route_id} 时发生异常：{exc}')
            self._publish_stop_cmd()
        finally:
            with self.state_lock:
                self.route_running = False
                self.route_thread = None
                self.current_feedback_waypoint = 0
            self.route_mode = self.MODE_NAVIGATING
            self.last_local_avoidance_direction = 0.0
            self.last_local_avoidance_angular_z = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = RouteExecutor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
