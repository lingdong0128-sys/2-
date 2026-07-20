#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Int32
from std_msgs.msg import String


class MissionOrchestrator(Node):
    """Mission state machine that coordinates line, QR, nav, and line return."""

    STATE_LINE_FOLLOW = 'LINE_FOLLOW'
    STATE_QR_SCAN = 'QR_SCAN'
    STATE_NAV_RUNNING = 'NAV_RUNNING'
    STATE_LINE_RETURN = 'LINE_RETURN'
    STATE_ERROR = 'ERROR'

    def __init__(self):
        super().__init__('mission_orchestrator')

        self.declare_parameter('qr_activate_sign_value', 0)
        self.declare_parameter('nav_start_sign_value', 5)
        self.declare_parameter('nav_finish_sign_value', 6)
        self.declare_parameter('qr_stable_count', 3)
        self.declare_parameter('qr_cooldown_sec', 3.0)
        self.declare_parameter('scene_timeout_sec', 2.0)
        self.declare_parameter('scene_mismatch_hold_sec', 2.0)

        self.qr_activate_sign_value = int(self.get_parameter('qr_activate_sign_value').value)
        self.nav_start_sign_value = int(self.get_parameter('nav_start_sign_value').value)
        self.nav_finish_sign_value = int(self.get_parameter('nav_finish_sign_value').value)
        self.qr_stable_count = int(self.get_parameter('qr_stable_count').value)
        self.qr_cooldown_sec = float(self.get_parameter('qr_cooldown_sec').value)
        self.scene_timeout_sec = float(self.get_parameter('scene_timeout_sec').value)
        self.scene_mismatch_hold_sec = float(
            self.get_parameter('scene_mismatch_hold_sec').value
        )

        sign4return_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.sign4return_pub = self.create_publisher(Int32, '/sign4return', sign4return_qos)
        self.qr_code_pub = self.create_publisher(String, 'qr_code', 10)
        self.mission_state_pub = self.create_publisher(String, '/mission/state', 10)
        self.recovery_hint_pub = self.create_publisher(String, '/mission/recovery_hint', 10)

        self.create_subscription(String, '/mission/scene_state', self._scene_callback, 10)
        self.create_subscription(Int32, '/sign4return', self._sign4return_callback, sign4return_qos)

        self.current_state = self.STATE_LINE_FOLLOW
        self.last_scene = {}
        self.last_scene_time = 0.0
        self.last_state_change_time = time.monotonic()
        self.last_qr_trigger_time = 0.0
        self.last_qr_text = ''
        self.last_qr_count = 0
        self.last_mismatch_time = 0.0

        self.state_timer = self.create_timer(0.5, self._publish_state)
        self.guard_timer = self.create_timer(0.5, self._guard_state)

        self.get_logger().info('任务编排节点已启动')

    def _scene_callback(self, msg):
        try:
            scene = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('收到不可解析的场景消息', throttle_duration_sec=5.0)
            return

        self.last_scene = scene
        self.last_scene_time = time.monotonic()

        qr_visible = bool(scene.get('qr_visible', False))
        qr_text = str(scene.get('qr_text', '')).strip()
        line_visible = bool(scene.get('line_visible', False))
        yellow_visible = bool(scene.get('yellow_nav_floor_visible', False))

        if self.current_state in (self.STATE_LINE_FOLLOW, self.STATE_QR_SCAN):
            if qr_visible:
                self._set_state(self.STATE_QR_SCAN)
                self._publish_sign4return(self.qr_activate_sign_value)
                self._handle_qr_candidate(qr_text)
            elif line_visible and not yellow_visible:
                self._set_state(self.STATE_LINE_FOLLOW)

        if self.current_state == self.STATE_LINE_RETURN and line_visible and not yellow_visible:
            self._set_state(self.STATE_LINE_FOLLOW)

    def _handle_qr_candidate(self, qr_text):
        if not qr_text:
            return

        if qr_text == self.last_qr_text:
            self.last_qr_count += 1
        else:
            self.last_qr_text = qr_text
            self.last_qr_count = 1

        now = time.monotonic()
        if self.last_qr_count < self.qr_stable_count:
            return
        if (now - self.last_qr_trigger_time) < self.qr_cooldown_sec:
            return

        route_id = qr_text
        if qr_text.isdigit():
            route_id = '1' if (int(qr_text) % 2 == 1) else '2'

        qr_msg = String()
        qr_msg.data = route_id
        self._publish_sign4return(self.nav_start_sign_value)
        self.qr_code_pub.publish(qr_msg)
        self.last_qr_trigger_time = now
        self.get_logger().info(f'二维码 {qr_text} 已稳定识别，切换到导航链，路线={route_id}')
        self._set_state(self.STATE_NAV_RUNNING)

    def _sign4return_callback(self, msg):
        sign_value = int(msg.data)
        if sign_value == self.nav_start_sign_value:
            self._set_state(self.STATE_NAV_RUNNING)
        elif sign_value == self.nav_finish_sign_value:
            self._set_state(self.STATE_LINE_RETURN)

    def _guard_state(self):
        now = time.monotonic()
        if self.last_scene_time <= 0.0:
            return

        if (now - self.last_scene_time) > self.scene_timeout_sec:
            self._publish_recovery_hint('未收到最新相机场景，请将小车放回停车场起点后重试')
            return

        if not self.last_scene:
            return

        scene_label = str(self.last_scene.get('scene_label', 'UNKNOWN'))
        expected = {
            self.STATE_LINE_FOLLOW: {'LINE_FOLLOW', 'QR_SCAN'},
            self.STATE_QR_SCAN: {'QR_SCAN', 'LINE_FOLLOW'},
            self.STATE_NAV_RUNNING: {'NAV_RUNNING', 'UNKNOWN'},
            self.STATE_LINE_RETURN: {'LINE_FOLLOW', 'UNKNOWN'},
        }.get(self.current_state, {'UNKNOWN'})

        if scene_label in expected:
            self.last_mismatch_time = 0.0
            return

        if self.last_mismatch_time == 0.0:
            self.last_mismatch_time = now
            return

        if (now - self.last_mismatch_time) < self.scene_mismatch_hold_sec:
            return

        hint = self._build_recovery_hint()
        self._set_state(self.STATE_ERROR)
        self._publish_recovery_hint(hint)

    def _build_recovery_hint(self):
        if self.current_state == self.STATE_QR_SCAN:
            return '链路异常，请把小车放回扫码点后重试'
        if self.current_state == self.STATE_NAV_RUNNING:
            return '链路异常，请把小车放回最近的航点白纸点后重试'
        if self.current_state == self.STATE_LINE_RETURN:
            return '链路异常，请把小车放回导航出口附近的巡线恢复点后重试'
        return '链路异常，请把小车放回停车场起点后重试'

    def _set_state(self, new_state):
        if new_state == self.current_state:
            return
        self.current_state = new_state
        self.last_state_change_time = time.monotonic()
        self.get_logger().info(f'任务状态切换为 {new_state}')
        self._publish_state()

    def _publish_sign4return(self, value):
        msg = Int32()
        msg.data = int(value)
        self.sign4return_pub.publish(msg)

    def _publish_recovery_hint(self, text):
        msg = String()
        msg.data = text
        self.recovery_hint_pub.publish(msg)
        self.get_logger().warn(text)

    def _publish_state(self):
        payload = {
            'state': self.current_state,
            'scene_age_sec': round(max(time.monotonic() - self.last_scene_time, 0.0), 3)
            if self.last_scene_time > 0.0
            else -1.0,
            'last_scene': self.last_scene,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.mission_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestrator()
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
