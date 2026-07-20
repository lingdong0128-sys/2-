#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge
import cv2
import time
from pyzbar import pyzbar
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
from rclpy.executors import ExternalShutdownException

class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector_node')
        
        # 创建兼容的QoS配置
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 订阅 /sign4return 话题
        self.sign_sub = self.create_subscription(
            Int32,
            '/sign4return',
            self.sign_callback,
            qos_profile=qos_profile
        )
        
        # 创建发布者 (保持原有话题名称)
        self.publisher_ = self.create_publisher(String, 'qr_code', 10)
        self.pic_info_pub = self.create_publisher(String, 'pic_info', 1)
        self.sign4return_pub = self.create_publisher(Int32, '/sign4return', qos_profile=qos_profile)
        
        # 图像处理相关
        self.bridge = CvBridge()
        self.image_sub = None
        
        # 状态控制
        self.activated = False
        self.last_sign_time = 0
        self.last_process_time = 0

        self.declare_parameter('image_topic', '/aurora/rgb/image_raw')
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.declare_parameter('publish_qr_code', True)
        self.publish_qr_code = self.get_parameter('publish_qr_code').get_parameter_value().bool_value
        
        # 基础参数配置 (保持简单)
        self.process_interval = 0.08  # 处理间隔
        self.scale_factor = 0.8     # 保持部分缩放优化性能

        self.get_logger().info('二维码检测节点已启动 (等待/sign4return激活信号)')

    def sign_callback(self, msg):
        """处理/sign4return信号的回调函数"""
        current_time = time.time()
        self.last_sign_time = current_time
        
        if msg.data == 0 and not self.activated:
            self.get_logger().info('收到激活信号(0)，启动二维码检测')
            self.activated = True
            
            # 仅在需要时创建图像订阅者
            if self.image_sub is None:
                self.image_sub = self.create_subscription(
                    Image,
                    self.image_topic,
                    self.image_callback,
                    10
                )
                self.get_logger().info('已创建图像订阅')

    def image_callback(self, msg):
        """处理图像的回调函数（仅在激活状态下工作）"""
        if not self.activated:
            return
            
        current_time = time.time()
        
        # 按固定频率处理图像
        if current_time - self.last_process_time < self.process_interval:
            return
            
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 可选缩小图像尺寸 (优化性能)
            if self.scale_factor < 1.0:
                h, w = cv_image.shape[:2]
                cv_image = cv2.resize(
                    cv_image, 
                    (int(w * self.scale_factor), int(h * self.scale_factor)),
                    interpolation=cv2.INTER_AREA
                )
            
            # 基础图像预处理 (改进识别率的核心)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            processed = cv2.equalizeHist(gray)  # 直方图均衡化增强对比度
            
            # 简单但有效的二维码检测
            barcodes = pyzbar.decode(processed)
            
            # 处理检测结果
            if barcodes:
                for barcode in barcodes:
                    try:
                        barcodeData = barcode.data.decode("utf-8")
                        
                        # 记录检测到的二维码
                        self.get_logger().info(f'检测到二维码: {barcodeData}')
                        
                        # 发布二维码内容 (保持原有话题)
                        if self.publish_qr_code:
                            qr_msg = String()
                            qr_msg.data = barcodeData
                            self.publisher_.publish(qr_msg)

                        self.send_stop_command()
                        return
                    except Exception as e:
                        self.get_logger().warn(f'处理二维码时出错: {str(e)}')
                        continue
            else:
                # 减少日志输出频率
                if current_time - self.last_sign_time > 5:
                    self.get_logger().info('扫描中...', throttle_duration_sec=2)
                
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}', throttle_duration_sec=5)
        finally:
            self.last_process_time = current_time
    
    def deactivate(self):
        """解除激活状态"""
        if self.image_sub:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            self.get_logger().info('已释放图像订阅')
        
        self.activated = False
        self.get_logger().info('二维码处理完成，等待下次激活')
    
    def send_stop_command(self):
        """发送停止命令到机器人 (保持原有话题)"""
        stop_msg = Int32()
        stop_msg.data = 5
        self.sign4return_pub.publish(stop_msg)
        self.deactivate()  # 完成处理后立即取消激活
        self.get_logger().info('已发送停止命令(5)')

def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    
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
