#!/usr/bin/env python3

import rclpy
import threading
import cv2
import numpy as np
import base64
import time
from openai import OpenAI
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data

class ClearDescriptionNode(Node):
    def __init__(self):
        super().__init__('qwen_pic_to_text_node')
        
        # API密钥配置
        self.api_key = self.declare_parameter('api_key', "sk-71dc6fbfcfec42e193ecbb3bb7787313").value
        
        # 状态标志
        self.activated = False
        self.image_processed = False
        self.lock = threading.Lock()
        
        # 初始化API客户端
        self.client = OpenAI(
            api_key=self.api_key,
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
        )
        
        # 为图像和控制信号使用标准传感器QoS配置
        self.sub_qos = qos_profile_sensor_data
        
        # 为结果发布创建专门的QoS配置（兼容可靠性）
        self.pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 结果发布器 - 使用专门配置
        self.result_pub = self.create_publisher(
            String,
            '/pic_info',
            self.pub_qos
        )
        
        # 订阅控制信号 - 使用传感器配置
        self.control_sub = self.create_subscription(
            Int32,
            '/sign4return',
            self.control_handler,
            self.sub_qos
        )
        
        self.get_logger().info("清晰描述节点准备就绪，使用分离QoS配置")

    def control_handler(self, msg):
        """处理控制信号"""
        if msg.data == -1 and not self.activated:
            self.activated = True
            self.get_logger().info("激活指令接收")
            
            # 创建图像订阅器 - 使用传感器配置
            self.image_sub = self.create_subscription(
                CompressedImage,
                '/image', 
                self.process_image,
                self.sub_qos
            )
            self.get_logger().info("已创建图像订阅器")

    def process_image(self, img_msg):
        """处理图像并生成清晰描述"""
        if not self.activated or self.image_processed:
            return
            
        with self.lock:
            if self.image_processed:
                return
                
            self.image_processed = True
            self.get_logger().info("开始处理图像...")
            start_time = time.time()
            
            try:
                # 图像处理
                img_data = np.frombuffer(img_msg.data, np.uint8)
                img_decoded = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
                
                # 检查图像是否有效
                if img_decoded is None or img_decoded.size == 0:
                    raise ValueError("图像解码失败，数据为空")
                
                # 调整图像尺寸以提升API处理性能
                max_dim = 1024
                h, w = img_decoded.shape[:2]
                if max(h, w) > max_dim:
                    scale = max_dim / max(h, w)
                    img_resized = cv2.resize(img_decoded, (int(w * scale), int(h * scale)))
                else:
                    img_resized = img_decoded
                
                # 编码为base64
                _, buf = cv2.imencode('.jpg', img_resized)
                if buf is None:
                    raise ValueError("图像编码失败")
                
                img_base64 = base64.b64encode(buf).decode('utf-8')
                
                # 生成清晰描述
                self.get_logger().info("正在请求Qwen-VL API...")
                description = self.generate_clear_description(img_base64)
                
                # 发布结果
                result = String()
                result.data = description
                self.result_pub.publish(result)
                self.get_logger().info("结果已发布到/pic_info")
                
                # 终端展示
                self.get_logger().info(f"\n图像描述({len(description)}字符):\n{description}")
                
            except Exception as e:
                error_msg = f"处理错误: {str(e)}"
                self.get_logger().error(error_msg)
                result = String()
                result.data = error_msg
                self.result_pub.publish(result)
            finally:
                # 记录处理时间
                elapsed = time.time() - start_time
                self.get_logger().info(f"图像处理完成, 耗时: {elapsed:.2f}秒")
                
                # 2秒后关闭节点
                self.shutdown_timer = self.create_timer(2.0, self.close_node)

    def generate_clear_description(self, img_base64):
        """生成结构清晰的白话文描述"""
        try:
            # 构建符合新API要求的多模态消息
            content_list = [
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_base64}"}},
                {"type": "text", "text": "用一段简明的白话文描述人型立牌主要内容，内容主要与医疗相关，50字以内，要求结构清晰、意义明确"}
            ]
            
            response = self.client.chat.completions.create(
                model="qwen-vl-plus",
                messages=[
                    {
                        "role": "user",
                        "content": content_list
                    }
                ],
                stream=False
            )
            
            # 确保得到有效响应
            if not response.choices:
                return "API未返回有效响应"
            
            if not response.choices[0].message or not response.choices[0].message.content:
                return "API返回的描述内容为空"
                
            raw_desc = response.choices[0].message.content
            return self.refine_description(raw_desc)
            
        except Exception as e:
            # 返回更详细的错误信息用于调试
            error_msg = f"描述生成失败: {str(e)}"
            if hasattr(e, 'response') and hasattr(e.response, 'content'):
                error_msg += f"\nAPI响应内容: {e.response.content}"
            return error_msg
    
    def refine_description(self, description):
        """精炼描述确保简洁明确"""
        # 简单处理：移除多余空格，截断到合理长度
        description = description.strip()
        if len(description) > 200:
            return description[:197] + "..."
        return description
        
    def close_node(self):
        """安全关闭节点"""
        try:
            # 销毁定时器
            if hasattr(self, 'shutdown_timer'):
                self.destroy_timer(self.shutdown_timer)
            
            # 销毁订阅者
            if hasattr(self, 'image_sub'):
                self.destroy_subscription(self.image_sub)
                
            # 关闭节点
            self.destroy_node()
            self.get_logger().info('节点已关闭')
            
        except Exception as e:
            self.get_logger().error(f"关闭节点时出错: {str(e)}")
        finally:
            if rclpy.ok():
                rclpy.shutdown()

def main():
    rclpy.init()
    try:
        node = ClearDescriptionNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"节点执行出错: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 