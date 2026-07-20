#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <origincar_msg/msg/sign.hpp>

using namespace std::chrono_literals;

class QRDetector : public rclcpp::Node {
public:
    QRDetector() : Node("qr_detector_cpp"), activated_(false) {
        // QoS配置 (平衡模式)
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);

        // 订阅激活信号
        sign4return_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/sign4return", qos_profile,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                this->sign_callback(msg);
            });

        // 创建发布者
        sign_switch_pub_ = create_publisher<origincar_msg::msg::Sign>("/sign_switch", 10);
        //qr_code_pub_ = create_publisher<std_msgs::msg::String>("qr_code", 10);
        qr_code_pub_ = create_publisher<std_msgs::msg::String>("qr_code", rclcpp::QoS(10).reliable());
        pic_info_pub_ = create_publisher<std_msgs::msg::String>("pic_info", 1);
        sign4return_pub_ = create_publisher<std_msgs::msg::Int32>("/sign4return", qos_profile);

        // 平衡模式参数
        scale_factor_ = declare_parameter<double>("scale_factor", 0.8);  // 中等缩放
        process_interval_ = declare_parameter<double>("process_interval", 0.08); // 10Hz处理频率
        use_preproc_ = declare_parameter<bool>("use_preproc", true);     // 启用预处理
        qos_depth_ = declare_parameter<int>("qos_depth", 12);             // 中等QoS深度

        // 初始化ZBar (平衡配置)
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_X_DENSITY, 2);  // 中等扫描密度
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_Y_DENSITY, 2);
        
        RCLCPP_INFO(get_logger(), "二维码检测节点已启动 (平衡模式)");
        RCLCPP_INFO(get_logger(), "参数配置: 缩放=%.2f, 间隔=%.3fs (%.1fHz), QoS深度=%d", 
                   scale_factor_, process_interval_, 1/process_interval_, qos_depth_);
    }

private:
    void sign_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data == 0 && !activated_) {
            RCLCPP_INFO(get_logger(), "收到激活信号(0)，启动二维码检测");
            activated_ = true;
            last_process_time_ = now();
            
            // 创建图像订阅（仅在激活状态）
            rclcpp::SensorDataQoS balanced_qos;
            balanced_qos.keep_last(qos_depth_);  // 使用参数配置的QoS深度
            
            image_sub_ = create_subscription<sensor_msgs::msg::Image>(
                "image_raw", 
                balanced_qos,
                [this](const sensor_msgs::msg::Image::SharedPtr img) {
                    this->image_callback(img);
                });
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!activated_) return;
        rclcpp::Time current_time = this->now();
        
        // 限流处理 - 跳过太早到达的帧
        if ((current_time - last_process_time_).seconds() < process_interval_) {
            return;
        }
        last_process_time_ = current_time;

        try {
            // 使用智能指针避免数据拷贝
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            
            // 处理图像
            cv::Mat gray;
            cv::Mat processing_frame;
            
            // 缩放图像 (如果需要)
            if (scale_factor_ < 0.99f) {
                cv::resize(cv_ptr->image, processing_frame, 
                          cv::Size(), scale_factor_, scale_factor_, 
                          cv::INTER_AREA);
            } else {
                processing_frame = cv_ptr->image;
            }
            
            // 转换为灰度图
            cv::cvtColor(processing_frame, gray, cv::COLOR_BGR2GRAY);

            // 可选: 直方图均衡化增强对比度
            if (use_preproc_) {
                cv::equalizeHist(gray, gray);
            }

            // 准备ZBar图像
            zbar::Image zbar_image(
                gray.cols, 
                gray.rows, 
                "Y800", 
                gray.data, 
                gray.cols * gray.rows
            );

            // 执行二维码扫描
            int scan_result = scanner_.scan(zbar_image);
            
            // 处理扫描结果
            if (scan_result > 0) {
                for (auto symbol = zbar_image.symbol_begin(); 
                     symbol != zbar_image.symbol_end(); 
                     ++symbol) {
                    
                    std::string data = symbol->get_data();
                    RCLCPP_INFO(get_logger(), "检测到二维码: %s", data.c_str());
                    
                    // 发布原始数据
                    auto qr_msg = std_msgs::msg::String();
                    qr_msg.data = data;
                    qr_code_pub_->publish(qr_msg);
                    
                    // 处理数字二维码
                    if (!data.empty() && std::all_of(data.begin(), data.end(), ::isdigit)) {
                        int value = std::stoi(data);
                        
                        // 创建转向消息 (奇左偶右)
                        auto sign_msg = origincar_msg::msg::Sign();
                        sign_msg.sign_data = (value % 2 == 1) ? 3 : 4;
                        
                        // 发布图片信息
                        // auto info_msg = std_msgs::msg::String();
                        // info_msg.data = data;
                        // pic_info_pub_->publish(info_msg);


                        
                        // 发布转向信号
                        sign_switch_pub_->publish(sign_msg);
                        RCLCPP_INFO(get_logger(), "已发布转向信号: %d", sign_msg.sign_data);
                        
                        // 发送停止命令
                        send_stop_command();
                        return;  // 发现二维码后立即退出处理循环
                    }
                }
            } else {
                // 限流日志 (仅debug级别)
                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "扫描中...");
            }
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "CV桥异常: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "处理异常: %s", e.what());
        }
    }

    void send_stop_command() {
        auto stop_msg = std_msgs::msg::Int32();
        stop_msg.data = 5;
        sign4return_pub_->publish(stop_msg);
        deactivate();
        RCLCPP_INFO(get_logger(), "已发送停止命令(5)");
    }

    void deactivate() {
        activated_ = false;
        image_sub_.reset();  // 销毁图像订阅节省资源
        RCLCPP_INFO(get_logger(), "二维码处理完成，等待下次激活");
    }

    // 成员变量
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sign4return_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<origincar_msg::msg::Sign>::SharedPtr sign_switch_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_code_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pic_info_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr sign4return_pub_;

    zbar::ImageScanner scanner_;
    bool activated_;
    rclcpp::Time last_process_time_;
    
    // 可配置参数
    double scale_factor_;        // 图像缩放因子
    double process_interval_;   // 处理间隔 (秒)
    bool use_preproc_;           // 是否使用预处理
    int qos_depth_;              // QoS订阅深度
};  // 这里添加分号

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QRDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}