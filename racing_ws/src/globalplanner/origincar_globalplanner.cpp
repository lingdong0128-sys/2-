#include "origincar_globalplanner.h"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <algorithm>
#include <angles/angles.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <chrono>  // 必须包含的时间库头文件
#include <thread> // 线程操作头文件
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/memory_strategy.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>

SyncPlannerNode::SyncPlannerNode() 
    : Node("syncplanner"),
      occupancy_grid_(500, std::vector<float>(200, 0.0f)),
      pose_initialized_(false) {
    
    // 创建消息过滤器订阅
    /*odom_sub_.subscribe(this, "/odom_combined");
    detection_sub_.subscribe(this, "/racing_obstacle_detection");

    // 设置同步策略 
    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
        ApproxSyncPolicy(30),
        odom_sub_, 
        detection_sub_);
    
    sync_->setInterMessageLowerBound(0, rclcpp::Duration(1, 0));  // 第一个消息类型
    sync_->setInterMessageLowerBound(1, rclcpp::Duration(1, 0));  // 第二个消息类型
    sync_->registerCallback(&SyncPlannerNode::sync_callback, this);*/

    /*depth_sub_.subscribe(this,"/aurora/depth/image_raw");
    detection_sub__.subscribe(this, "/racing_obstacle_detection");
    sync__ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy_>>(
        ApproxSyncPolicy_(30), 
        detection_sub__,
        depth_sub_);
    sync__->setInterMessageLowerBound(0, rclcpp::Duration(1, 0));  // 第一个消息类型
    sync__->setInterMessageLowerBound(1, rclcpp::Duration(1, 0));  // 第二个消息类型
    sync__->registerCallback(&SyncPlannerNode::sync_callback_, this);*/
    
    // QoS配置
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // 1. 创建高优先级回调组
    auto high_priority_group = this->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive,
          true
    );
    
    // 创建订阅选项对象
    rclcpp::SubscriptionOptions options;
    // 将回调组分配给订阅选项
    options.callback_group = high_priority_group;

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom_combined", 10,
            std::bind(&SyncPlannerNode::odom_callback, this, std::placeholders::_1));

    hat_subscriber_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        "racing_obstacle_detection", qos_profile,
        std::bind(&SyncPlannerNode::subscription_callback_target, this, std::placeholders::_1),options); 
    
    depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
           "/aurora/depth/image_raw", 
            qos_profile, 
            std::bind(&SyncPlannerNode::depth_image_callback_, this, std::placeholders::_1),options);
    
    /*hat_subscriber_second = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
            "racing_obstacle_detection_second", 10,
            std::bind(&SyncPlannerNode::subscription_callback_target_second, this, std::placeholders::_1));*/
    
    qr_subscriber_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
            "racing_obstacle_detection_qr", qos_profile,
            std::bind(&SyncPlannerNode::subscription_callback_target_qr, this, std::placeholders::_1)); 
    
    service_ = create_service<origincar_msg::srv::UpdatePlanner>(
            "update_planner",
            std::bind(&SyncPlannerNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));
    
    client = this->create_client<origincar_msg::srv::UpdatePlanner>("update_planner");
    
    // 发布控制命令
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    subscriber_return = this->create_subscription<std_msgs::msg::Int32>(
        "/sign4return", 10,
        std::bind(&SyncPlannerNode::subscription_callback_return, this, std::placeholders::_1)); 

    subscriber_switch = this->create_subscription<origincar_msg::msg::Sign>(
            "/sign_switch", qos_profile,
            std::bind(&SyncPlannerNode::subscription_callback_switch, this, std::placeholders::_1), 
            options);
    
    // 定时规划器 
    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),  // 100毫秒 = 0.1秒
        [this](){
                this->plan_path();
        }
    );

    RCLCPP_INFO(get_logger(), "Sync Planner Node initialized");
}

/*void SyncPlannerNode::sync_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr& detections) {

     // 1. 检查消息有效性
     if (!odom || !detections) {
        RCLCPP_ERROR(get_logger(), "Invalid message received (odom: %d, detections: %d)", 
                    (odom != nullptr), (detections != nullptr));
        return;
    }
    
    // 更新当前位置
    current_pose_.header = odom->header;
    current_pose_.pose = odom->pose.pose;
    
    // 更新障碍物地图
    std::lock_guard<std::mutex> lock(grid_mutex_);

    for (const auto& detection : detections->targets) {
        if (detection.type == "hat") {

        for (const auto& roi : detection.rois){
         // 检查置信度
         if (roi.confidence < confidence_threshold_) {
            RCLCPP_WARN(get_logger(), "Low confidence(%.2f < %.2f)", 
                        roi.confidence, confidence_threshold_);
            continue;
          }
        
        // 添加严格验证
        if (roi.rect.x_offset > 10000 || roi.rect.y_offset > 10000 || 
            roi.rect.width > 10000 || roi.rect.height > 10000) {
                RCLCPP_ERROR(get_logger(), "Abnormal ROI values: x=%u y=%u w=%u h=%u",
                roi.rect.x_offset, roi.rect.y_offset,
                roi.rect.width, roi.rect.height);
                continue;
            }

        //坐标转换
        double theta = tf2::getYaw(odom->pose.pose.orientation);
        int img_x = static_cast<int>(roi.rect.x_offset + roi.rect.width / 2.0);
        int img_y = static_cast<int>(roi.rect.y_offset + roi.rect.height);

        //RCLCPP_WARN(get_logger(), "Raw image coords: x=%d, y=%d", img_x, img_y);
        auto local = inverse_perspective_point(img_x, img_y);

        //RCLCPP_WARN(get_logger(), "current_x=%.f, current_y=%.f", current_pose_.pose.position.y + x_dff, 
        //current_pose_.pose.position.x + y_dff);
        //RCLCPP_WARN(get_logger(), "Local coords: x=%d, y=%d", local.first, local.second);
        std::pair<double,double> goal_d;
        if(!is_second.load()){
        goal_d = local_to_global((double)(current_pose_.pose.position.y+x_dff) ,
        (double)(current_pose_.pose.position.x+y_dff),
            theta,(double)local.first,
            (double)local.second,(!is_second.load()?0:1));
        }else{
            goal_d = local_to_global((double)(current_pose_.pose.position.x+y_dff - x_),
            (double)(current_pose_.pose.position.y+ x_dff -y_),
            theta,(double)local.first,
            (double)local.second,(!is_second.load()?0:1));
        }

        // 转换为全局坐标 (厘米)
        int gx = static_cast<int>(goal_d.first);
        int gy = static_cast<int>(goal_d.second);

        int cost_x = 30; 
        int cost_y = 30;

        const int start =std::max(0, gx - cost_x);
        const int end = std::min(199, gx + cost_x);

        // 更新占用网格 (障碍物
            for (int i = std::max(0, gy-cost_y); i <= std::min(499, gy+cost_y); ++i) {
                for (int j = start; j <= end ; ++j) {
                    occupancy_grid_[i][j] = 1.0f;
                }
            }
    }
  }
}
pose_initialized_.store(true);
}*/

void SyncPlannerNode::plan_path() {
    std::lock_guard<std::mutex> lock(planning_mutex_);
    /*if (!pose_initialized_.load()) {
        RCLCPP_WARN(get_logger(), "Current pose not initialized, cannot plan");
        return;
    }*/

    if (stop_planning_.load()){
        //std::lock_guard<std::mutex> lock(grid_mutex_);
        //occupancy_grid_ = std::vector<std::vector<float>>(500, std::vector<float>(200, 0.0f));
        return;
    }

    if(is_plan.load()){
    // 转换为网格坐标 (厘米)
    if(!is_second.load()){
       start_= {20,80};
        goal_ = {175,480}; //175 485
    }else{
        start_= {190,250};
        //goal_ = {20,35};
        //start_= {(int)((last_odom_.pose.pose.position.y + x_dff)*100) , (int)((last_odom_.pose.pose.position.x + y_dff)*100)};
        goal_ = {30,0};//(15,30)
    }

    if((is_local_.load() || is_local_first.load() ) && !is_second.load()){
        start_= {(int)((last_odom_.pose.pose.position.y + x_dff)*100) , (int)((last_odom_.pose.pose.position.x + y_dff)*100)};
        RCLCPP_WARN(get_logger(), "current_local_x=%d, current_local_y=%d",
        (int)(last_odom_.pose.pose.position.y + x_dff), 
        (int)(last_odom_.pose.pose.position.x + y_dff));
        //goal_ = {165,480};
        goal_ = std::make_pair(180, 475);
    }

    if(is_local_second.load() && is_second.load()){
        start_= {(int)((last_odom_.pose.pose.position.x + y_dff -x_)*100) , 
            (int)((last_odom_.pose.pose.position.y + x_dff - y_)*100)};
        RCLCPP_WARN(get_logger(), "current_local_x=%d, current_local_y=%d, second!!!!!!!!!!!",
        (int)(last_odom_.pose.pose.position.x + y_dff -x_), 
        (int)(last_odom_.pose.pose.position.y + x_dff - y_));
        goal_ = {15,30};
    }

    // 执行A*算法
    // 加锁读取occupancy_grid_
    if(!pose_initialized_.load()){
    
        // 等待服务可用
        if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "Service not available");
        return;
        }

        // 构造请求
        auto request = std::make_shared<origincar_msg::srv::UpdatePlanner::Request>();
        if(!is_second.load()){
        {
        std::lock_guard<std::mutex> lock(point_target_mutex_);
       
        // 2. 寻找目标数量最多的消息
        auto max_targets_it = targets_queue_.top(); // 初始化默认值
        size_t max_targets = max_targets_it->targets.size();

        // 遍历队列所有消息（需根据队列类型调整遍历方式）
        std::vector<ai_msgs::msg::PerceptionTargets::ConstSharedPtr> temp_queue;
        while (!targets_queue_.empty()) {
            auto msg = targets_queue_.top();
            temp_queue.push_back(msg);
            if (msg->targets.size() > max_targets) {
                max_targets = msg->targets.size();
                max_targets_it = msg;
            }
            targets_queue_.pop();
        }

        // 3. 恢复队列（如果需要保留历史数据）
        for (auto& msg : temp_queue) {
            targets_queue_.push(msg);
        }

        //request->detections = *targets_queue_.top();
        request->detections = ai_msgs::msg::PerceptionTargets(*max_targets_it);
        }
    }else{
        std::lock_guard<std::mutex> lock(point_target_mutex_second);
       
        // 2. 寻找目标数量最多的消息
        auto max_targets_it = targets_queue_.top(); // 初始化默认值
        size_t max_targets = max_targets_it->targets.size();

        // 遍历队列所有消息（需根据队列类型调整遍历方式）
        std::vector<ai_msgs::msg::PerceptionTargets::ConstSharedPtr> temp_queue;
        while (!targets_queue_.empty()) {
            auto msg = targets_queue_.top();
            temp_queue.push_back(msg);
            if (msg->targets.size() > max_targets) {
                max_targets = msg->targets.size();
                max_targets_it = msg;
            }
            targets_queue_.pop();
        }

        // 3. 恢复队列（如果需要保留历史数据）
        for (auto& msg : temp_queue) {
            targets_queue_.push(msg);
        }

        //request->detections = *targets_queue_.top();
        request->detections = ai_msgs::msg::PerceptionTargets(*max_targets_it);
    }
    
        auto future = client->async_send_request(request,
            [this](rclcpp::Client<origincar_msg::srv::UpdatePlanner>::SharedFuture future) {
              auto response = future.get();
              RCLCPP_WARN(get_logger(), "Service call: %s", response->message.c_str());
            });
    }

    {
    std::lock_guard<std::mutex> lock(astar_mutex_);
    if(pose_initialized_.load()){
        {
            std::lock_guard<std::mutex> lock(grid_mutex_);
            grid_copy = std::vector<std::vector<float>>(500, std::vector<float>(200));
            for (size_t i = 0; i < 500; ++i) {
                std::copy(occupancy_grid_[i].begin(), occupancy_grid_[i].end(), grid_copy[i].begin());
            }
        }
        geometry_msgs::msg::PoseStamped current_pose_1;
    if (!get_current_pose(current_pose_1)) {
        return;
    }
    /*path = is_second.load() ? straight_line_with_obstacle_avoidance(grid_copy, start_, goal_): 
    a_star(grid_copy,start_,goal_,tf2::getYaw(current_pose_.pose.orientation),current_pose_1);*/
    path = a_star(grid_copy,start_,goal_,tf2::getYaw(current_pose_.pose.orientation),current_pose_1);
    }else {
        RCLCPP_WARN(get_logger(),"pose_initialized_ not ok");
        return;
    }
    }
    
    // 转换为ROS Path消息
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = "map";
    
    for (const auto& point : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = point.first / 100.0;  // 厘米转米
        pose.pose.position.y = point.second / 100.0;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    std::lock_guard<std::mutex> path_lock(path_mutex_);
    {
        current_path_ = path_msg;
    }

    std::lock_guard<std::mutex> paln_lock(plan_mutex_);
    {
        is_plan.store(false);
    }

    is_start.store(true);
    RCLCPP_WARN(get_logger(), "plan path");
}

//开始避障
if (!is_start.load()) {
    RCLCPP_WARN(get_logger(), "Current start not ok, cannot plan");
    return;
}

geometry_msgs::msg::Twist cmd;
// 3. 获取当前路径
nav_msgs::msg::Path current_path;
{
    std::lock_guard<std::mutex> lock(path_mutex_);
    {
        //RCLCPP_WARN(get_logger(), "is planning");
        current_path = current_path_;
    }
}

if (current_path.poses.empty()) {
    return;
}

//  获取当前位姿（假设通过订阅odom或tf获取）
geometry_msgs::msg::PoseStamped current_pose;
if (!get_current_pose(current_pose)) {
    return;
}

double current_yaw = tf2::getYaw(current_pose.pose.orientation);

// 4. 寻找路径上最近的前方目标点
auto target_pose = find_target_point(current_pose, current_path);
// 5. 计算控制误差
double global_dx;
double global_dy;
if(!is_second.load()){
     global_dx = target_pose.pose.position.x - (current_pose.pose.position.y + x_dff);
     global_dy = target_pose.pose.position.y - (current_pose.pose.position.x + y_dff);
}else{
     global_dx = target_pose.pose.position.x - (current_pose.pose.position.y + x_dff_ - y_);
     global_dy = target_pose.pose.position.y - (current_pose.pose.position.x + y_dff_ - x_);
}

//计算dt
double dt =0.05;

// 角度误差（目标方向与当前朝向的差）
double robot_dx = is_second.load() ? global_dy : global_dy;
double robot_dy = is_second.load() ? global_dx : global_dx;
    
double target_yaw = is_second.load() ? std::atan2(robot_dy, robot_dx) : std::atan2(robot_dy, robot_dx);

/*if(is_second_obs.load()){
    auto now = std::chrono::steady_clock::now();
    double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(now - slow_stop_start_time_).count();
    
    // 如果停止时间不足，保持停止状态
    if (elapsed_seconds < SLOW_STOP_DURATION_second) {
        //linear_vel = 0.0;
        //angular_vel = 0.0;
        target_yaw = -1.2;
        RCLCPP_WARN(get_logger(), "In slow stop period (%.1f/%.1f seconds)", 
                   elapsed_seconds, SLOW_STOP_DURATION_second);
    }
    is_second_obs.store(false);
}*/

//if(is_findt_ok.load()) target_yaw = is_findt_ok_.load() ? -3.1 :-2.85; //0.5 -2.9
/*if(is_findt_ok.load()){
    target_yaw  = -2.85;
    if(is_findt_ok_.load()){
        target_yaw = is_findt_ok_left.load() ? -2.95 : -1.5;
    }
}*/

//target_yaw -= 0.3*(3.2-fabs(target_yaw));

if (is_findt_ok.load()) {
    if(!is_find_ok_yaw.load()){
    error_x = 0.2 - current_pose.pose.position.y + x_dff_ - y_;
    error_y = 0.5 - current_pose.pose.position.x + y_dff_ - x_;
    is_find_ok_yaw.store(true);
    //is_findt_ok_.store(false);
    }
    //double distance_to_goal = std::hypot(error_x, error_y);

    // 降速
    //double linear_vel = Kp_linear * distance_to_goal;
    //linear_vel = std::min(linear_vel, 0.4);

    // 强制直线对准
    //double target_yaw = std::atan2(error_y, error_x) - 0.2*(fabs(std::atan2(error_y, error_x)) - 3.2);
    //if(!is_find_ok_yaw.load()){
        //if(!is_findt_ok_.load()){
        if(!is_findt_ok_.load()){
        target_yaw =  std::atan2(error_x, error_y) - M_PI*1.5;
        target_yaw = adjustYawSmoothly(target_yaw);
        }
      //  is_find_ok_yaw.store(true);
    //}

        //if(target_yaw_count > 5){
        //is_find_ok_yaw.store(false);
    

    if(is_findt_ok_.load()){
        target_yaw = is_findt_ok_left.load() ? -3.1 : -1.5;
        if(is_findt_ok_left.load()) linear_vel_yaw.store(true);
        RCLCPP_WARN(get_logger(), "target_yaw:::::%.2f",target_yaw);
    }
    RCLCPP_WARN(get_logger(), "target_yaw:%.2f",target_yaw);
    //angle_error_ = angles::shortest_angular_distance(current_yaw, target_yaw);
    //is_findt_ok_.store(true);
    //is_findt_ok.store(false);
    //double angular_vel = Kp_angular * angle_error;

    //publish_cmd_vel(linear_vel, angular_vel);
}

double angle_error =  angles::shortest_angular_distance(current_yaw, target_yaw);

// 6. PID控制
static double prev_angle_error = 0;

// PID参数（需要调试）
double Kp_angular = fabs(angle_error) > M_PI/4 ? 30: 20;//fabs(angle_error) > M_PI/4 ? 30.0: 20.0;
//if(is_local_.load()) Kp_angular *= 1.5;
double Kd_angular = 5.0;//5.0
//if(is_second.load()) Kp_angular *= 1.5;

static SpeedController speed_controller(get_logger());
double linear_vel = speed_controller.calculateSpeed(
    current_pose,
    current_path,
    grid_copy,
    is_second.load(),
    current_pose.pose.position.x + y_dff,
    angle_error,
    is_local_.load(),
    is_qr_ok.load(),
    obs_count,
    is_first_slow_.load(),
    is_depth_second.load()
);

//linear_vel = is_local_.load() ? (linear_vel/3)*2 : linear_vel;
double k_base = 2.5;//2.5
double Kff_angular = speed_controller.configureBaseKff(current_pose, 
    grid_copy ,k_base , 1.5, 8.0, is_second.load(),is_local.load());

//local_second
/*if(is_second.load() && !is_local_second.load() && (current_pose.pose.position.x + y_dff_ - x_ < 1.0)){
    RCLCPP_WARN(get_logger(), "is local planning_second! ! !,obs_count: %d", obs_count);
    is_start.store(false);
    pose_initialized_.store(false);
    is_local_second.store(true);
    is_plan.store(true);
    is_slow_first.store(false);

    if (!is_slow_stopping_second.load()) {
        slow_stop_start_time_ = std::chrono::steady_clock::now();
        is_slow_stopping_second.store(true);
        RCLCPP_WARN(get_logger(), "Starting slow stop timer");
    }
    
    // 计算已经停止的时间
    auto now = std::chrono::steady_clock::now();
    double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(now - slow_stop_start_time_).count();
    
    // 如果停止时间不足，保持停止状态
    if (elapsed_seconds < SLOW_STOP_DURATION) {
        linear_vel = 0.0;
        //angular_vel = 0.0;
        RCLCPP_WARN(get_logger(), "In slow stop period (%.1f/%.1f seconds)", 
                   elapsed_seconds, SLOW_STOP_DURATION);
    }


    // 4. 渐进式减速（更安全的停止逻辑）
    /*static double last_linear_vel = linear_vel;  // 保存当前速度
    const double deceleration_rate = 0.8;        // 减速度 (m/s²)
    const double dt = 0.4;                       // 控制周期 (100ms)
    
    // 计算新速度（确保不小于0）
    linear_vel = std::max(0.0, last_linear_vel - deceleration_rate * dt);
    last_linear_vel = linear_vel;

    // 5. 如果速度已归零，发送一次停止命令
    if (linear_vel <= 0.1) {
        linear_vel = 0.0;
    }

}*/

//local_first
/*is_slow_first.store(isPastLastCone_first(current_pose.pose,grid_copy));
if(is_slow_first.load() && !is_local_first.load() && !is_second.load()){
    /*{
        std::lock_guard<std::mutex> lock(grid_mutex_);
        occupancy_grid_ = std::vector<std::vector<float>>(500, std::vector<float>(200, 0.0f));
    }
    RCLCPP_WARN(get_logger(), "is local planning_first ! ! !,obs_count: %d", obs_count);
    is_start.store(false);
    pose_initialized_.store(false);
    is_local_first.store(true);
    is_plan.store(true);
    is_slow_first.store(false);

    if (!is_slow_stopping_first.load()) {
        slow_stop_start_time_ = std::chrono::steady_clock::now();
        is_slow_stopping_first.store(true);
        RCLCPP_WARN(get_logger(), "Starting slow stop timer");
    }
    
    // 计算已经停止的时间
    auto now = std::chrono::steady_clock::now();
    double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(now - slow_stop_start_time_).count();
    
    // 如果停止时间不足，保持停止状态
    if (elapsed_seconds < SLOW_STOP_DURATION) {
        linear_vel = 0.0;
        //angular_vel = 0.0;
        RCLCPP_WARN(get_logger(), "In slow stop period (%.1f/%.1f seconds)", 
                   elapsed_seconds, SLOW_STOP_DURATION);
    }
    //linear_vel = 0.0;

    // 4. 渐进式减速（更安全的停止逻辑）
    /*static double last_linear_vel = linear_vel;  // 保存当前速度
    const double deceleration_rate = 0.6;        // 减速度 (m/s²)
    const double dt = 0.4;                       // 控制周期 (100ms)
    
    // 计算新速度（确保不小于0）
    linear_vel = std::max(0.0, last_linear_vel - deceleration_rate * dt);
    last_linear_vel = linear_vel;

    // 5. 如果速度已归零，发送一次停止命令
    if (linear_vel <= 0.1) {
        linear_vel = 0.0;
    }
}*/

//depth + yolo local
//const double slow_y = 3.0;
//double distance_threshold_ = slow_y - (current_pose.pose.position.x + y_dff);
/*is_slow.store(isPastLastCone_first(current_pose.pose,grid_copy));
if(is_slow.load() && !is_local_.load() && !is_second.load()){
    /*{
        std::lock_guard<std::mutex> lock(grid_mutex_);
        occupancy_grid_ = std::vector<std::vector<float>>(500, std::vector<float>(200, 0.0f));
    }
    RCLCPP_WARN(get_logger(), "is local planning ! ! !,obs_count: %d", obs_count);
    is_start.store(false);
    pose_initialized_.store(false);
    is_local_.store(true);
    is_plan.store(true);
    is_slow.store(false);
    //linear_vel = 0.0;

    if (!is_slow_stopping_.load()) {
        slow_stop_start_time_ = std::chrono::steady_clock::now();
        is_slow_stopping_.store(true);
        RCLCPP_WARN(get_logger(), "Starting slow stop timer");
    }
    
    // 计算已经停止的时间
    auto now = std::chrono::steady_clock::now();
    double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(now - slow_stop_start_time_).count();
    
    // 如果停止时间不足，保持停止状态
    if (elapsed_seconds < SLOW_STOP_DURATION) {
        linear_vel = 0.0;
        //angular_vel = 0.0;
        RCLCPP_WARN(get_logger(), "In slow stop period (%.1f/%.1f seconds)", 
                   elapsed_seconds, SLOW_STOP_DURATION);
    }

    // 4. 渐进式减速（更安全的停止逻辑）
    /*static double last_linear_vel = linear_vel;  // 保存当前速度
    const double deceleration_rate = 0.8;        // 减速度 (m/s²)
    const double dt = 0.5;                       // 控制周期 (100ms)
    
    // 计算新速度（确保不小于0）
    linear_vel = std::max(0.0, last_linear_vel - deceleration_rate * dt);
    last_linear_vel = linear_vel;

    // 5. 如果速度已归零，发送一次停止命令
    if (linear_vel <= 0.1) {
        linear_vel = 0.0;
    }
}*/

double qr_y;
const float LINE_REAL_MIDX = 340;
//double distance_threshold = qr_y - (current_pose.pose.position.x + y_dff);
/*if(!is_second.load()){
    if(isPastLastCone(current_pose.pose,grid_copy)) is_qr.store(true);
}*/

if(!qr_queue_.empty() && !is_second.load()){
    float current_x = target_x;
    int Target_value = LINE_REAL_MIDX;
    double angle_error_qr = (Target_value - current_x)/110;
    if(angle_error_qr < 1.2 && angle_error_qr > -1.2 && qr_size > 60000) is_qr_ok.store(true); //1.2 6
    //Kp_angular *= 0.8;
    //Kd_angular = 5.0;

    qr_y = is_qr_y.load() ? qr_y : current_pose.pose.position.x + y_dff;
    is_qr_y.store(true);

    //double error_factor = 1.0;
    const double slow_down_range = 4.6 - qr_y;//4.6
    double distance_to_threshold = current_pose.pose.position.x + y_dff - qr_y;
    if ( distance_to_threshold > 0) {
        // 当接近阈值时，线性减小ros2 launch rosbridge_server rosbridge_websocket_launch.xml

        double normalized_distance_factor = distance_to_threshold / slow_down_range;
        //error_factor = 0.1 + 0.9 * normalized_distance * normalized_distance;
        double normalized_distance_factor_ = 0.3; //0.4 0.5

        angle_error =  angle_error_qr;

        //angle_error = angle_error;
    }

}

// 1. 计算目标角速度变化率（前馈部分）
static double last_target_yaw = target_yaw;
double target_yaw_rate = (target_yaw - last_target_yaw)/dt; // dt为循环周期时间
last_target_yaw = target_yaw;

double effective_ff =  Kff_angular;
//double effective_ff = 0.03;
//effective_ff = is_second.load() ? effective_ff*2.0 : effective_ff;
if(is_qr.load()) effective_ff = 0;
if(fabs(angle_error) < M_PI/12) effective_ff *= 0.7;
if(fabs(angle_error) > M_PI/6) effective_ff *= 1.3;

if(is_second.load()){
    //Kp_angular *= 2.0;
    const double slow_down_range = 1.0;//4.6
    double distance_to_threshold = 2.5 - (current_pose.pose.position.y + x_dff_ - y_);
    if ( distance_to_threshold > 0) {
        // 当接近阈值时，线性减小
        double normalized_distance_factor = distance_to_threshold / slow_down_range;
        Kp_angular = Kp_angular*1.5 + normalized_distance_factor*Kp_angular;
        Kd_angular = 0.0;
    }
}

if(is_second.load()) effective_ff  = 0.0;

// 角速度控制
double angular_vel = Kp_angular * angle_error +
                    Kd_angular * (angle_error - prev_angle_error) + effective_ff * target_yaw_rate; // 前馈项
prev_angle_error = angle_error;

//if(is_qr.load()) angular_vel = angular_vel/114;
double angular_z = is_second.load() ? 50.0 : 10.0;
angular_vel = std::clamp(angular_vel, -angular_z, angular_z); // 限制角速度范围

if(is_findt_ok.load() ) linear_vel *=0.7;
if(linear_vel_yaw.load()) linear_vel*= 2.0;
//if(is_findt_ok.load()) angular_vel -= 15;

/*if(is_first_slow.load()){
    //target_yaw = 1.6;
    auto now = std::chrono::steady_clock::now();
    double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(now - slow_stop_start_time_).count();
    
    // 如果停止时间不足，保持停止状态
    if (elapsed_seconds < SLOW_STOP_DURATION_low) {
        linear_vel = -0.4;
        angular_vel = 0.0;
        RCLCPP_WARN(get_logger(), "In slow stop period (%.1f/%.1f seconds)", 
                   elapsed_seconds, SLOW_STOP_DURATION);
    }else{
        is_first_slow.store(false);
    }
    //is_first_slow_.store(true);
}*/

is_obs_local_t.store(isPastLastCone_first(current_pose.pose,grid_copy));
SyncPlannerNode::ConeInfo current_cone = getMaxConeInfo();
//if(is_second.load() && is_findt_ok.load() && current_cone.is_center)

if((is_obs_local_t.load() && !is_second.load()) 
|| (obs_count  > 2 && !is_second.load())
|| (is_second.load() && !is_findt_ok.load())){
    State = checkAvoidanceCondition(current_cone,current_pose,current_path);
    
    if(is_avoiding_.load()){
        linear_vel = State.linear_vel;
        angular_vel = State.angular_vel;
    }
}


// 7. 发布控制命令
if(cmd_vel.load()){
cmd.linear.x = linear_vel;
cmd.angular.z = angular_vel;
cmd_vel_pub_->publish(cmd);
//RCLCPP_WARN(get_logger(), "vel publish, z: %.2f", angular_vel);
update_current_speed(linear_vel);
}
}

double SyncPlannerNode::adjustYawSmoothly(double target_yaw) {
    const double MAX_YAW_RATE = 0.0015; // 最大角度变化率(rad/control cycle)
    const double MAX_YAW_RATE_ = 0.002; // 最大角度变化率(rad/control cycle)
    // 1. 计算当前理论目标角度
    double raw_target_yaw = target_yaw;
    
    // 2. 计算允许的角度变化范围
    if(!is_first_last_yaw.load()){
        last_target_yaw_ = raw_target_yaw;
        is_first_last_yaw.store(true);
    }
    double min_yaw = last_target_yaw_ - MAX_YAW_RATE;
    double max_yaw = last_target_yaw_ + MAX_YAW_RATE_;
    
    // 3. 渐进式调整
    double smoothed_yaw;
    /*if (target_yaw < -1.6) {
        // 递增策略：不超过最大变化率
        smoothed_yaw = min_yaw; 
        smoothed_yaw = std::max(smoothed_yaw, -3.1); // 增加10%上限
    } else {
        // 递减策略：平滑减小到0.1
        smoothed_yaw = max_yaw;
        smoothed_yaw = std::min(smoothed_yaw, -1.5); // 下限0.1
    }*/
    smoothed_yaw = min_yaw; 
    smoothed_yaw = std::max(smoothed_yaw, -3.1); // 增加10%上限
    // 4. 更新记录值
    last_target_yaw_ = smoothed_yaw;
    return smoothed_yaw;
}

double SyncPlannerNode::pointToLineDistance(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Pose& line_start,
    const geometry_msgs::msg::Pose& line_end) 
{
    double ax = pose.pose.position.y + x_dff - line_start.position.x;
    double ay = pose.pose.position.x + y_dff - line_start.position.y;
    double bx = line_end.position.x - line_start.position.x;
    double by = line_end.position.y - line_start.position.y;

    double projection = (ax * bx + ay * by) / (bx * bx + by * by);
    projection = std::clamp(projection, 0.0, 1.0);

    double closest_x = line_start.position.x + projection * bx;
    double closest_y = line_start.position.y + projection * by;
    
    return std::hypot(pose.pose.position.y+x_dff - closest_x, 
                     pose.pose.position.x+y_dff - closest_y);
}

/**
 * @brief 计算动态调节系数（ROS 2消息版本）
 */
double SyncPlannerNode::calculateDynamicCoeff(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const nav_msgs::msg::Path& current_path,
    const std::vector<std::pair<float, float>>& static_obstacles,
    double path_threshold,
    double obs_threshold) 
{
    // 1. 计算路径偏移距离
    double min_path_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < current_path.poses.size() - 1; ++i) {
        double dist = pointToLineDistance(
            current_pose,
            current_path.poses[i].pose,
            current_path.poses[i+1].pose
        );
        min_path_dist = std::min(min_path_dist, dist);
    }

    // 2. 计算最近障碍物距离
    double min_obs_dist = std::numeric_limits<double>::max();
    for (const auto& obs : static_obstacles) {
        double dist = std::hypot(
            current_pose.pose.position.y + x_dff - obs.first,
            current_pose.pose.position.x + y_dff- obs.second
        );
        min_obs_dist = std::min(min_obs_dist, dist);
    }

    // 3. 计算系数（保持原有算法）
    double alpha = std::clamp(0.5 + 1.5 * (min_path_dist / path_threshold), 0.5, 2.0);
    double beta = std::clamp(3.0 - 2.0 * (min_obs_dist / obs_threshold), 1.0, 3.0); //2.7
    
    return alpha * beta;
}

// 判断是否需要避障及避障方向
SyncPlannerNode::AvoidanceState SyncPlannerNode::checkAvoidanceCondition(const ConeInfo& cone,
    const geometry_msgs::msg::PoseStamped& current_pose,
    const nav_msgs::msg::Path& path) {
    SyncPlannerNode::AvoidanceState state;
    // 速度相关参数
    const double MAX_SAFE_SPEED = 1.0;  // 最大安全速度(m/s)
    const double MIN_ANGULAR = 0.3;     // 最小角速度(rad/s)
    const double MAX_ANGULAR = 1.5;     // 基础最大角速度(rad/s)

    if(is_second.load()){
        //OBSTACLE_AREA_THRESHOLD = is_findt_ok.load() ? 70000: 50000;
        OBSTACLE_AREA_THRESHOLD =  50000;
        AVOIDANCE_DURATION = 0.25;
        //AVOIDANCE_ANG *= 1.5;
        if(is_findt_ok.load()){
            AVOIDANCE_SPEED = 1.0;
            AVOIDANCE_DURATION = 0.3;
            //AVOIDANCE_ANG *= 1.5;
            OBSTACLE_AREA_THRESHOLD = 80000;
            RCLCPP_INFO(get_logger(), "Avoidance second local.. completed");
        }
    }

    if(!is_second.load()){
        //double k_obs = (current_pose.pose.position.x + y_dff < 2.0 
          //  && (!is_first_slow.load() || !is_first_slow_again.load())) ? 1.5 : 0.8;

        //double k_path = (current_pose.pose.position.x + y_dff < 2.0 
          //      && (!is_first_slow.load() || !is_first_slow_again.load())) ? 0.1 : 0.3;

        OBSTACLE_AREA_THRESHOLD = (current_pose.pose.position.x + y_dff < 2.0 
            && (!is_first_slow.load() || !is_first_slow_again.load())) ? 120000 : 60000; //7
            
        /*double K = calculateDynamicCoeff( 
            current_pose, 
            path, 
            yolo_obstacles,
            k_path,  // path_threshold 0.3
            k_obs   // obs_threshold 0.5
        );*/
    //double dynamic_threshold = base_threshold * K;
    if((is_obs_local_t.load() && obs_count <= 3) || obs_count >= 5 ) OBSTACLE_AREA_THRESHOLD = 50000;
    //OBSTACLE_AREA_THRESHOLD = OBSTACLE_AREA_THRESHOLD;
    //RCLCPP_WARN(this->get_logger(), 
      //  "动态系数: %.2f", K);
    }

       // 2. 如果锥桶靠近图像边缘，降低转向幅度（避免过度转向）
    double edge_factor = 1.0;
    const double EDGE_THRESHOLD = 200.0; // 距离图像边缘的阈值（像素）
    if ((cone.bottom_center < EDGE_THRESHOLD || cone.bottom_center > (640.0 - EDGE_THRESHOLD)) && !is_second.load()) {
        edge_factor = 0.5; // 边缘区域减半转向速度
    }

     // 1. 计算速度影响因子 (0.2~1.0)
     double speed_factor = (1.0 - (current_speed_ / MAX_SAFE_SPEED) * 0.8)*edge_factor;
     speed_factor = std::clamp(speed_factor, 0.2, 1.0);
     double first_slow = 2.0; 

    if (cone.area > OBSTACLE_AREA_THRESHOLD || (is_first_slow.load() && is_first_slow_again.load())
    || (is_depth_second.load())){
        state.should_avoid = true;
        //if(is_first_slow.load()) IMAGE_CENTER_X  = 420;
        state.avoid_left = (cone.bottom_center > IMAGE_CENTER_X);

        if(is_second.load() && is_findt_ok.load()){
            //state.avoid_left = (cone.bottom_center > 340); //250
            state.avoid_left = false; //250
            is_findt_ok_left.store(state.avoid_left);
        }

        if((is_first_slow.load() && is_first_slow_again.load()) && cone.bottom_center < 400 ){
            state.avoid_left = true;
        }else if((is_first_slow.load() && is_first_slow_again.load()) && cone.bottom_center > 400){
            is_avoiding_.store(false);
            is_first_slow.store(false);
            is_first_slow_again.store(false);
            max_cone_info_ = ConeInfo(); // 重置为默认值
            RCLCPP_INFO(get_logger(), "Avoidance not first completed");
            state.should_avoid = false;
            return state;
        }
        

        double base_angular_vel = AVOIDANCE_ANG*(1 - cone.area/307200);

        if (!is_avoiding_.load()) {
            // 新避障开始
            is_avoiding_ .store(true);
            avoidance_start_time_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(get_logger(), "Start avoidance: %s at area %.1f", 
                       state.avoid_left ? "LEFT" : "RIGHT", cone.area);
        }
        
        // 计算剩余避障时间
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - avoidance_start_time_).count();

        if((!is_first_slow.load() || !is_first_slow_again.load()) && !is_second.load() 
        && current_pose.pose.position.x + y_dff < 2.0){
        AVOIDANCE_DURATION = 0.2;
        } 

        state.remaining_time = AVOIDANCE_DURATION - elapsed;
        
        if (state.remaining_time > 0) {
            state.linear_vel = is_second.load() ? AVOIDANCE_SPEED*0.8 : AVOIDANCE_SPEED;
            state.angular_vel = state.avoid_left ? base_angular_vel*speed_factor 
            : -base_angular_vel*speed_factor; //5.0
            if(is_first_slow.load()) state.angular_vel *= first_slow; 

        } else {
            // 避障时间结束
            is_avoiding_.store(false);
            is_first_slow.store(false);
            is_first_slow_again.store(false);
            max_cone_info_ = ConeInfo(); // 重置为默认值
            if(is_findt_ok.load()) is_findt_ok_.store(true);
            RCLCPP_INFO(get_logger(), "Avoidance completed");
            state.should_avoid = false;
        }
    }
    return state;
}

std::vector<std::pair<int, int>> SyncPlannerNode::a_star(std::vector<std::vector<float>> &Map, 
    const std::pair<int, int> &start, 
    const std::pair<int, int> &des,
    double yaw,
    const geometry_msgs::msg::PoseStamped& current_pose) {

    
    std::vector<std::pair<int, int>> path;
    std::pair<int, int> now_loc;
    std::pair<int, int> now_des;
    std::pair<int, int> now_start;
    std::pair<int, int> now_loc_des;
    std::pair<int, int> now_loc_des_second;
    int max_retries = 2;  // 最大重试次数
    int retry_count = 0;
    bool success = false;
    bool is_local = false;

    // 初始参数
    int step_length = is_second.load() ? 10 : 10;
    //if(is_local_.load()) step_length = 15;
    double angle_k = is_second.load() ? 10.0 : 20.0;
    double des_m = is_second.load()? 10.0 : 15.0;
    int right_off = 50; //50
    int left_off = -50; //-50
    bool is_hat_left = false;
    bool is_hat_right = false;
    std::pair<int, int> now_start_second;
    std::pair<int, int> now_des_second;
    
    //下面这四行在防止越界
    if(!is_second.load()){
    now_start = {std::min(start.first, 30), std::min(start.second, 70)}; //max
    now_start = {std::max(now_start.first, 15), std::max(now_start.second, 35)}; //min
    if(is_local_.load() || is_local_first.load()) now_start = start;
    now_des = {std::min(des.first, 190), std::min(des.second, 499)};
    now_des = {std::max(now_des.first, 140), std::max(now_des.second, 440)};

    /*now_des = find_PastLastCone(Map);
    if(now_des.first > 80 && now_des.second - 25 < 420){
        is_local = true;
        now_loc_des = des;

    }else{
        now_des = des;
        is_local = false;
    }*/
    //if(is_local_.load()) now_des = des;
    /*if(is_local_.load()){
        std::pair<int, int> start_local = std::make_pair((current_pose.pose.position.y + x_dff)*100, 
        (current_pose.pose.position.x + y_dff)*100);
        //std::vector<Eigen::Vector3f> depth_obstacles;
    {
        /*std::lock_guard<std::mutex> lock(obstacle_mutex_);
        for (const auto& [id, obs] : obstacle_history_) {
            depth_obstacles.push_back(obs);
        }
    }
        return straight_line_with_obstacle_avoidance(Map,start_local,now_des);
    }*/
        
    }else{
    now_start = {std::min(start.first, 250), std::min(start.second, 250)};
    now_start = {std::max(now_start.first, 150), std::max(now_start.second, 180)};
    now_des = {std::min(des.first, 80), std::min(des.second, 80)};
    now_des = {std::max(now_des.first, 0), std::max(now_des.second, 0)};
    now_start_second = now_start;
    now_des_second = {30,30};
    /*if(is_local_second.load()){
        now_start = start;
        return bresenham_line(now_start, now_des,15);
    }*/
    //return straight_line_with_obstacle_avoidance(Map,now_start,now_des);
    return bresenham_line(now_start, now_des, 7);
    //now_loc_des_second = now_des;
    //now_des = {80,160};
    }

     // 预处理：生成启发式网格
     std::vector<std::vector<double>> heuristic_grid = dijkstra_heuristic(now_des, Map);

     // A* 主逻辑
     auto get_heuristic = [&](int x, int y) {
         return heuristic_grid[y][x]; // 直接查表
     };

    std::pair<int, int> last_valid_point = now_des;  // 初始化为终点

    //如果起点被障碍物遮住了，就找最近的没被遮住的点
    if (Map[now_start.second][now_start.first] != 0.0f) {
        now_start = find_nearest_free_point(Map, now_start);
    }
    
    now_loc=now_des; //出于实际经验我们是从终点往小车位置搜的 
    std::vector<std::vector<std::pair<int, int>>> pre(map_height, std::vector<std::pair<int, int>>(map_width, {-1, -1}));
    std::pair<int, int> tmp = now_start;
    
      // 定义局部get_cost函数 (lambda表达式)
      auto get_cost = [&](const std::pair<int, int>& p) -> double {
        double cost = 1.0; // 基础通行代价
        
        // 1. 静态障碍物代价
        float cell_value = Map[p.second][p.first];
        /*if (cell_value > 0.9f) { // 确定障碍物
            return std::numeric_limits<double>::max(); // 不可通行区域
        }*/
        
        // 2. 深度可信度加权
        if (cell_value > 0.7f) {
            cost += 50.0; // 高置信度障碍物
        } else if (cell_value > 0.4f) {
            cost += 30.0;  // 中等置信度
        } else if (cell_value > 0.1f) {
            cost += 5.0;   // 低置信度
        }
        
        // 3. 动态障碍物惩罚
        auto obstacles = get_tracked_obstacles();
        for (const auto& obs : obstacles) {
            // 转换为网格坐标 (假设网格单位是厘米)
            float obs_x = obs.position.x() * 100;
            float obs_y = obs.position.y() * 100;
            
            float dist = std::hypot(p.first - obs_x, p.second - obs_y);
            
            // 动态障碍物影响范围 (50厘米)
            if (dist < 50.0) {
                // 距离越近惩罚越大
                double penalty = 50.0 * (1.0 - dist/50.0);
                
                // 考虑运动方向 (朝向我们的障碍物更危险)
                float relative_angle = std::atan2(obs.velocity.y(), obs.velocity.x());
                float danger_factor = 1.0 + std::cos(relative_angle);
                
                cost += penalty * danger_factor;
            }
        }
        
        return cost;
    };

    do {
    	
        std::priority_queue<AStarNode> priority_queue;
        priority_queue.push(AStarNode(last_valid_point, 0.0));  // 从最后一个有效点开始

        // 重置 visited 和 pre
        std::vector<std::vector<double>> visited(map_height, std::vector<double>(map_width, -1.0f));
        visited[last_valid_point.second][last_valid_point.first] = 0.0f;

        int cnt = 0;
        bool force_path = (retry_count == max_retries - 1);  // 最后一次尝试强制找到路径
        if(force_path) step_length = step_length / max_retries-1;

        while (!priority_queue.empty()) {
            cnt++;
            if (cnt > 100000) {
                RCLCPP_WARN(get_logger(), "Max steps exceeded");
                break;
            }

            AStarNode current_node = priority_queue.top();
            priority_queue.pop();
            now_loc = current_node.pos;

            if (get_dis(now_loc, now_start) <= des_m) {
                tmp = now_loc;
                success = true;  // 标记成功
                RCLCPP_WARN(get_logger(), "tmp now_loc");
                break;
            }
            
            last_valid_point = now_loc; // 保存最后一个有效点

            for (int angle = 0; angle < 360; angle += 30) {
                int i = static_cast<int>(now_loc.second + step_length * cos(angle * M_PI / 180.0));
                int j = static_cast<int>(now_loc.first + step_length * sin(angle * M_PI / 180.0));

                if (i >= (!is_second.load() ? 0 : 0) && i < (!is_second.load() ? 490 : 460) && 
                    j >= (!is_second.load() ? 15 : 15) && j <(!is_second.load() ? 190 : 200)) {

                    bool can_pass = force_path || Map[i][j] == 0.0f;
                    double additional_cost = get_cost({j,i});
                    if (additional_cost >= std::numeric_limits<double>::max()) {
                        continue;
                    } 
                    
                    double get_heuristic_d = get_heuristic(j,i);

                    if (can_pass && (visited[i][j] == -1.0f || visited[now_loc.second][now_loc.first] + step_length < visited[i][j])) {
                    double now_dis=get_dis({j, i}, now_start);
                    //double now_dis= is_second.load()?heuristic({j, i}, now_start,step_length):get_dis({j, i}, now_start);
                    double current_angle = atan2(i - now_loc.second, j - now_loc.first);
                    double angle_diff = 0.0;
                    if (pre[now_loc.second][now_loc.first].first != -1) {
                        auto prev = pre[now_loc.second][now_loc.first];
                        double prev_angle = atan2(now_loc.second - prev.second, now_loc.first - prev.first);
                        angle_diff = fabs(current_angle - prev_angle);  // 计算方向变化
                    }
                    double turn_penalty = angle_diff * angle_k;  // 系数  可调整

                    // 障碍物惩罚
                    double obstacle_penalty = 0.0;
                    int check_radius = 2;
                    for (int dy = -check_radius; dy <= check_radius; ++dy) {
                        for (int dx = -check_radius; dx <= check_radius; ++dx) {
                            int ny = i + dy;
                            int nx = j + dx;
                                if (ny >= 0 && ny < map_height && nx >= 0 && nx < map_width && 
                                    Map[ny][nx] != 0.0f) {
                                    double dist = sqrt(dx*dx + dy*dy);
                                    obstacle_penalty += fabs((check_radius - dist)) * 30.0;
                                }
                            }
                        }


                    pre[i][j] = now_loc;
                    /*double f = force_path ? visited[now_loc.second][now_loc.first] + 1
                    :visited[now_loc.second][now_loc.first] + step_length + now_dis + turn_penalty; //A*估价函数*/

                    double dy = i - now_des.second;
                    double dx = j - now_des.first;

                    double angle = atan2(dy, dx); // 当前点到终点的方向角
                    double tie_breaker = 1.0 + 0.01 * std::abs(sin(angle)); // 优先轴向移动

                    /*double f = is_local_.load() ? visited[now_loc.second][now_loc.first] + step_length + now_dis + additional_cost
                    :visited[now_loc.second][now_loc.first] + step_length + now_dis + turn_penalty + additional_cost; //A*估价函数*/
                    double f = is_second.load() ? visited[now_loc.second][now_loc.first] + step_length + additional_cost + get_heuristic_d*1.5
                    :visited[now_loc.second][now_loc.first] + step_length + now_dis + additional_cost + turn_penalty;

                    /*if(is_second.load()){
                        f = visited[now_loc.second][now_loc.first] + step_length + additional_cost + get_heuristic_d*1.5; 
                    }*/
                        priority_queue.push(AStarNode({j, i}, f));
                        visited[i][j] = visited[now_loc.second][now_loc.first] + step_length;
                        if(force_path && !is_second.load()) return straight_line_with_obstacle_avoidance(Map,now_start,now_des);
                        if(is_second.load() && retry_count == 1) return straight_line_with_obstacle_avoidance(Map,now_start_second,now_des_second);;
                    }
                }
            }
        }

        if (!success && priority_queue.empty()) {
            RCLCPP_WARN(get_logger(), "Priority queue empty, retrying with relaxed conditions (retry %d/%d)", retry_count + 1, max_retries);
            step_length *= 2;  // 增大步长
            angle_k /= 2;      // 减少转向惩罚
            retry_count++;
        }
    } while (!success && retry_count < max_retries);

    if (!success) {
        RCLCPP_ERROR(get_logger(), "Failed to find path after %d retries", max_retries);
        path = bresenham_line(now_start, now_des,3);
    }

    // 回溯路径
    int max_backtrack = 100; // 防止无限循环
    int backtrack_count = 0;
    while (success && pre[tmp.second][tmp.first].first != -1 && backtrack_count < max_backtrack && 
        get_dis({tmp.first, tmp.second}, now_des) > des_m/2) {
        path.push_back(tmp);
        RCLCPP_WARN(get_logger(), "Path back track point: (%d, %d)", tmp.first, tmp.second); 
        tmp = pre[tmp.second][tmp.first];
        backtrack_count++;
    }
    
    /*if(!is_second.load() && is_local){
    std::vector<std::pair<int, int>> path_ = bresenham_line(now_des, now_loc_des,10);
    path.insert(path.end(), path_.begin(), path_.end());
    }*/

    /*if(is_second.load()){
        //std::vector<std::pair<int, int>> path_ = bresenham_line(now_des, now_loc_des_second,8);
        //straight_line_with_obstacle_avoidance(Map,now_start,now_des);
        //std::vector<std::pair<int, int>> path_ = straight_line_with_obstacle_avoidance(Map,now_des,now_loc_des_second);
        //path.insert(path.end(), path_.begin(), path_.end());
        path.insert(path.end(), std::make_pair(40, 10));
        path.insert(path.end(), std::make_pair(50, 10));
    }*/
   // 在函数返回路径前添加以下逻辑：
/*if (!path.empty() && !is_second.load()) {
    // 检查路径终点附近 y < 150 的区域是否有障碍物
    bool has_obstacle = false;
    const int check_radius = 5;  // 检查周围5个像素的范围
    const int y_threshold = 150; // y坐标阈值
    
    // 获取当前路径的最后一个点
    auto last_point =  path.front();
    
    // 检查该点附近 y < 150 的区域
    for (int dy = -check_radius; dy <= check_radius; ++dy) {
        for (int dx = -check_radius; dx <= check_radius; ++dx) {
            int y = last_point.second + dy;
            int x = last_point.first + dx;
            
            // 确保坐标在合法范围内
            if (y >= 0 && y < map_height && x >= 0 && x < map_width) {
                // 如果 y < 150 且该位置有障碍物
                if (y < y_threshold && Map[y][x] != 0.0f) {
                    has_obstacle = true;
                    break;
                }
            }
        }
        if (has_obstacle) break;
    }
    
    // 如果没有障碍物，则增加一个目标点（例如向右延伸）
    if (!has_obstacle) {
        int new_x = 20;  // 向右延伸20单位
        int new_y = 55;
        
        // 确保新点在合法范围内
        new_x = std::min(new_x, map_width - 1);
        new_y = std::min(new_y, map_height - 1);
        
        // 将新点加入路径
        path.emplace_back(new_x, new_y);
        RCLCPP_INFO(get_logger(), "Added new point (%d, %d) due to clear space in y < 150", new_x, new_y);
    }
}*/
if(!is_second.load())
{
std::vector<std::pair<int, int>> cone_positions;
const int height = Map.size();
const int width = Map[0].size();

for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
        if (Map[y][x] > 0.0) {
            cone_positions.emplace_back(x, y);
        }
    }
}


auto first_cone_it = std::min_element(
    cone_positions.begin(),
    cone_positions.end(),
    [](const auto& a, const auto& b) {
        return a.second < b.second;
    });

const int first_cone_y = first_cone_it->second + 25;
if(first_cone_y > 150){
    int new_x = 20;  // 向右延伸20单位
    int new_y = 70;
        
        // 确保新点在合法范围内
    new_x = std::min(new_x, map_width - 1);
    new_y = std::min(new_y, map_height - 1);
        
        // 将新点加入路径
    path.insert(path.begin(), std::make_pair(new_x, new_y));
    is_first_ok.store(true);
    RCLCPP_INFO(get_logger(), "Added new point (%d, %d) due to clear space in y < 150", new_x, new_y);
}else if (first_cone_y < 120){

    is_first_slow_again.store(true);
    RCLCPP_INFO(get_logger(), "Added new point");
}
}

    pose_initialized_.store(false);
    return path;
}

std::vector<std::pair<int, int>> SyncPlannerNode::straight_line_with_obstacle_avoidance(
    std::vector<std::vector<float>> &Map,
    const std::pair<int, int> &start,
    const std::pair<int, int> &des) 
{
    // 1. 基本检查
    if (Map.empty() || Map[0].empty()) {
        RCLCPP_ERROR(get_logger(), "Invalid map dimensions");
        return {};
    }

    // 2. 生成直线路径
    std::vector<std::pair<int, int>> straight_path = bresenham_line(start, des, 10);
    
    // 3. 检查路径上的障碍物冲突
    std::vector<std::pair<int, int>> affected_centers;
    const int obstacle_radius = 30; // 60cm直径对应30单元格半径
    
    for (const auto& center : yolo_obstacles) {
        // 将浮点坐标转为整型
        std::pair<int, int> int_center = {
            static_cast<int>(center.first),
            static_cast<int>(center.second)
        };
        
        // 检查障碍物是否在特殊区域内 (0 < x < 100 且 y < 150)
        /*if (int_center.first < 100 && int_center.second < 120) {
            // 如果在特殊区域内，跳过避障
            //is_local_obs.store(true);
            continue;
        }*/
        
        // 检查路径点是否在障碍物影响范围内
        for (const auto& path_point : straight_path) {
            double dist = std::hypot(
                path_point.first - int_center.first,
                path_point.second - int_center.second);
            
            if (dist <= obstacle_radius) {
                affected_centers.push_back(int_center);
                break; // 每个障碍物只需记录一次
            }
        }
    }
    
    // 4. 如果没有障碍物冲突，直接返回直线路径
    if (affected_centers.empty()) {
        return straight_path;
    }
    
    // 5. 对每个冲突障碍物生成避障路径
    std::vector<std::pair<int, int>> final_path;
    std::pair<int, int> current = start;
    
    for (const auto& center : affected_centers) {
        // 5.1 计算避障起点和终点
        auto [avoid_start, avoid_end] = calculate_avoidance_points(
            straight_path, current, center, des, obstacle_radius,Map);
        
        // 5.2 添加当前点到避障起点的路径
        auto to_avoid = bresenham_line(current, avoid_start, 10);
        final_path.insert(final_path.end(), to_avoid.begin(), to_avoid.end());
        
        // 5.3 添加避障路径
        auto avoid_path = generate_arc_path(avoid_start, center, avoid_end,Map);
        final_path.insert(final_path.end(), avoid_path.begin(), avoid_path.end());
        
        // 5.4 更新当前位置
        current = avoid_end;
    }
    
    // 6. 添加最后一段到终点的路径

    if(is_local_obs.load()){
    std::pair<int, int> des_ = {100,70};
    auto last_segment_first = bresenham_line(current, des_, 10);
    final_path.insert(final_path.end(), last_segment_first.begin(), last_segment_first.end());
    std::pair<int, int> des = {20, 50};
    auto last_segment = bresenham_line(des_, des, 10);
    final_path.insert(final_path.end(), last_segment.begin(), last_segment.end());
    }else{
    auto last_segment = bresenham_line(current, des, 10);
    final_path.insert(final_path.end(), last_segment.begin(), last_segment.end());
    }
    
    return final_path;
}

std::pair<std::pair<int, int>, std::pair<int, int>> 
SyncPlannerNode::calculate_avoidance_points(
    const std::vector<std::pair<int, int>>& straight_path,
    const std::pair<int, int>& current_pos,
    const std::pair<int, int>& obstacle_center,
    const std::pair<int, int>& destination,
    int obstacle_radius,
    const std::vector<std::vector<float>>& Map) 
{
    const int approach_margin = is_second.load() ? 10 : 15; // 安全距离 //10 15
    
    // 计算避障起点（障碍物前方安全点）
    std::pair<int, int> avoid_start = current_pos;
    for (const auto& point : straight_path) {
        double dist = std::hypot(
            point.first - obstacle_center.first,
            point.second - obstacle_center.second);
        
        if (dist <= obstacle_radius + approach_margin) {
            avoid_start = find_nearest_safe_point(Map, point, approach_margin);
            break;
        }
    }
    
    // 计算避障终点（障碍物后方安全点）
    std::pair<int, int> avoid_end = destination;
    bool found_obstacle = false;
    for (const auto& point : straight_path) {
        double dist = std::hypot(
            point.first - obstacle_center.first,
            point.second - obstacle_center.second);
        
        if (dist <= obstacle_radius) {
            found_obstacle = true;
        } else if (found_obstacle) {
            avoid_end = find_nearest_safe_point(Map, point, approach_margin );
            break;
        }
    }
    
    return {avoid_start, avoid_end};
}

std::vector<std::pair<int, int>> SyncPlannerNode::generate_arc_path(
    const std::pair<int, int>& start,
    const std::pair<int, int>& center,
    const std::pair<int, int>& end,
    const std::vector<std::vector<float>>& Map) 
{
    std::vector<std::pair<int, int>> arc_path;
    
    // 计算绕行方向（向左或向右） 
    //bool go_clockwise = should_avoid_right(Map, start, center, end);
    bool go_clockwise = true;
    
    // 计算绕行半径（障碍物半径+安全距离）
    const int safety_margin = is_second.load() ? 15 : 15; //15 //18 //10
    const int total_radius = 35 + safety_margin; // 30是障碍物半径
    
    // 计算起始和结束角度
    double start_angle = atan2(start.second - center.second, start.first - center.first);
    double end_angle = atan2(end.second - center.second, end.first - center.first);
    
    // 调整角度方向
    /*if (go_clockwise) {
        if (end_angle > start_angle) start_angle += 2*M_PI;
    } else {
        if (end_angle < start_angle) end_angle += 2*M_PI;
    }*/

    if (end_angle > start_angle) {
        start_angle += 2 * M_PI;  // 跨0度时调整连续性
    }
    
    // 生成圆弧点
    const int steps = 8;
    for (int i = 1; i < steps; ++i) {
        double ratio = static_cast<double>(i) / steps;
        double angle = start_angle + (end_angle - start_angle) * ratio;
        int x = center.first + total_radius * cos(angle);
        int y = center.second + total_radius * sin(angle);
        arc_path.emplace_back(x, y);
    }
    
    return arc_path;
}

bool SyncPlannerNode::should_avoid_right(
    const std::vector<std::vector<float>>& Map,
    const std::pair<int, int>& start,
    const std::pair<int, int>& center,
    const std::pair<int, int>& end)
{
    // 检查障碍物位置决定绕行方向
    // 如果在0<x<100有障碍物，且y>60则向右绕行，否则向左绕行
    
    // 检查中心点附近的障碍物
    const int check_distance = 30;
    bool has_obstacle_above_60 = false;
    bool has_obstacle_below_60 = false;
    
    for (int dy = -check_distance; dy <= check_distance; ++dy) {
        for (int dx = -check_distance; dx <= check_distance; ++dx) {
            int x = center.first + dx;
            int y = center.second + dy;
            
            // 检查是否在0<x<100范围内
            if (x > 0 && x < 100 && y >= 0 && y < Map.size() && x < Map[0].size()) {
                if (Map[y][x] > 0.1f) {  // 有障碍物
                    if (y > 60) {
                        has_obstacle_above_60 = true;
                    } else {
                        has_obstacle_below_60 = true;
                    }
                }
            }
        }
    }
    
    // 决策逻辑
    /*if (has_obstacle_above_60) {
        return true;  // 向右绕行
    } else if (has_obstacle_below_60) {
        return false; // 向左绕行
    }*/
    
    // 默认情况（如果没有障碍物），使用原始启发式方法
    int right_score = 0;
    int left_score = 0;
    
    // 计算右侧障碍物密度
    for (int dy = -check_distance; dy <= check_distance; ++dy) {
        for (int dx = 0; dx <= check_distance; ++dx) {
            int x = center.first + dx;
            int y = center.second + dy;
            if (x >= 0 && x < Map[0].size() && y >= 0 && y < Map.size()) {
                right_score += Map[y][x] > 0.1f ? 1 : 0;
            }
        }
    }
    
    // 计算左侧障碍物密度
    for (int dy = -check_distance; dy <= check_distance; ++dy) {
        for (int dx = -check_distance; dx <= 0; ++dx) {
            int x = center.first + dx;
            int y = center.second + dy;
            if (x >= 0 && x < Map[0].size() && y >= 0 && y < Map.size()) {
                left_score += Map[y][x] > 0.1f ? 1 : 0;
            }
        }
    }
    
    return right_score < left_score; // 右侧障碍物较少则向右绕行
}

std::pair<int, int> SyncPlannerNode::find_nearest_safe_point(
    const std::vector<std::vector<float>>& Map,
    const std::pair<int, int>& reference_point,
    int safe_margin) 
{
    // 螺旋搜索周围的点
    const int max_search_radius = 10;
    for (int r = 0; r <= max_search_radius; ++r) {
        for (int x = -r; x <= r; ++x) {
            for (int y = -r; y <= r; ++y) {
                if (abs(x) != r && abs(y) != r) continue; // 只检查外围
                
                int check_x = reference_point.first + x;
                int check_y = reference_point.second + y;
                
                if (check_x >= 0 && check_x < Map[0].size() &&
                    check_y >= 0 && check_y < Map.size() &&
                    is_point_safe(Map, {check_x, check_y}, safe_margin)) {
                    return {check_x, check_y};
                }
            }
        }
    }
    
    // 如果找不到理想点，返回原始点（最后手段）
    return reference_point;
}

bool SyncPlannerNode::is_point_safe(
    const std::vector<std::vector<float>>& Map,  // 改为 int 类型，存储 0 或 1
    const std::pair<int, int>& point,
    int safe_distance) 
{
    // 检查点本身是否是障碍物（1）
    if (Map[point.second][point.first] == 1.0) {
        return false;
    }
    
    // 检查周围安全距离内的点
    for (int dy = -safe_distance; dy <= safe_distance; ++dy) {
        for (int dx = -safe_distance; dx <= safe_distance; ++dx) {
            int nx = point.first + dx;
            int ny = point.second + dy;
            
            // 检查是否在地图范围内
            if (nx >= 0 && nx < map_width && ny >= 0 && ny < map_height) {
                if (Map[ny][nx] == 1.0) {
                    return false;  // 发现障碍物
                }
            }
        }
    }
    
    return true;  // 安全
}

void SyncPlannerNode::subscription_callback_return(
    const std_msgs::msg::Int32::SharedPtr msg) {
    if (msg->data == 0) {
      is_second.store(false);
      std::lock_guard<std::mutex> lock1(plan_mutex_);
      {
          is_plan.store(true);
          cmd_vel.store(true);
          is_find.store(true);
      }
    }
    if(msg->data == 6){
        is_second.store(true);
        // 创建消息过滤器订阅
        std::lock_guard<std::mutex> lock2(plan_mutex_);
    {
        is_plan.store(true);
        cmd_vel.store(true);
        is_find.store(true);
        stop_planning_.store(false);
        //is_sync.store(true);
    }
    hat_subscriber_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        "racing_obstacle_detection", 10,
        std::bind(&SyncPlannerNode::subscription_callback_target, this, std::placeholders::_1)); 

    depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/aurora/depth/image_raw", 
        10, 
        std::bind(&SyncPlannerNode::depth_image_callback_, this, std::placeholders::_1));
    //x_ y_
    /*{
    geometry_msgs::msg::PoseStamped current_pose;
    if (!get_current_pose(current_pose)) {
    return;
    }
    x_ = current_pose.pose.position.x + y_dff - x_ok;
    y_ =  current_pose.pose.position.y + x_dff - y_ok;
    }*/
    subscriber_return.reset();
    }

    if(msg->data == 5){
        auto cmd_vel_message = geometry_msgs::msg::Twist();
        cmd_vel_message.linear.x = 0.0, cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
        cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0, cmd_vel_message.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel_message);
    }
}

void SyncPlannerNode::subscription_callback_switch(
    const origincar_msg::msg::Sign::SharedPtr msg) {
    // 如果扫描到二维码，会被调用，关闭hbmem和sign_switch的订阅
    if (msg->sign_data == 3 || msg->sign_data == 4) {
        auto cmd_vel_message = geometry_msgs::msg::Twist();
        cmd_vel_message.linear.x = 0.0, cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
        cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0, cmd_vel_message.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel_message);
        is_find.store(false);
      stop_planning_.store(true);
      pose_initialized_.store(false);
      cmd_vel.store(false);
      is_second.store(true);
      is_plan.store(false);
      is_qr.store(false);
      is_hat.store(true);
      is_local_ok.store(false);
      //count_second = 0;
      //is_local.store(false);
      //is_local_.store(false);
     /* { 
        std::lock_guard<std::mutex> lock(grid_mutex_);
       occupancy_grid_ = std::vector<std::vector<float>>(500, std::vector<float>(200, 0.0f));
      }
      last_closest_idx = 0;
      //is_sync.store(true);
      depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/aurora/depth/image_raw", 
         10, 
         std::bind(&SyncPlannerNode::depth_image_callback_, this, std::placeholders::_1));*/
    obstacle_detected.store(false);
    depth_sub.reset();
    
    RCLCPP_WARN(get_logger(), "Topic sign_switch receive msg equals 3 or 4!");
    subscriber_switch.reset();
    } else {
      RCLCPP_INFO(rclcpp::get_logger("racingcontrol_node"), "Topic sign_switch did not receive msg equals 3 or 4!");
    }
  }
  

std::pair<int, int> SyncPlannerNode::find_nearest_free_point(
    const std::vector<std::vector<float>> &Map,
    const std::pair<int, int> &start) {
    
    std::queue<std::pair<int, int>> q;
    std::vector<std::vector<bool>> visited(map_height, std::vector<bool>(map_width, false));
    
    q.push(start);
    visited[start.first][start.second] = true;

    // 8方向搜索（可以改成4方向：上、下、左、右）
    const int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};

    while (!q.empty()) {
        auto current = q.front();
        q.pop();

        // 如果当前点是可通行的（无障碍），返回该点
        if (Map[current.first][current.second] == 0.0f) {
            RCLCPP_WARN(get_logger(),"return current");
            return current;
        }

        // 检查8个方向
        for (int i = 0; i < 8; ++i) {
            int nx = current.first + dx[i];
            int ny = current.second + dy[i];

            // 检查是否越界 & 是否已访问
            if (nx >= 0 && nx < map_height && ny >= 0 && ny < map_width && !visited[nx][ny]) {
                visited[nx][ny] = true;
                q.push({nx, ny});
            }
        }
    }

    // 如果所有点都被障碍物占据（理论上不应该发生），返回起点
    RCLCPP_WARN(get_logger(),"return start");
    return start;
}

double SyncPlannerNode::get_dis(const std::pair<int, int> &a, const std::pair<int, int> &b) {
    int dx = a.first - b.first;
    int dy = a.second - b.second;
    return std::sqrt(dx * dx + dy * dy);
}

std::pair<int, int> SyncPlannerNode::inverse_perspective_point(int x, int y) {
    static const std::vector<std::vector<double>> matrix_first = {
        {-6.70309244e-02 ,-4.63994704e-01 , 1.23194245e+02},
        {-1.70032916e-03 ,-3.97530534e-02 ,-7.05156910e+00},
        {-4.55938343e-05 ,-4.51456869e-03 , 1.00000000e+00}
    };
    
    static const std::vector<std::vector<double>> matrix_second = {
        {-4.50626410e-02, -8.33512063e-04 , 1.32532572e+01},
        {-1.55471016e-03 ,-5.51702090e-02 ,-1.45588961e+00},
        {-5.03831816e-05 ,-4.42349834e-03 , 1.00000000e+00}
    };

    // 根据 is_second 选择矩阵
    const auto& matrix = !is_second.load() ? matrix_first : matrix_second;

    std::vector<double> homogeneous_point = {static_cast<double>(x), static_cast<double>(y), 1.0};
    std::vector<double> transformed_point(3, 0.0);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            transformed_point[i] += matrix[i][j] * homogeneous_point[j];
        }
    }

    transformed_point[0] /= transformed_point[2];
    transformed_point[1] /= transformed_point[2];
    int out_x;
    int out_y;
    if(!is_second.load()){
    out_x = static_cast<int>(-round(transformed_point[0]) + 65);
    out_y = static_cast<int>(round(transformed_point[1]));
    if(out_x < 0 && out_y <160 ) out_x += 25;
    if(out_x < 0 && out_y >160 ) out_x -= 5;
    
    }else{
        out_x = static_cast<int>(round(transformed_point[0]));
        out_y = static_cast<int>(round(transformed_point[1]));
    }

    return {out_x, out_y};
}

std::pair<double, double> SyncPlannerNode::local_to_global(double car_x, double car_y, double theta, double c_x, double c_y, int type) {
    double x_df = 13;
    double y_df = -10;
    double x_df_ = 35;
    double y_df_ = 10;
    if(is_second.load()) c_x += 80;
    double cone_dis = sqrt(c_x * c_x + c_y * c_y);
    double adjusted_theta = !is_second.load()?0.0:theta;  
    double global_x;
    double global_y;
    if(!is_second.load()){ 
    //global_x = car_x * 100.0 + cone_dis * cos(atan2(c_y, c_x)-fabs(adjusted_theta)) + x_df;
    //global_y = car_y * 100.0 + cone_dis * sin(atan2(c_y, c_x)-fabs(adjusted_theta)) + y_df;
    c_y+=23.0;
    global_x = c_x + car_x * 100.0;
    global_y = c_y + car_y * 100.0;
    if(is_local_.load() && c_y > 0){
        if(c_x > 0){
            global_x = car_x*100 + cone_dis*cos((M_PI/2 - theta) - (M_PI/2 - atan2(c_y, c_x)));
            global_y = car_y*100 + cone_dis*sin((M_PI/2 - theta) - (M_PI/2 - atan2(c_y, c_x)));
        }else{
            global_x = car_x*100 - cone_dis*cos((theta + M_PI/2) - (fabs(atan2(c_y, c_x) - M_PI/2)));
            global_x = car_y*100 + cone_dis*sin((theta + M_PI/2) - (fabs(atan2(c_y, c_x) - M_PI/2)));
        }
    }
    if(global_y > 320) global_y += 70;
    /*if(!is_local_.load()){
        //global_x = (global_y > 180 && c_x < 70) ? global_x - 25 : global_x;
        //global_y =  global_y > 150 ? global_y - 15 : global_y;
        global_x = c_x + car_x * 100.0;
        global_y = c_y + car_y * 100.0;
    }*/
    }else{
    //global_x = 200 - fabs(c_y) + x_df_;
    //global_y = 250 - fabs(c_x) + y_df_;
    //global_x = car_x * 100.0 + cone_dis * cos((adjusted_theta) -atan2(c_y, c_x)) ;
    //global_y = car_y * 100.0 + cone_dis * sin((adjusted_theta) -atan2(c_y, c_x)) ;
    //global_x = car_x * 100.0 + cone_dis * cos((adjusted_theta) -0.7*atan2(c_y, c_x)) ;
    //global_y = car_y * 100.0*(1.2+ (c_y/200)*0.2) - cone_dis * sin((adjusted_theta) -atan2(c_y, c_x)) ;
    //c_y+=23.0;
    //if(c_y > 0 && c_x > 0){
      //  global_x = 220.0 - fabs(c_y);
        //global_y = 250 - (c_x);
        //if(global_x > 100) global_y+=75;
        //if(global_x<100 && global_y < 150) global_x += 40;
        //if(global_x < 80 && global_y < 70) global_y = 90;
        //if(global_y < 200 && global_y > 100 && global_x > 80) global_y =  global_x<130 ? (200-global_y)*4.5 -30 : (200-global_y)*4.5;
    //}else if(c_y < 0 ){
        //global_x = -1;
        //global_y = -1;
    //}
    //cone_dis = sqrt(c_x * c_x + c_y * c_y);
    //double angle = theta + atan2(c_x, c_y);  // 关键调整点！
    //global_x = car_x*100 - (fabs(c_x * cos(theta)) + fabs(c_y * sin(theta))) ;
    //global_y = car_y*100 + (c_x * sin(theta) + c_y * cos(theta)) ;
    //global_x  = 0.021 * std::pow(global_x, 2) + 1.35 * global_x + 32.2;
    if(c_y > 0){
        if(c_x < 0){
            global_x = 220 - cone_dis * sin(fabs(theta) - (fabs(atan2(c_y, c_x)) - M_PI/2)) - 40;
            global_y = 250 + cone_dis * cos(fabs(theta) - (fabs(atan2(c_y, c_x)) - M_PI/2));
        }else{
            global_x = 220 - cone_dis * sin((M_PI-fabs(theta)) - fabs(M_PI/2 - atan2(c_y, c_x))) - 40;
            global_y = 250 - cone_dis * cos((M_PI-fabs(theta)) - fabs(M_PI/2 - atan2(c_y, c_x)));
        }
    }else{
        global_x = -1;
        global_y = -1;
    }
    }

    RCLCPP_WARN(get_logger(), "LocalToGlobal: car(%.2f,%.2f) theta=%.2f | local(%.2f,%.2f) -> global(%.2f,%.2f)",
             car_x, car_y, theta, c_x, c_y, global_x, global_y);
    return {global_x, global_y};
}

std::pair<double, double> SyncPlannerNode::local_to_global_depth(double car_x, double car_y, double theta, double depth, int type) {
    double global_x;
    double global_y;
    if(!is_second.load()){
        global_x = car_x + depth*sin(theta);
        global_y = car_y + depth*cos(theta);
    }else{
        global_x = car_x - depth*cos(fabs(theta));
        global_y = car_y - depth*sin(fabs(theta));
    }
    return {global_x, global_y};
}

double SyncPlannerNode::predict(double output_x, double output_y) {
    // 计算多项式特征
    double output_x_sq = output_x * output_x;
    double output_x_output_y = output_x * output_y;
    double output_y_sq = output_y * output_y;

    // 应用模型公式: intercept + coef1*x + coef2*y + coef3*x² + coef4*xy + coef5*y²
    double prediction = intercept
        + coefficients[0] * output_x
        + coefficients[1] * output_y
        + coefficients[2] * output_x_sq
        + coefficients[3] * output_x_output_y
        + coefficients[4] * output_y_sq;

    return prediction;
}

geometry_msgs::msg::PoseStamped SyncPlannerNode::find_target_point(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const nav_msgs::msg::Path& path) {

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - last_speed_update_time_).count();
    if (elapsed > 1) {
            current_speed_ = 0.0; // 视为停止状态
    }
    //RCLCPP_WARN(get_logger(), "dynamic_safety_distance:%.2f",dynamic_safety_distance);
    // 1. 获取深度障碍物数据
    std::vector<Eigen::Vector3f> depth_obstacles;
    {
        std::lock_guard<std::mutex> lock(obstacle_mutex_);
        for (const auto& [id, obs] : obstacle_history_) {
            depth_obstacles.push_back(obs);
        }
    }
    
    if(!is_second.load() && is_first_ok.load()){
        is_first_ok.store(false);
        return path.poses[0];
    }
    
    // 1. 寻找路径上离小车最近的点
    size_t closest_idx = 0;
    double min_distance = std::numeric_limits<double>::max(); 
    
    if(!is_second.load()){
        for (size_t i = 0; i < path.poses.size(); ++i) {
            double dx;
            double dy;
            if(!is_second.load()){
             dx = path.poses[i].pose.position.x - (current_pose.pose.position.y+x_dff);
             dy = path.poses[i].pose.position.y - (current_pose.pose.position.x+y_dff);
            }else{
             dx = path.poses[i].pose.position.x - (current_pose.pose.position.x+y_dff_-x_);
             dy = path.poses[i].pose.position.y - (current_pose.pose.position.y+x_dff_-y_);
            }
            double distance = dx*dx + dy*dy;
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_idx = i;
            }
        }
    }else{
        closest_idx = findGeometricClosest(current_pose,path);
    }   

    //if (closest_idx > max_reached_idx && is_second.load()) {
      //  max_reached_idx = closest_idx;
   // }

    //(is_second.load() && current_pose.pose.position.y+x_dff-y_ >1.8 && (count_second == 0)
    //||(current_pose.pose.position.y+x_dff-y_ > 1.8 && is_second.load()
    if(!is_local_ok.load()){
    if((current_pose.pose.position.x+y_dff < 0.8 && !is_second.load())){
    // 3. 检查路径前方是否有障碍物

    if(!obstacle_detected.load()){
    const double CHECK_DISTANCE = 1.0; // 检查前方1米内的路径
    double accumulated_distance = 0.0;

    for (size_t i = 0; i < path.poses.size(); ++i) {
        // 计算当前路径点到机器人的距离
        double dx;
        double dy;
        if(!is_second.load()){
            dx = path.poses[i].pose.position.x - (current_pose.pose.position.y+x_dff);
            dy = path.poses[i].pose.position.y - (current_pose.pose.position.x+y_dff);
           }else{
            dx = path.poses[i].pose.position.x - (current_pose.pose.position.x+y_dff_-x_);
            dy = path.poses[i].pose.position.y - (current_pose.pose.position.y+x_dff_-y_);
           }
        double segment_distance = std::hypot(dx, dy);
        
        // 检查这个路径点附近是否有障碍物
        if ((check_point_near_obstacle(path.poses[i], depth_obstacles))) {
            obstacle_detected.store(true);
            break;
        }
        
        accumulated_distance = segment_distance;
        if (accumulated_distance > CHECK_DISTANCE) {
            break;
        }
    }
    
    // 4. 如果有障碍物，生成避让目标点
    if (obstacle_detected.load()) {
        RCLCPP_WARN(get_logger(), "obstacle_detected!!!!!!!!!!!!!");
        //return generate_avoidance_target(current_pose, path, closest_idx, depth_obstacles);
        avoidance_path = generate_avoidance_path(current_pose, path, closest_idx, depth_obstacles);
        following_avoidance.store(true);
    }
}

}
 // 5. 如果正在跟随避障路径
 if (following_avoidance.load() && !avoidance_path.poses.empty() && obstacle_detected.load() && !is_second.load()) {
    // 获取当前避障路径点
    geometry_msgs::msg::PoseStamped target;
    // 计算到当前目标点的距离
    for (size_t i = closest_idx; i < avoidance_path.poses.size(); ++i) {
        double dx;
        double dy;
        if(!is_second.load()){
            dx = avoidance_path.poses[i].pose.position.x - (current_pose.pose.position.y+x_dff);
            dy = avoidance_path.poses[i].pose.position.y - (current_pose.pose.position.x+y_dff);
        }else{
            dx = avoidance_path.poses[i].pose.position.x - (current_pose.pose.position.x+y_dff_ - x_);
            dy = avoidance_path.poses[i].pose.position.y - (current_pose.pose.position.y+x_dff_ - y_);
        }
    double distance_ = std::hypot(dx, dy);
    static double min_distance_ = 10.0;
    
    // 如果接近当前点，移动到下一个点
    //const double REACH_THRESHOLD = 0.1;
    if (distance_ < min_distance_ && !is_second.load()) {
        min_distance_ = distance_;
        avoidance_idx++;
    }

    if(is_second.load()){
        avoidance_idx++;
        break;
    }
}

    // 如果已经到达避障路径终点，切换回主路径
    if (avoidance_idx >= avoidance_path.poses.size() || local_count > 3) {
        following_avoidance.store(false);
        avoidance_path.poses.clear();  // 清空所有路径点
        obstacle_detected.store(false);
        avoidance_idx = 0;
        local_count = 0;
        is_local_ok.store(true);
        RCLCPP_WARN(get_logger(), "Completed avoidance path, returning to main path");
    }
    
    // 返回当前避障点
    if (following_avoidance.load()) {
        target = avoidance_path.poses[avoidance_idx];
        local_count++;
        RCLCPP_WARN(get_logger(), "Completed !!!");
        return target;
    }
 } 
}
    double distance_to_goal;
    if(!is_second.load()){
    distance_to_goal = std::hypot(
        path.poses.back().pose.position.x - (current_pose.pose.position.y+x_dff),
        path.poses.back().pose.position.y - (current_pose.pose.position.x+y_dff)
    );
    }else{
        distance_to_goal = std::hypot(
            path.poses.back().pose.position.x - (current_pose.pose.position.x+y_dff_ -x_),
            path.poses.back().pose.position.y - (current_pose.pose.position.y+x_dff_ -y_)
        );
    }
    
    // 动态调整前瞻距离：如果距离终点较远，用较大的前瞻距离；如果较近，用较小的值
    double max_lookahead;
    double min_lookahead;
    double lookahead_distance;
    if(!is_second.load()){
    lookahead_distance = std::min(0.25, distance_to_goal * 0.4);
    lookahead_distance = std::max(lookahead_distance, 0.1); // 最小前瞻距离
    }else{
        // 动态计算前瞻距离（整合 getAdaptiveLookahead）
    //min_lookahead = 0.3;
    //max_lookahead = 0.5;
    Point current_point = {current_pose.pose.position.x + y_dff_ - x_, 
    current_pose.pose.position.y + x_dff_ - y_};

    // 将 nav_msgs::msg::Path 转换为 std::vector<Point>
    std::vector<Point> path_points;
    for (const auto& pose : path.poses) {
    path_points.push_back({pose.pose.position.x, pose.pose.position.y});
    }

    // 调用动态前瞻函数
    lookahead_distance = getAdaptiveLookahead(current_point, path_points, yolo_obstacles,closest_idx);

    // 强制前瞻距离不小于最小值（防卡顿）
    lookahead_distance = std::max(lookahead_distance, 0.2); //0.4

    // 如果是 is_second 模式，确保前瞻距离足够大以快速推进
    //if (distance_to_goal > 1.0) {
    //lookahead_distance = std::min(lookahead_distance * 1.5, max_lookahead);
    //}
           
    }
    size_t target_idx = closest_idx;
    bool found = false;
    // 4. 特殊处理is_second模式：强制前进防卡顿
    /*if (is_second.load()) {
        size_t target_idx = std::max(closest_idx, max_reached_idx);
        // 如果选择的点和上次相同，强制前进
        if (target_idx == last_target_idx) {
            target_idx = std::min(target_idx + 1, path.poses.size() - 1);
            RCLCPP_WARN(get_logger(), "is_second: Force advancing to point %zu", target_idx);
        }
        // 更新最远点（is_second模式下允许快速推进）
        max_reached_idx = std::max(max_reached_idx, target_idx);
    }*/

    for (size_t i = closest_idx; i < path.poses.size(); ++i) {
        double dx;
        double dy;
        if(!is_second.load()){
             dx = path.poses[i].pose.position.x - (current_pose.pose.position.y+x_dff);
             dy = path.poses[i].pose.position.y - (current_pose.pose.position.x+y_dff);
            }else{
             dx = path.poses[i].pose.position.x - (current_pose.pose.position.x+y_dff_-x_);
             dy = path.poses[i].pose.position.y - (current_pose.pose.position.y+x_dff_-y_);
            }
        double distance = std::hypot(dx, dy);

        if (distance >= lookahead_distance) {
            target_idx = i;
            found = true;
            break;
    }
    
}

    // 如果已经接近终点，选择最后一个点
    if (!found) {
        // 选择路径上最远的可达点
        target_idx = std::min(closest_idx + 3, path.poses.size() - 1);
        
        // 特别处理接近终点的情况
        if (distance_to_goal < min_lookahead) {
            target_idx = path.poses.size() - 1;  // 直接选择终点
        }
    }

    /*if(is_second.load()){
        target_idx = std::max(target_idx, max_reached_idx);
        target_idx = std::min(target_idx, path.poses.size() - 1);
        last_target_idx = target_idx;
        if (target_idx > max_reached_idx) {
            max_reached_idx = target_idx;
        }
    }*/

    //1.6
    if((target_idx > path.poses.size() - 5 || distance_to_goal < 1.4) && is_second.load()){
        is_findt_ok.store(true);
        //target_yaw_count++;
        target_idx = path.poses.size() - 1;
        RCLCPP_WARN(get_logger(), "Completed ??");
    }

    /*if(target_idx == path.poses.size() - 1 && is_second.load()){
        if()
        is_find_path.load(true);
    }*/
    RCLCPP_WARN(
        get_logger(),
        "Selected target point [%zu/%zu]: (x=%.3f m, y=%.3f m),closest_idx:%zu,current_pose.pose.position.x:%.2f,current_pose.pose.position.y:%.2f",
        target_idx,
        path.poses.size() - 1,
        path.poses[target_idx].pose.position.x,
        path.poses[target_idx].pose.position.y,
        closest_idx,
        current_pose.pose.position.x+y_dff_-x_,
        current_pose.pose.position.y+x_dff_-y_
    );
    return path.poses[target_idx];
}

double SyncPlannerNode::calculateCurvature(const Point& p1, const Point& p2, const Point& p3) {
    double dx1 = p2.x - p1.x, dy1 = p2.y - p1.y;
    double dx2 = p3.x - p2.x, dy2 = p3.y - p2.y;
    
    // 计算叉积和点积
    double cross = dx1 * dy2 - dy1 * dx2;
    double dot = dx1 * dx2 + dy1 * dy2;
    double norm1 = std::hypot(dx1, dy1);
    double norm2 = std::hypot(dx2, dy2);
    
    // 避免除以零
    if (norm1 < 1e-6 || norm2 < 1e-6) return 0.0;
    
    // 曲率 = 交叉积 / (模长乘积)
    return std::abs(cross) / (norm1 * norm2);
}


double SyncPlannerNode::getNearestObstacleDistance(double current_x, double current_y, 
    const std::vector<std::pair<float, float>>& yolo_obstacles,
    double obstacle_radius) {
double min_distance = std::numeric_limits<double>::max(); // 初始化为最大值

for (const auto& obs : yolo_obstacles) {
double dx = obs.first - current_x;
double dy = obs.second - current_y;
double distance = std::sqrt(dx * dx + dy * dy) - obstacle_radius; // 减去障碍物半径

if (distance < min_distance) {
min_distance = distance;
}
}
return (min_distance > 0) ? min_distance : 0.0; // 避免负值（机器人已碰撞）
}

double SyncPlannerNode::getAdaptiveLookahead(const Point& current_pose, 
    const std::vector<Point>& path,
    const std::vector<std::pair<float, float>>& obstacles,int closest_idx) {
// 基础参数
double min_lookahead = 0.3;
double max_lookahead = 1.0;

// 1. 计算到终点的距离
double distance_to_goal = std::hypot(path.back().x - current_pose.x, 
                 path.back().y - current_pose.y);

// 2. 计算曲率（需确保 path.size() >= 3）
double curvature = 0.0;
if (path.size() >= 3 && closest_idx + 1 < path.size()) {
    size_t prev_idx = (closest_idx == 0) ? 0 : closest_idx - 1;
    size_t next_idx = closest_idx + 1;
    curvature = calculateCurvature(path[prev_idx], path[closest_idx], path[next_idx]);
}

double curvature_factor = 1.0 / (1.0 + 6.0 * std::abs(curvature));//5.0

// 3. 计算障碍物距离
double obstacle_distance = getNearestObstacleDistance(current_pose.x, current_pose.y, obstacles);
double obstacle_factor = std::min(1.0, obstacle_distance / 4.0); //2.0

// 4. 动态调整前馈
double adaptive_lookahead;
if (distance_to_goal > 2.0) {
adaptive_lookahead = 0.3 * curvature_factor * obstacle_factor; //0.6
} else if (distance_to_goal > 1.0) {
adaptive_lookahead = 0.2 * curvature_factor * obstacle_factor; //0.4
} else {
adaptive_lookahead = min_lookahead;
}

return std::clamp(adaptive_lookahead, min_lookahead, max_lookahead);
}


size_t SyncPlannerNode::findGeometricClosest(const geometry_msgs::msg::PoseStamped& current_pose, 
    const nav_msgs::msg::Path& path) {

// 1. 查找几何最近点
size_t closest_idx = 0;
double min_squared_dist = std::numeric_limits<double>::max();

for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto& pose = path.poses[i].pose.position;
    double dx;
    double dy;
    if(is_second.load()){
        dx = pose.x - (current_pose.pose.position.x + y_dff_ - x_);
        dy = pose.y - (current_pose.pose.position.y + x_dff_ - y_);
    }else{
        dx = pose.x -  (current_pose.pose.position.y + x_dff);
        dy = pose.y -  (current_pose.pose.position.x  +y_dff);
    }
    double squared_dist = dx*dx + dy*dy;
    
    if (squared_dist < min_squared_dist) {
        min_squared_dist = squared_dist;
        closest_idx = i;
    }
}

    // 禁止回退
    closest_idx = std::max(closest_idx, last_closest_idx);
    
    // 检查是否停滞
    if (closest_idx == last_closest_idx) {
        same_point_count++;
        
        // 连续2次选择同一点则强制前进
        if (same_point_count >= 2) {
            closest_idx = std::min(closest_idx + 1, path.poses.size() - 1);
            same_point_count = 0;  // 重置计数器
            RCLCPP_WARN(get_logger(), "Force advancing to point %zu", closest_idx);
        }
    } else {
        same_point_count = 0;  // 有进展时重置计数器
    }
    
    last_closest_idx = closest_idx;
    return closest_idx;
}

size_t SyncPlannerNode::find_safe_return_index(
    const nav_msgs::msg::Path& path, 
    size_t start_idx,
    const std::vector<Eigen::Vector3f>& obstacles,
    double safe_distance,
    int lookahead) 
{
    // 至少前瞻3个点，最多前瞻20个点或路径终点
    size_t max_lookahead = std::min(start_idx + lookahead, path.poses.size() - 1);
    
    for (size_t i = start_idx; i <= max_lookahead; ++i) {
        bool is_safe = true;
        
        // 检查该点周围安全区域
        for (const auto& obs : obstacles) {
            double dist = std::hypot(
                path.poses[i].pose.position.x - obs.x(),
                path.poses[i].pose.position.y - obs.y());
                
            if (dist < safe_distance) {
                is_safe = false;
                break;
            }
        }
        
        if (is_safe) return i; // 找到第一个安全点
    }
    
    return max_lookahead; // 默认返回最大前瞻点
}

// 修改check_point_near_obstacle函数，考虑速度因素
bool SyncPlannerNode::check_point_near_obstacle(
    const geometry_msgs::msg::PoseStamped& point,
    const std::vector<Eigen::Vector3f>& obstacles)
{
    // 动态障碍物阈值 = 基础阈值 + 速度因素
    double dynamic_threshold = is_second.load() ? 30 : 20 + current_speed_ * 35;
    dynamic_threshold = std::min(dynamic_threshold, 60.0); // 最大阈值
    
        for (const auto& obs : obstacles) {
            double dx = point.pose.position.x*100 - obs.x();
            double dy = point.pose.position.y*100 - obs.y();
            double distance = std::hypot(dx, dy);
            //RCLCPP_WARN(get_logger(), "Obstacle at (%.1f, %.1f) cm, distance: %.1f cm, threshold: %.1f cm", 
              //  obs.x(), obs.y(), distance, dynamic_threshold);
            
            if (distance < dynamic_threshold && !is_second.load()) {
                return true;
            }
            if ((obs.x() > 160 && obs.y() >200) && is_second.load()) {
                return true;
            }
        }
        return false;
}

nav_msgs::msg::Path SyncPlannerNode::generate_avoidance_path(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const nav_msgs::msg::Path& original_path,
    size_t closest_idx,
    const std::vector<Eigen::Vector3f>& obstacles)
{
    nav_msgs::msg::Path avoidance_path;
    avoidance_path.header = original_path.header;
    
    // 1. 计算路径的平均方向
    /*Eigen::Vector2d path_direction(0, 0);
    int lookahead = std::min(5, static_cast<int>(original_path.poses.size() - closest_idx - 1));
    for (int i = 0; i < lookahead; ++i) {
        path_direction.x() += original_path.poses[closest_idx + i].pose.position.x - 
                             original_path.poses[closest_idx].pose.position.x;
        path_direction.y() += original_path.poses[closest_idx + i].pose.position.y - 
                             original_path.poses[closest_idx].pose.position.y;
    }
    path_direction.normalize();*/
    
    // 2. 分析障碍物分布
    const double ROBOT_WIDTH = 0.5;
    const double INFLATION_RADIUS = ROBOT_WIDTH * 0.5;
    bool is_up = false;
    
    double left_score = 0.0;
    double right_score = 0.0;
    double min_obstacle_distance = std::numeric_limits<double>::max();
    geometry_msgs::msg::PoseStamped avoid_point;
    
    for (const auto& obs : obstacles) {
        Eigen::Vector2d obs_pos(obs.x(), obs.y());
        Eigen::Vector2d path_pos;
        if(!is_second.load()){
            path_pos.x() = current_pose.pose.position.y+x_dff;
            path_pos.y() = current_pose.pose.position.x+y_dff;
        }else{
            path_pos.x() = current_pose.pose.position.x+y_dff - x_;
            path_pos.y() = current_pose.pose.position.y+x_dff - y_;
        }
        
        double dist = (obs_pos - path_pos).norm();
        //if (dist > 1.5) continue;
        
        min_obstacle_distance = std::min(min_obstacle_distance, dist);
        
        Eigen::Vector2d relative_pos = obs_pos;
        //double cross_product = obs_pos.x();
        
        //double influence = 1.0 / (dist * dist);
        if(!is_second.load()){
        if ((obs_pos.x() < 35 && obs_pos.y() > 90) || (obs_pos.y() < 90 && obs_pos.x() < 25)) {
            right_score += 1;
        } else {
            left_score +=  1;
        }
    }else{
        if(obs_pos.y() < 290){
            is_up = true;  
        }else{
            is_up = false;
        }
    }
        avoid_point.pose.position.x = obs.x();
        avoid_point.pose.position.y = obs.y();
    }
    
    // 3. 确定避让方向和距离
    //bool avoid_right = left_score < right_score;
    double avoid_distance = 0.35;
   // avoid_distance = std::max(avoid_distance, 0.6);
   // avoid_distance = std::min(avoid_distance, 0.3);//1.5
    //avoid_distance = avoid_distance;
    RCLCPP_WARN(this->get_logger(), "avoid_distance:%.2f",avoid_distance);
    
    // 4. 计算避让路径的三个关键点：起始点、避让点和回归点
    std::vector<geometry_msgs::msg::PoseStamped> avoidance_points;
    
    // 起始点（当前最近路径点）
    //if(!is_second.load()) avoidance_points.push_back(original_path.poses[closest_idx]);
    
    // 避让点（偏移点）
    //Eigen::Vector2d avoid_direction(-path_direction.y(), path_direction.x());
    int avoid_direction = 1;
    //if (avoid_right) avoid_direction = -avoid_direction;
    
    //geometry_msgs::msg::PoseStamped avoid_point = original_path.poses[closest_idx];
    //float k_ff = current_pose.pose.position.y+x_dff > 1.5 ? 1.3 : 2.0;
    if(!is_second.load()){
        avoid_point.pose.position.x =  right_score > left_score ?avoid_point.pose.position.x + 40: 0.1; 
        avoid_point.pose.position.y += 50;
    }
    if(is_second.load()) {
        avoid_point.pose.position.y = is_up ? avoid_point.pose.position.y + 50 
        :avoid_point.pose.position.y - 50;
        avoid_point.pose.position.x = is_up ? avoid_point.pose.position.x - 20 : 190;
    }
    avoid_point.pose.position.x = std::clamp(avoid_point.pose.position.x, 0.0, 200.0);
    avoid_point.pose.position.y = std::clamp(avoid_point.pose.position.y, 0.0, 500.0);

    RCLCPP_INFO(get_logger(), 
    "Avoid Point - Position: (%.2f, %.2f)",avoid_point.pose.position.x,avoid_point.pose.position.y);
    avoidance_points.push_back(avoid_point);
    
    // 回归点（前方3-5个点，回到原路径）
   
    //size_t return_idx = std::min(closest_idx + 5, original_path.poses.size() - 1)
    int looked = is_second.load() ? 5 : 5;
    size_t return_idx = find_safe_return_index(original_path, closest_idx, obstacles, 0.4, looked);
    return_idx = !is_second.load() ? std::max(return_idx, closest_idx + 2) : std::max(return_idx, closest_idx + 4); // 确保至少前进2个点
    return_idx = std::min(closest_idx + 5, original_path.poses.size() - 1);
    avoidance_points.push_back(original_path.poses[return_idx]);
    
    avoidance_path.poses = avoidance_points;

    return avoidance_path;
}

void SyncPlannerNode::update_current_speed(double speed) {
    current_speed_ = speed;
    last_speed_update_time_ = std::chrono::steady_clock::now();
}

void SyncPlannerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    last_odom_ = *msg;
}

bool SyncPlannerNode::get_current_pose(geometry_msgs::msg::PoseStamped& pose) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    
    // 检查odom数据是否有效
    if (last_odom_.header.stamp.sec == 0) {
        RCLCPP_WARN(this->get_logger(), "No valid odometry data received yet!");
        return false;
    }

    // 填充PoseStamped
    pose.header = last_odom_.header;
    pose.pose = last_odom_.pose.pose;
    return true;
}

double SyncPlannerNode::heuristic(const std::pair<int, int>& a, const std::pair<int, int>& b,int D) {
    int dx = abs(a.first - b.first);
    int dy = abs(a.second - b.second);
    int D2 = (int)(2*D- sqrt(D*D));
    return D * (dx + dy) -  D2* std::min(dx, dy);  // D=10, D2=14（近似对角线）
} 

double SyncPlannerNode::get_cost(const std::pair<int,int>& p) {
    // 基础通行代价
    double cost = 1.0; 
    
    // 增加障碍物附近惩罚
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            int ny = p.second + dy, nx = p.first + dx;
            if (ny >=0 && nx >=0 && ny < map_height && nx < map_width) {
                cost += 0.5 * occupancy_grid_[ny][nx]; // 障碍物越近惩罚越大
            }
        }
    }
    return cost;
}

std::vector<std::pair<int, int>> SyncPlannerNode::bresenham_line(
    std::pair<int, int> start, 
    std::pair<int, int> end, 
    int step_size // 默认步长为1（即原始算法）
) {
    // 1. 先用标准Bresenham算法生成完整直线
    std::vector<std::pair<int, int>> full_line;
    int x1 = start.first, y1 = start.second;
    int x2 = end.first, y2 = end.second;
    
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
        full_line.push_back({x1, y1});
        if (x1 == x2 && y1 == y2) break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }

    // 2. 按步长采样点
    std::vector<std::pair<int, int>> sampled_line;
    for (size_t i = 0; i < full_line.size(); i += step_size) {
        sampled_line.push_back(full_line[i]);
    }
    
    // 3. 确保终点被包含（如果未被采样）
    if (!full_line.empty() && !sampled_line.empty() && 
        sampled_line.back() != full_line.back()) {
        sampled_line.push_back(full_line.back());
    }

    return sampled_line;
}

/*void SyncPlannerNode::sync_callback_(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr& yolo_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) 
{
    std::lock_guard<std::mutex> lock(depth_mutex_);
    
    // 1. 深度图像预处理
    cv_bridge::CvImagePtr cv_depth;
    try {
        // 1. 转换为32位浮点型深度图
        /*cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO16);
        
        if (cv_depth->image.empty()) {
            RCLCPP_ERROR(get_logger(), "Empty depth image received!");
            return;
        }
        
        // 2. 预处理深度图
        cv::Mat depth_float;
        cv_depth->image.convertTo(depth_float, CV_32F, 0.001); // 转换为米为单位的浮点型
        
        // 应用高斯滤波降噪
        cv::Mat blurred_depth;
        cv::GaussianBlur(depth_float, blurred_depth, 
                         cv::Size(gaussian_kernel_size_, gaussian_kernel_size_), 
                         gaussian_sigma_);
        
        // 3. 创建有效深度掩码 (去除太远或太近的无效点)
        float  min_obstacle_distance_ = 0.1f;
        float max_obstacle_distance_ = 5.0f;
        cv::Mat valid_depth_mask = (blurred_depth > min_obstacle_distance_) & 
                                  (blurred_depth < max_obstacle_distance_);
        
        // 4. 转换为8位图像用于轮廓检测
        cv::Mat depth_8u;
        cv::normalize(blurred_depth, depth_8u, 0, 255, cv::NORM_MINMAX, CV_8U);
        
        // 5. 二值化处理
        cv::Mat binary;
        int depth_threshold_ = 50;
        cv::threshold(depth_8u, binary, depth_threshold_, 255, cv::THRESH_BINARY);
        binary &= valid_depth_mask; // 应用有效深度掩码
        
        // 6. 形态学操作去除噪声
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
        
        // 7. 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // 8. 处理检测到的轮廓
        std::map<int, Eigen::Vector3f> current_frame_obstacles;
        float min_contour_area_ = 25000.0; 
        
        for (const auto& contour : contours) {
            // 过滤小面积轮廓
            double area = cv::contourArea(contour);
            if (area < min_contour_area_) {
                continue;
            }
            
            // 获取轮廓的边界矩形
            cv::Rect boundRect = cv::boundingRect(contour);
            
            // 收集ROI内的有效深度值
            std::vector<float> depths;
            cv::Mat roi_depth = blurred_depth(boundRect);

            float depth_world_x;
            float depth_world_y;
            float depth_origin_x;
            //std::pair<float,float> detected_cones = detectConesWithClustering(boundRect,blurred_depth);
            std::vector<Eigen::Vector3f> detected_cones = detectConesWithClustering(boundRect, blurred_depth);
            std::vector<std::pair<float, float>> depth_position;  // 保存所有锥筒的世界坐标(x,y)
            for (const auto& cone : detected_cones) {
                geometry_msgs::msg::PoseStamped current_pose;
                if (!get_current_pose(current_pose)) {
                    return;
                }
                if (!is_second.load()) {
                    depth_world_x = cone.x() * 100 + (current_pose.pose.position.y + x_dff) * 100;
                    depth_world_y = cone.z() * 100 + (current_pose.pose.position.x + y_dff) * 100;
                    depth_origin_x = cone.x() * 100;
                    
                    // 可以在这里处理每个锥筒的坐标，或者存储到某个容器中
                    RCLCPP_INFO(get_logger(), 
                        "Processing cone at (%.2f, %.2f, %.2f)", 
                        cone.x(), cone.y(), cone.z());
                } else {
                    depth_world_x = 200.0 - cone.x() * 100; 
                    depth_world_y = 250.0 - cone.z() * 100;
                }
                depth_position.emplace_back(depth_world_x, depth_world_y);
            }
                
        }

        std::map<int, Eigen::Vector3f> current_frame_obstacles;
        for (const auto& target : yolo_msg->targets) {
            if (target.type == "hat") {
                for (const auto& roi : target.rois) {
                    // 跳过低置信度检测
                    if (roi.confidence < confidence_threshold_) {
                        continue;
                    }
                    
                    // 计算ROI中心点
                    int center_x = roi.rect.x_offset + roi.rect.width / 2;
                    int center_y = roi.rect.y_offset + roi.rect.height ;
                        
                        // 转换到3D坐标
                        geometry_msgs::msg::PoseStamped current_pose;
                        if (!get_current_pose(current_pose)) {
                            return;
                        }
                        auto local = inverse_perspective_point(center_x, center_y);
                        double theta = tf2::getYaw(current_pose.pose.orientation);

                        std::pair<double,double> camera_goal_d;
                        if(!is_second.load()){
                        camera_goal_d = local_to_global((double)(current_pose.pose.position.y+x_dff) ,
                        (double)(current_pose.pose.position.x+y_dff),
                        theta,(double)local.first,
                        (double)local.second,(!is_second.load()?0:1));
                        }else{
                        camera_goal_d = local_to_global((double)(current_pose.pose.position.y) ,
                        (double)(current_pose.pose.position.x+y_dff),
                        theta,(double)local.first,
                        (double)local.second,(!is_second.load()?0:1));
                        }
                        camera_goal_path_.push_back(camera_goal_d);
                    }
                }
            }
    // 存储融合后的锥筒数据
    std::vector<std::pair<float, float>> fused_cones;

    // 1. 获取深度摄像头检测到的锥筒位置 (假设只有一个)
    Eigen::Vector3f depth_cone = depth_position; // depth_world_x, depth_world_y, depth_origin_x

    // 2. 遍历单目摄像头检测到的所有锥筒
    for (const auto& camera_cone : camera_goal_path_) {
        float camera_x = camera_cone.first;
        float camera_y = camera_cone.second;
        fused_cones.emplace_back(camera_x, camera_y);
    // 3. 检查是否存在深度摄像头检测到的锥筒
        //if (depth_cone[0] != 0 && depth_cone[1] != 0) {
        // 计算y坐标差值
            //float y_diff = fabs(camera_y - depth_cone[1]);
        
        // 4. 如果y坐标相差小于20cm(0.2m)，认为是同一个锥筒
            /*if (y_diff < 20.0f) { // 假设单位为cm
            // 使用单目摄像头的x坐标和深度摄像头的y坐标
            float depth_weight = (depth_cone[1] < 3.0) ? 0.7 : 0.3;
            float camera_weight = 1.0 - depth_weight; // 确保权重总和=1
            float fused_y = depth_weight * depth_cone[1] + camera_weight * camera_y;

            fused_cones.emplace_back(camera_x, fused_y);
            
            // 标记深度锥筒已匹配
                depth_cone[0] = 0;
                depth_cone[1] = 0;
                continue;
            }
        }
    
    // 5. 如果没有匹配的深度锥筒，直接使用单目摄像头的数据
    fused_cones.emplace_back(camera_x, camera_y);
    }

    {
        for (const auto& cone : fused_cones) {
        float obstacle_x = cone.first;
        float obstacle_y = cone.second;
        float obstacle_origin_x = cone.second;

                // 生成障碍物唯一ID (基于位置哈希)
        int obstacle_id = std::hash<float>{}(obstacle_x) ^ 
                                (std::hash<float>{}(obstacle_y) << 1);
                
                // 存储当前帧障碍物
        current_frame_obstacles[obstacle_id] = 
                    Eigen::Vector3f(obstacle_x, obstacle_y, obstacle_origin_x);
                
        //RCLCPP_WARN(get_logger(), 
          //          "Detected obstacle at (%.2f, %.2f, %.2f) with depth %.2fm",
            //       obstacle_x, obstacle_y, obstacle_origin_x, obstacle_origin_x);
        }
    }
    
    // 9. 更新全局障碍物历史
    {
        std::lock_guard<std::mutex> obs_lock(obstacle_mutex_);
        const float alpha = 0.3f; // 指数移动平均系数
        
        // 更新现有障碍物或添加新障碍物
        for (auto& [id, obs] : current_frame_obstacles) {
            obstacle_history_[id] = obs;  // 直接更新为最新观测值
        }
        
        // 移除未在当前帧中检测到的障碍物
        /*for (auto it = obstacle_history_.begin(); it != obstacle_history_.end(); ) {
            if (current_frame_obstacles.find(it->first) == current_frame_obstacles.end()) {
                it = obstacle_history_.erase(it);
            } else {
                ++it;
            }
        }
        if (obstacle_history_.size() > 5) {
            // 计算需要删除的数量
            size_t items_to_remove = obstacle_history_.size() - 5;
            auto it = obstacle_history_.begin();
            // 移动迭代器到要删除的位置
            std::advance(it, items_to_remove);
            // 删除最旧的记录
            obstacle_history_.erase(obstacle_history_.begin(), it);
        }
        
    }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Depth processing error: %s", e.what());
        return;
    }
}*/

void SyncPlannerNode::depth_image_callback_(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) 
{
    std::lock_guard<std::mutex> lock(depth_mutex_);
    
    try {
        // 1. 转换为32位浮点型深度图
        cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO16);
        
        if (cv_depth->image.empty()) {
            RCLCPP_ERROR(get_logger(), "Empty depth image received!");
            return;
        }
        
        // 2. 预处理深度图
        cv::Mat depth_float;
        cv_depth->image.convertTo(depth_float, CV_32F, 0.001); // 转换为米为单位的浮点型
        
        // 应用高斯滤波降噪
        cv::Mat blurred_depth;
        cv::GaussianBlur(depth_float, blurred_depth, 
                         cv::Size(gaussian_kernel_size_, gaussian_kernel_size_), 
                         gaussian_sigma_);
        
        // 3. 创建有效深度掩码 (去除太远或太近的无效点)
        float  min_obstacle_distance_ = 0.1f;
        float max_obstacle_distance_ = 5.0f;
        cv::Mat valid_depth_mask = (blurred_depth > min_obstacle_distance_) & 
                                  (blurred_depth < max_obstacle_distance_);
        
        // 4. 转换为8位图像用于轮廓检测
        cv::Mat depth_8u;
        cv::normalize(blurred_depth, depth_8u, 0, 255, cv::NORM_MINMAX, CV_8U);
        
        // 5. 二值化处理
        cv::Mat binary;
        int depth_threshold_ = 50;
        cv::threshold(depth_8u, binary, depth_threshold_, 255, cv::THRESH_BINARY);
        binary &= valid_depth_mask; // 应用有效深度掩码
        
        // 6. 形态学操作去除噪声
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
        
        // 7. 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // 8. 处理检测到的轮廓
        std::map<int, Eigen::Vector3f> current_frame_obstacles;
        float min_contour_area_ = 25000.0; 
        
        for (const auto& contour : contours) {
            // 过滤小面积轮廓
            double area = cv::contourArea(contour);
            if (area < min_contour_area_) {
                continue;
            }
            
            // 获取轮廓的边界矩形
            cv::Rect boundRect = cv::boundingRect(contour);
            
            // 计算ROI中心点
            int center_x = boundRect.x + boundRect.width / 2;
            int center_y = boundRect.y + boundRect.height;
            
            // 收集ROI内的有效深度值
            std::vector<float> depths;
            cv::Mat roi_depth = blurred_depth(boundRect);

            // 1. 创建调试图像（直接使用深度图伪彩色）
            /*cv::Mat debug_img;
            cv::normalize(blurred_depth, debug_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::cvtColor(debug_img, debug_img, cv::COLOR_GRAY2BGR); // 转彩色

            // 2. 绘制当前轮廓和ROI框
            cv::drawContours(debug_img, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0,0,255), 1);
            cv::rectangle(debug_img, boundRect, cv::Scalar(0,255,0), 1);
            // 3. 标记中心点
            cv::circle(debug_img, cv::Point(center_x, center_y), 3, cv::Scalar(255,0,0), -1);
            cv::imwrite("debug_contour.png", debug_img);*/
            
            /*for (int y = boundRect.y; y < boundRect.y + boundRect.height; ++y) {
                for (int x = boundRect.x; x < boundRect.x + boundRect.width; ++x) {
                    float depth = blurred_depth.at<float>(y, x);
                    //loat depth_m = depth*0.001f;
                    if (depth > min_obstacle_distance_ && depth < max_obstacle_distance_) {
                        depths.push_back(depth);
                    }
                }
            }*/
            
            float depth_world_x;
            float depth_world_y;
            float depth_origin_x;
            std::vector<Eigen::Vector3f> detected_cones = detectConesWithClustering(boundRect,blurred_depth);

                geometry_msgs::msg::PoseStamped current_pose;
                if (!get_current_pose(current_pose)) {
                    return;
                }

                for (const auto& cone : detected_cones) {
                    float cone_x = -cone.x();  // 相机坐标系X（右）
                    float cone_z = cone.z();  // 相机坐标系Z（前）
                
                    if (!is_second.load()) {
                        // 第一视角的坐标转换
                        depth_world_x = cone_x * 100 + (current_pose.pose.position.y + x_dff) * 100;
                        depth_world_y = cone_z * 100 + (current_pose.pose.position.x + y_dff) * 100 + 10;
                        depth_origin_x = cone_x * 100;
                        depth_ = cone_z;
                        if(depth_<depth_max && current_pose.pose.position.x + y_dff < 0.6){
                            is_first_slow.store(true);
                            RCLCPP_WARN(get_logger(), "1111111");
                        }else if(current_pose.pose.position.y + x_dff < 0.6){
                            depth_first.x = depth_world_x;
                            depth_first.y = depth_world_y;
                            is_depth_first.store(true);
                        }

                        RCLCPP_WARN(get_logger(), 
                            "Cone at (%.2f, %.2f, %.2f) -> World (%.2f, %.2f)", 
                            cone.x(), cone.y(), cone.z(), depth_world_x, depth_world_y);
                    } else {
                        // 第二视角的坐标转换
                        depth_world_x = (current_pose.pose.position.x + y_dff_ - x_)*100 - cone_z * 100; 
                        depth_world_y = (current_pose.pose.position.y + x_dff_ - y_ )*100- cone_x * 100;
                        depth_origin_x = cone.x()*100;

                        depth_second_ = cone_z;

                        if(depth_second_ < depth_max_second && current_pose.pose.position.x + y_dff - x_ > 1.9){
                            depth_second.x = depth_world_x;
                            depth_second.y = depth_world_y;
                            is_depth_second.store(true);
                        }

                        RCLCPP_WARN(get_logger(), 
                            "Cone at (%.2f, %.2f, %.2f,%.2f) -> World (%.2f, %.2f)", 
                            cone.x(), cone.y(), cone.z(), depth_ ,depth_world_x, depth_world_y);
                    }
                }
                
                // 将像素坐标转换为3D坐标
                float obstacle_x = depth_world_x;
                float obstacle_y = depth_world_y;
                float obstacle_origin_x = depth_origin_x;

                // 生成障碍物唯一ID (基于位置哈希)
                int obstacle_id = std::hash<float>{}(obstacle_x) ^ 
                                (std::hash<float>{}(obstacle_y) << 1);
                
                // 存储当前帧障碍物
                current_frame_obstacles[obstacle_id] = 
                    Eigen::Vector3f(obstacle_x, obstacle_y, obstacle_origin_x);
                
               // RCLCPP_WARN(get_logger(), 
                 //   "Detected obstacle at (%.2f, %.2f, %.2f) with depth %.2fm",
                   // obstacle_x, obstacle_y, obstacle_origin_x, obstacle_origin_x);
            }
        
        
        // 9. 更新全局障碍物历史
        {
            std::lock_guard<std::mutex> obs_lock(obstacle_mutex_);
            const float alpha = 0.3f; // 指数移动平均系数
            
            // 更新现有障碍物或添加新障碍物
            for (auto& [id, obs] : current_frame_obstacles) {
                obstacle_history_[id] = obs;  // 直接更新为最新观测值
            }
            
            // 移除未在当前帧中检测到的障碍物
            /*for (auto it = obstacle_history_.begin(); it != obstacle_history_.end(); ) {
                if (current_frame_obstacles.find(it->first) == current_frame_obstacles.end()) {
                    it = obstacle_history_.erase(it);
                } else {
                    ++it;
                }
            }*/
            if (obstacle_history_.size() > 1) {
                // 计算需要删除的数量
                size_t items_to_remove = obstacle_history_.size() - 1;
                auto it = obstacle_history_.begin();
                // 移动迭代器到要删除的位置
                std::advance(it, items_to_remove);
                // 删除最旧的记录
                obstacle_history_.erase(obstacle_history_.begin(), it);
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Depth processing error: %s", e.what());
        return;
    }
}

void SyncPlannerNode::convertDepthROIToPointCloud(const cv::Rect& roi, 
    const cv::Mat& depth_image,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) 
{
// 1. 配置深度相机内参（需根据实际标定值修改）
const float fx = 365.0, fy = 365.0; // 焦距（像素）
const float cx = 160.0, cy = 100.0; // 光心（320x200分辨率）
const float depth_scale = 1.0;       // 深度值缩放因子（通常1.0表示米）

// 2. 遍历ROI区域生成点云
cloud->clear();
cloud->width = roi.width;
cloud->height = roi.height;
cloud->is_dense = false;
cloud->points.resize(roi.area());

#pragma omp parallel for
for (int y = roi.y; y < roi.y + roi.height; ++y) {
for (int x = roi.x; x < roi.x + roi.width; ++x) {
// 获取深度值（单位：米）
float depth = depth_image.at<float>(y, x) * depth_scale;

// 跳过无效深度
if (depth <= 0.1f || depth > 10.0f) continue;

// 计算3D坐标
pcl::PointXYZ point;
point.z = depth;
point.x = (x - cx) * point.z / fx;
point.y = (y - cy) * point.z / fy;

cloud->at(x - roi.x, y - roi.y) = point;
}
}
}


std::vector<Eigen::Vector3f> SyncPlannerNode::detectConesWithClustering(
    const cv::Rect& depth_roi,          // 直接传入ROI区域
    const cv::Mat& depth_image) 
{
    std::vector<Eigen::Vector3f> cones_3d;

    // 5. 转换ROI区域为点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int y = depth_roi.y; y < depth_roi.y + depth_roi.height; ++y) {
        for (int x = depth_roi.x; x < depth_roi.x + depth_roi.width; ++x) {
            uint16_t depth_mm = depth_image.at<uint16_t>(y, x);
            if (depth_mm > 100 && depth_mm < max_obstacle_distance_ * 1000) {
                // 转换为相机坐标系（X右，Y下，Z前）
                float z = depth_mm * 0.001f; // mm -> m
                //float x_cam = (x - (depth_camera_cx_ - depth_roi.x)) * z / depth_camera_fx_;
                //float y_cam = (y - (depth_camera_cy_ - depth_roi.y)) * z / depth_camera_fy_;
                float x_cam = (x - (depth_camera_cx_)) * z / depth_camera_fx_;
                float y_cam = (y - (depth_camera_cy_)) * z / depth_camera_fy_;
                if (std::isfinite(x_cam) && std::isfinite(y_cam)) {
                    cloud->points.emplace_back(x_cam, y_cam, z);
                }
                cloud->points.emplace_back(x_cam, y_cam, z);
            }
        }
    }

    if (cloud->empty()) {
        RCLCPP_INFO(get_logger(), "No valid points after conversion.");
        return {};
    }

    // 6. 执行欧式聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.1); // 5cm间距阈值
    ec.setMinClusterSize(30);     // 最小点数
    ec.setMaxClusterSize(2000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // 7. 处理聚类结果
    if (!cluster_indices.empty()) {
        // 找到最大簇（锥筒主体）
        auto largest_cluster = std::max_element(
            cluster_indices.begin(), cluster_indices.end(),
            [](const auto& a, const auto& b) { 
                return a.indices.size() < b.indices.size(); 
            });

        // 计算簇的3D质心
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, *largest_cluster, centroid);

        // 存储结果
        cones_3d.emplace_back(centroid[0], centroid[1], centroid[2]);

        // 调试输出
       // RCLCPP_INFO(get_logger(), 
       //     "Cone centroid: (%.2f, %.2f, %.2f) with %zu points",
         //   centroid[0], centroid[1], centroid[2],
           // largest_cluster->indices.size());
    }else{
       //RCLCPP_INFO(get_logger(), "No valid points !!!!!!.");
    }

    return cones_3d; // 返回相机坐标系下的3D位置
}

// 输入：depth_image - 深度图（单位：mm）
//      roi - 锥桶所在的ROI区域
//      fx, cx - 相机内参（焦距和光心x坐标）
std::pair<float, float> SyncPlannerNode::detectConePosition(const cv::Mat& depth_image, const cv::Rect& roi) {
    // 1. 提取ROI内有效深度
    std::vector<uint16_t> valid_depths;
    double fx = depth_camera_fx_;
    double cx = depth_camera_cx_;
    float sum_x = 0, sum_d = 0;
    for (int y = roi.y; y < roi.y + roi.height; ++y) {
        for (int x = roi.x; x < roi.x + roi.width; ++x) {
            uint16_t d = depth_image.at<uint16_t>(y, x);
            if (d > 100 && d < 1000) {  // 过滤无效值（0.3m~2m）
                sum_x += x * d;
                sum_d += d;
                valid_depths.push_back(d);
            }
        }
    }

    if (valid_depths.empty()) {
       // RCLCPP_INFO(get_logger(), "No valid points !!!!!!.");
        return {NAN, NAN};  // 无有效数据
    }

    // 2. 计算中值深度（抗噪声更好）
    std::nth_element(valid_depths.begin(), valid_depths.begin() + valid_depths.size()/2, valid_depths.end());
    float median_depth_mm = valid_depths[valid_depths.size()/2];
    float z = median_depth_mm * 0.001f;  // 转为米

    // 3. 计算ROI中心与图像中心的横向偏移（像素）
    //float roi_center_x = roi.x + roi.width / 2.0f;
    float roi_center_x = sum_x / sum_d;
    float pixel_offset = roi_center_x - cx;  // >0表示锥桶在中心右侧

    // 4. 转换为真实世界的横向偏移（米）
    float x_offset = pixel_offset * z / fx;  // X = (u - cx) * z / fx
    //RCLCPP_WARN(get_logger(), "x_offset:%.2f,z:%.2f",-x_offset,z);

    return {-x_offset,z};  // 返回距离和横向偏移
}

void SyncPlannerNode::subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg) {
    std::lock_guard<std::mutex> lock(point_target_mutex_);

    bool has_hat = false;
    SyncPlannerNode::ConeInfo current_max;  // 当前消息中的最大锥桶
    for (const auto& target : targets_msg->targets) {
        if (target.type == "hat") {
            has_hat = true;
            for (const auto& roi : target.rois) {
                // 计算当前锥桶面积（假设target.roi中有width和height）
                double current_area = roi.rect.width * roi.rect.height;
                bool is_center = (roi.rect.x_offset + roi.rect.width / 2.0) > 250 
                && (roi.rect.x_offset + roi.rect.width / 2.0) < 400;
            // 更新当前消息中的最大锥桶
                if (current_area > current_max.area) {
                    current_max.area = current_area;
                // 假设target.pose.position是底部中心点
                    current_max.bottom_center = roi.rect.x_offset + roi.rect.width / 2.0;
                    current_max.is_center = is_center;
                }
        }
            //break; // 找到第一个就退出
        }
    }
    
    // 2. 如果有hat目标，将整个消息推入队列一次
    if (has_hat) {
        std::lock_guard<std::mutex> cone_lock(max_cone_mutex_);
            if (current_max.area > max_cone_info_.area) {
                max_cone_info_ = current_max;
                RCLCPP_INFO(this->get_logger(), 
                    "New max cone detected! Area: %.2f",
                    max_cone_info_.area);
            }
        targets_queue_.push(targets_msg);
        
        // 3. 队列大小管理
        const size_t MAX_QUEUE_SIZE = is_second.load() ? 1 : 1; // 总是保持1个最新消息
        while (targets_queue_.size() > MAX_QUEUE_SIZE) {
            targets_queue_.pop();
        }
    }

    if(is_hat.load()){
        hat_subscriber_.reset();
        is_hat.store(false);
    }
}  


SyncPlannerNode::ConeInfo SyncPlannerNode::getMaxConeInfo() {
    std::lock_guard<std::mutex> lock(max_cone_mutex_);
    return max_cone_info_;
}

void SyncPlannerNode::subscription_callback_target_second(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg) {
    std::lock_guard<std::mutex> lock(point_target_mutex_second);
    /*for (const auto& target : targets_msg->targets) {
        if (target.type == "hat") {
            targets_queue_.push(targets_msg);
        }
        RCLCPP_WARN(this->get_logger(), "Current queue size: %zu", targets_queue_.size());
    }

    // 根据消息频率设置
    const size_t MAX_QUEUE_SIZE = is_second.load() ? 1 : 1;
    while (targets_queue_.size() > MAX_QUEUE_SIZE) {
      targets_queue_.pop();
    } */
    bool has_hat = false;
    for (const auto& target : targets_msg->targets) {
        if (target.type == "hat") {
            has_hat = true;
            break; // 找到第一个就退出
        }
    }
    
    // 2. 如果有hat目标，将整个消息推入队列一次
    if (has_hat) {
        targets_queue_second.push(targets_msg);
        
        // 3. 队列大小管理
        const size_t MAX_QUEUE_SIZE = is_second.load() ? 5 : 5; // 总是保持1个最新消息
        while (targets_queue_second.size() > MAX_QUEUE_SIZE) {
            targets_queue_second.pop();
        }
        /*RCLCPP_WARN(this->get_logger(), "Current queue size: %zu", targets_queue_.size());
        size_t total_targets = targets_msg->targets.size();
        RCLCPP_WARN(this->get_logger(), "Received targets_msg with %zu targets", total_targets);*/
    }
}  

void SyncPlannerNode::subscription_callback_target_qr(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg) {
    std::lock_guard<std::mutex> lock(point_qr_mutex_);
    for (const auto& target : targets_msg->targets) {
        if (target.type == "qr") {
            qr_queue_.push(targets_msg);
          for (const auto& roi : target.rois) {
            if (roi.confidence < confidence_threshold_) {
              RCLCPP_WARN(get_logger(), "Low confidence(%.2f < %.2f)", 
                          roi.confidence, confidence_threshold_);
              continue;
            }
            target_x = roi.rect.x_offset + roi.rect.width / 2.0;
             // 计算QR码大小（宽度×高度）
            qr_size = roi.rect.width * roi.rect.height;
                
             // 保存到数组
            qr_sizes_.push_back(qr_size);
          }
        }
      }

    while (qr_queue_.size() > 1) {
      qr_queue_.pop();
    } 
    if (!qr_sizes_.empty()) {
        qr_sizes_.erase(qr_sizes_.begin(), qr_sizes_.end() - 1); // 只保留最后一个元素
    }
}  

void SyncPlannerNode::handle_service(
    const std::shared_ptr<origincar_msg::srv::UpdatePlanner::Request> request,
    std::shared_ptr<origincar_msg::srv::UpdatePlanner::Response> response)
{
    if (request->detections.targets.empty()) {
        RCLCPP_ERROR(get_logger(), "Service request has empty detections.targets");
        if (!targets_queue_.empty() && targets_queue_.top()) {
            std::lock_guard<std::mutex> lock(point_target_mutex_);
            request->detections = *targets_queue_.top();
            RCLCPP_WARN(get_logger(), "Recovered detections from queue");
        } else {
            response->success = true;
            response->message = "No valid detections available";
            std::lock_guard<std::mutex> lock(grid_mutex_);
            occupancy_grid_ = std::vector<std::vector<float>>(500, std::vector<float>(200, 0.0f));
            pose_initialized_.store(true);
            RCLCPP_ERROR(get_logger(), "Invalid service request");
            return;
        }
    }

    // 1. 处理YOLO检测的障碍物

    for (const auto& detection : request->detections.targets) {
        if (detection.type == "hat") {
            for (const auto& roi : detection.rois) {
                if (roi.confidence < confidence_threshold_) {
                    RCLCPP_WARN(get_logger(), "Low confidence(%.2f < %.2f)", 
                                roi.confidence, confidence_threshold_);
                    continue;
                }
                
                if (roi.rect.x_offset > 10000 || roi.rect.y_offset > 10000 || 
                    roi.rect.width > 10000 || roi.rect.height > 10000) {
                    RCLCPP_ERROR(get_logger(), "Abnormal ROI values: x=%u y=%u w=%u h=%u",
                        roi.rect.x_offset, roi.rect.y_offset,
                        roi.rect.width, roi.rect.height);
                    continue;
                }
                
                geometry_msgs::msg::PoseStamped current_pose;
                if (!get_current_pose(current_pose)) {
                    return;
                }
                double theta = tf2::getYaw(current_pose.pose.orientation);
    
                int img_x = static_cast<int>(roi.rect.x_offset + roi.rect.width / 2.0);
                int img_y = static_cast<int>(roi.rect.y_offset + roi.rect.height);
           
                auto local = inverse_perspective_point(img_x, img_y);
        
                std::pair<double,double> goal_d;
                if(!is_second.load()) {
                    goal_d = local_to_global((double)(current_pose.pose.position.y+x_dff),
                                   (double)(current_pose.pose.position.x+y_dff),
                                   theta,(double)local.first,
                                   (double)local.second,0);
                    if(goal_d.first >1 && goal_d.second >10) {
                        first_line_obstacles.emplace_back(goal_d.first, goal_d.second);
                    }
                } else {
                    goal_d = local_to_global((double)(current_pose.pose.position.x),
                                   (double)(current_pose.pose.position.y+x_dff_ - y_),
                                   theta,(double)local.first,
                                   (double)local.second,1);
                    if(goal_d.first >1 && goal_d.second >10) {
                        second_line_obstacles.emplace_back(goal_d.first, goal_d.second);
                    }
                }
            }
        }
    }

    // 2. 修正第二条路径的障碍物（基于第一条路径的结果）
    /*if (!first_line_obstacles.empty() && !second_line_obstacles.empty() && is_second.load()) {
        std::vector<std::pair<float, float>> corrected_second_line;
        
        for (const auto& second_obs : second_line_obstacles) {
            bool need_correction = false;
            
            // 检查与第一条路径障碍物的距离
            for (const auto& first_obs : first_line_obstacles) {
                float dx = second_obs.first - first_obs.first;
                float dy = second_obs.second - first_obs.second;
                float distance = sqrt(dx*dx + dy*dy);
                
                if (distance < 80.0f && distance > 30) {  // 小于90cm需要修正
                    need_correction = true;
                    
                    // 计算最小移动向量
                    float dx = second_obs.first - first_obs.first;
                    float dy = second_obs.second - first_obs.second;
                    float current_dist = sqrt(dx*dx + dy*dy);
                    
                    // 计算需要移动的距离（90cm + 10cm缓冲）
                    float required_dist = 100.0f - current_dist;
                    
                    // 标准化方向向量
                    float norm_x = dx / current_dist;
                    float norm_y = dy / current_dist;
                    
                    // 计算修正后的位置（沿远离方向移动）
                    std::pair<float, float> corrected_obs = {
                        second_obs.first + norm_x * required_dist,
                        second_obs.second + norm_y * required_dist
                    };
                    
                    // 边界检查
                    corrected_obs.first = std::clamp(corrected_obs.first, 0.0f, static_cast<float>(map_width-1));
                    corrected_obs.second = std::clamp(corrected_obs.second, 0.0f, static_cast<float>(map_height-1));
                    
                    // 检查修正后的位置是否有效
                    bool valid_position = true;
                    for (const auto& other_obs : first_line_obstacles) {
                        float new_dx = corrected_obs.first - other_obs.first;
                        float new_dy = corrected_obs.second - other_obs.second;
                        float new_dist = sqrt(new_dx*new_dx + new_dy*new_dy);
                        
                        if (new_dist < 80.0f) {
                            valid_position = false;
                            break;
                        }
                    }
                    
                    if (valid_position) {
                        corrected_second_line.push_back(corrected_obs);
                        RCLCPP_WARN(get_logger(), "Obstacle corrected: (%.1f,%.1f) -> (%.1f,%.1f) (distance: %.1fcm)",
                                   second_obs.first, second_obs.second,
                                   corrected_obs.first, corrected_obs.second,
                                   required_dist + current_dist);
                    } else {
                        // 如果直接远离无效，尝试垂直方向移动
                        std::pair<float, float> alt_correction1 = {
                            second_obs.first - norm_y * required_dist,
                            second_obs.second + norm_x * required_dist
                        };
                        
                        std::pair<float, float> alt_correction2 = {
                            second_obs.first + norm_y * required_dist,
                            second_obs.second - norm_x * required_dist
                        };
                        
                        // 选择第一个有效的位置
                        if (is_position_valid(alt_correction1, first_line_obstacles)) {
                            corrected_second_line.push_back(alt_correction1);
                        } else if (is_position_valid(alt_correction2, first_line_obstacles)) {
                            corrected_second_line.push_back(alt_correction2);
                        } else {
                            // 如果还是无效，直接丢弃该障碍物
                            corrected_second_line.push_back(second_obs);
                            RCLCPP_WARN(get_logger(), "Cannot find valid correction for obstacle at (%.1f,%.1f)", 
                                        second_obs.first, second_obs.second);
                        }
                    }
                    
                    break;
                }
            }
        }
        
        second_line_obstacles = corrected_second_line;
    }*/

    // 3. 合并障碍物
    yolo_obstacles.insert(yolo_obstacles.end(), first_line_obstacles.begin(), first_line_obstacles.end());
    yolo_obstacles.insert(yolo_obstacles.end(), second_line_obstacles.begin(), second_line_obstacles.end());

    // 将第一条线障碍物加入新容器（标记为false）
    for (const auto& obs : first_line_obstacles) {
        tagged_yolo_obstacles.push_back({obs.first, obs.second, false});
    }

    // 将第二条线障碍物加入新容器（标记为true）
    for (const auto& obs : second_line_obstacles) {
        tagged_yolo_obstacles.push_back({obs.first, obs.second, true});
    }


    // 4. 更新占用网格
    {
        std::lock_guard<std::mutex> grid_lock(grid_mutex_);
        occupancy_grid_ = std::vector<std::vector<float>>(500, std::vector<float>(200, 0.0f));
        obs_count = 0;
        
        for (const auto& obs : tagged_yolo_obstacles) {
            int gx = static_cast<int>(obs.x);
            int gy = static_cast<int>(obs.y);
            bool is_left = true;
    
            //int cost_x = is_second.load() ? 30 : 25;
            //int cost_y = is_second.load() ? 30 : 25;
            if(is_depth_first.load() && !is_second.load()){
                if( gx < 50 && gy < 140){
                    //gx = static_cast<int>(depth_first.x);
                    //gy = static_cast<int>(depth_first.y);
                    RCLCPP_WARN(get_logger(), "gx:""""");
                    is_depth_first.store(false);
                    continue;
                } 
            }
            // 根据来源选择半径
            int cost_x = obs.is_from_second_line ? 35 : 25; // 第二条线30，第一条线25
            int cost_y = obs.is_from_second_line ? 35 : 25;
            
            if(!obs.is_from_second_line){
                if(gx >= 25 && gy <=150) is_left = false;

            if(gx>200) gx = 199;
            if(gy<160) cost_x = 30;

            if(gy>250){
                cost_x = 35; 
                cost_y = 35;
            }

            if(gx < 40){
                cost_x = 30; 
                cost_y = 30;
            }
            }

            const int start = is_left ? std::max(0, gx - cost_x) : 20;
            const int end = std::min(199, std::max(45, abs(gx) + cost_x));
            
            gx = std::clamp(gx, 0, 199);
            gy = std::clamp(gy, 0, 499);
            
            for (int i = std::max(0, gy-cost_y); i <= std::min(499, gy+cost_y); ++i) {
                for (int j = start; j <= end ; ++j) {
                    occupancy_grid_[i][j] = 1.0f;
                }
            }
            
            obs_count++;
        }
        
        // 检查是否通过最后一个锥桶
        geometry_msgs::msg::PoseStamped current_pose;
        if (get_current_pose(current_pose)) {
            isPastLastCone(current_pose.pose, occupancy_grid_);
        }
    }
    
    // 5. 准备响应
    response->success = true;
    response->message = "Occupancy grid updated with " + 
                       std::to_string(obs_count) + " obstacles (" +
                       std::to_string(first_line_obstacles.size()) + " first line, " +
                       std::to_string(second_line_obstacles.size()) + " second line)";    
    pose_initialized_.store(true);
    
    RCLCPP_WARN(get_logger(), 
               "Occupancy grid updated. Total obstacles: %d (First line: %zu, Second line: %zu)",
               obs_count, first_line_obstacles.size(), second_line_obstacles.size());
}

bool SyncPlannerNode::is_position_valid(const std::pair<float, float>& pos, 
    const std::vector<std::pair<float, float>>& existing_obstacles,
    float min_distance) {
// 边界检查
if (pos.first < 0 || pos.first >= map_width || 
pos.second < 0 || pos.second >= map_height) {
return false;
}

// 与现有障碍物的距离检查
for (const auto& obs : existing_obstacles) {
float dx = pos.first - obs.first;
float dy = pos.second - obs.second;
float distance = sqrt(dx*dx + dy*dy);

if (distance < min_distance) {
return false;
}
}

return true;
}

double SpeedController::calculateSpeed(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const nav_msgs::msg::Path& path,
    const std::vector<std::vector<float>>& occupancy_grid,
    bool is_second_path,
    double now_loc,
    double angle_error,
    bool is_local,
    bool is_qr_ok,
    int obs_count,
    bool is_first_slow,
    bool is_depth_second
) {
    // 1. 基础速度配置
    double base_speed = configureBaseSpeed(is_second_path,now_loc,is_local,obs_count,is_first_slow,is_depth_second);
    
    // 2. 动态调整因素
    curvatureFactor_ = curvatureFactor(current_pose,path,is_second_path);
    base_speed *= curvatureFactor_;

    trackErrorFactor_ = trackErrorFactor(current_pose, path,is_second_path);
    base_speed *= trackErrorFactor_;

    obstacleProximityFactor_ = obstacleProximityFactor(current_pose, occupancy_grid,is_second_path,obs_count);
    base_speed *= obstacleProximityFactor_;
    
    // 4. 确保安全约束
    return applyConstraints(base_speed,is_second_path,current_pose,angle_error,is_local,is_qr_ok,is_depth_second);
}

double SpeedController::curvatureFactor(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const nav_msgs::msg::Path& path,
    bool is_second_path
) {
    const int lookahead_points = 5;
    double max_curvature = 0.0;
    
    if (path.poses.empty()) {
        return 1.0;
    }
    
    // 1. 首先找到路径上距离当前位姿最近的点
    size_t closest_idx = 0;
    double min_distance = 100;
    
    for (size_t i = 0; i < path.poses.size(); ++i) {
        double dx;
        double dy;
        if(!is_second_path){
            dx = path.poses[i].pose.position.x - (current_pose.pose.position.y + x_dff);
            dy = path.poses[i].pose.position.y - (current_pose.pose.position.x + y_dff);
        }else{
            dx = path.poses[i].pose.position.x - (current_pose.pose.position.x + x_dff_);
            dy = path.poses[i].pose.position.y - (current_pose.pose.position.y + y_dff_);
        }
        double distance = std::hypot(dx, dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = i;
        }
    }
    
    // 2. 从最近点开始，检查前方几个路径点计算最大曲率
    for (size_t i = closest_idx; 
         i < std::min(closest_idx + lookahead_points, path.poses.size() - 1); 
         ++i) {
        double dx1 = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
        double dy1 = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
        
        if (i > 0) {
            double dx0 = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
            double dy0 = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
            
            // 计算曲率 (dθ / ds)
            double angle_diff = std::abs(std::atan2(dy1, dx1) - std::atan2(dy0, dx0));
            double distance = std::hypot(dx1, dy1);
            double curvature = angle_diff / std::max(0.01, distance);
            
            if (curvature > max_curvature) {
                max_curvature = curvature;
            }
        }
    }
    
    // 曲率越大，减速越多
    const double MAX_CURVATURE = 2.0; // 经验值
    double curvature_factor = 1.0 / (0.8 + max_curvature);
    
    //RCLCPP_WARN(logger_, "Curvature factor: %.2f (max curv: %.2f)", curvature_factor, max_curvature);
    return curvature_factor;
}

double SpeedController::trackErrorFactor(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const nav_msgs::msg::Path& path,
    bool is_second_path
) {
    double min_distance = std::numeric_limits<double>::max();
    
    // 寻找路径上最近点
    for (const auto& pose : path.poses) {
        double dx;
        double dy;  
        if(!is_second_path){
            dx = pose.pose.position.x - (current_pose.pose.position.y + x_dff);
            dy = pose.pose.position.y - (current_pose.pose.position.x + y_dff);  
        }else{
            dx = pose.pose.position.x - (current_pose.pose.position.x + x_dff_);
            dy = pose.pose.position.y - (current_pose.pose.position.y + y_dff_);  
        }
        double distance = std::hypot(dx, dy);
        
        if (distance < min_distance) {
            min_distance = distance;
        }

    }
    
    // 跟踪误差越大，减速越多
    const double MAX_TRACK_ERROR = 0.2; 
    double error_factor = 1.0 - std::min(1.0, min_distance / MAX_TRACK_ERROR);
    
    //RCLCPP_WARN(logger_, "Track error factor: %.2f (error: %.2fm)", error_factor, min_distance);
    return 1.0 - (0.1 * error_factor); //0.1
}

double SpeedController::applyConstraints(double target_speed, bool is_second ,
    const geometry_msgs::msg::PoseStamped& current_pose,double angle_error,bool is_local,bool is_qr_ok
    ,bool is_depth_second) {
    // 限速约束
    double MAX_SPEED = 1.2;   // 最大速度
    double MIN_SPEED = 0.6;   // 最小速度
    
    // 加速/减速限制 (最大加速度0.5m/s²)
    static double last_speed = 0.0;
    static auto last_time = std::chrono::steady_clock::now();
    
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_time).count();
    last_time = now;
    
    double max_change = 0.5 * dt; // 0.5m/s²加速度限制
    
    if (target_speed > last_speed + max_change) {
        target_speed = last_speed + max_change;
    } else if (target_speed < last_speed - max_change) {
        target_speed = last_speed - max_change;
    }
    
    last_speed = target_speed;

    if(!is_second){
        // 定义减速区域阈值
        const double slow_down_threshold_x = 2.0;
        const double slow_down_threshold_y = 4.3;  //4.6 //5.0
        
        // 计算当前位置到减速边界的距离
        double distance_to_threshold = std::min(
            slow_down_threshold_x - (current_pose.pose.position.y + x_dff),
            slow_down_threshold_y - (current_pose.pose.position.x + y_dff)
        );
        
        // 定义减速区域范围
        const double slow_down_range = 1.0; // 在距离阈值 开始减速 //1.2
        const double cut_down_threshold_y = 4.0;//4.4 4.0
        const double cut_down_threshold_x = 2.0;
        //bool is_cut = ((current_pose.pose.position.x + y_dff > cut_down_threshold_y) 
        //|| (current_pose.pose.position.y + x_dff > cut_down_threshold_x));
        //sconst double local_tansition = 0.7;
        
        // 计算速度比例因子 (0.0-1.0)
        double speed_factor = 1.0;
        if (distance_to_threshold < slow_down_range) {
            // 当接近阈值时，线性减小速度
            double normalized_distance = distance_to_threshold / slow_down_range;
            speed_factor = (is_qr_ok || (current_pose.pose.position.x + y_dff >4.1 && current_pose.pose.position.y + x_dff >1.0)) ?
             0.0 : 0.1 + 0.9 * normalized_distance * normalized_distance;

            // 应用速度因子
            target_speed = (0.8 + (fabs(angle_error)/M_PI/2))*speed_factor ; //0.6 0.85 0.8
            //if(current_pose.pose.position.x + y_dff > 3.8) target_speed *= 0.7; //0.75 0.5
            MIN_SPEED =  (is_qr_ok || (current_pose.pose.position.x + y_dff > 4.1 && current_pose.pose.position.y + x_dff >1.0)) ? 0.0 : 0.5;//0.2
            MAX_SPEED = 0.7;
        }
        
        }

        if(is_second){
        const double slow_down_threshold_y_depth_first = 2.7;
        const double slow_down_threshold_y_depth_second = 2.0;
        const double slow_down_range_depth = 0.7;
        double distance_to_threshold_depth = current_pose.pose.position.y + y_dff_ 
        - slow_down_threshold_y_depth_second ;

        if(is_depth_second && distance_to_threshold_depth > 0.4){
            target_speed = ((1-distance_to_threshold_depth/slow_down_range_depth) + 0.6)*target_speed;
        }    

        // 定义减速区域阈值
        const double slow_down_threshold_x = 0.0;
        const double slow_down_threshold_y = 0.0;//0.8  //0.5
        
        // 计算当前位置到减速边界的距离
        double distance_to_threshold = std::min(
            (current_pose.pose.position.x + x_dff_) - slow_down_threshold_x,
            (current_pose.pose.position.y + y_dff_) - slow_down_threshold_y
        );
        
        // 定义减速区域范围
        const double slow_down_range = 1.0; // 在距离阈值 开始减速 //1.5 0.5
        //const double cut_down_threshold_y = 0.1;//4.4
        //const double local_tansition = 0.7;
        
        // 计算速度比例因子 (0.0-1.0)
        double speed_factor = 1.0;
        if (distance_to_threshold < slow_down_range) {
            // 当接近阈值时，线性减小速度
            double normalized_distance = distance_to_threshold / slow_down_range;
            speed_factor =  0.1 + 0.9 * normalized_distance * normalized_distance;

            // 应用速度因子
            target_speed =(target_speed + (fabs(angle_error)/M_PI/2))*speed_factor*2.0; //0.4 0.5 1.2 1.8 //2.0
            //if(is_local) target_speed *= 0.8; //0.75
            MIN_SPEED = 0.4;//0.2
            MAX_SPEED = 0.6;
        }
        }
    
    // 确保在速度范围内
    return std::clamp(target_speed, MIN_SPEED, MAX_SPEED);
}

double SpeedController::configureBaseSpeed(bool is_second_path,double now_loc,bool is_local,
    int obs_count,bool is_first_slow, bool is_depth_second) {
    // 第二圈行驶策略不同
    if (is_second_path) {
        return 0.58; //0.6 0.8
    }
    
    // 第一圈速度策略
    const double SAFE_ZONE_SPEED = is_first_slow ? 0.5 : 0.8; //0.7
    const double NORMAL_SPEED = obs_count > 3 ? 1.35 : 1.55; //1.0
    
    // 根据赛道位置决定基础速度
    if (now_loc < 1.5 && !is_local) {
        return SAFE_ZONE_SPEED;
    } else if(is_local){
        return SAFE_ZONE_SPEED;
    }
    else {
        return NORMAL_SPEED;
    }
}

double SpeedController::obstacleProximityFactor(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const std::vector<std::vector<float>>& occupancy_grid,
    bool is_second_path,
    int obs_count
) {
    const int OBSTACLE_CHECK_RANGE = 1.0; 
    
    // 检查前方障碍物距离
    const double NO_OBSTACLE_FACTOR = 1.0; // 无障碍物时的默认因子
    double min_obstacle_dist = OBSTACLE_CHECK_RANGE; // 初始设为检测范围
    double obstacle_angle = 0.0;
    double current_yaw = tf2::getYaw(current_pose.pose.orientation);
    int count = 0;

    // 前方扇区扫描
    for (double angle = -M_PI/3; angle <= M_PI/3; angle += M_PI/18) {
        for (double dist = 0.1; dist <= 0.8; dist += 0.1) {
            double dx = dist * std::cos(current_yaw + angle);
            double dy = dist * std::sin(current_yaw + angle);
            
            // 转换为网格坐标并确保在边界内
            int grid_x;
            int grid_y;
            if(!is_second_path){
                grid_x = static_cast<int>((current_pose.pose.position.y + x_dff + dx) * 100);
                grid_y = static_cast<int>((current_pose.pose.position.x + y_dff + dy) * 100);
            }else{
                grid_x = static_cast<int>((current_pose.pose.position.x + x_dff_ + dx) * 100);
                grid_y = static_cast<int>((current_pose.pose.position.y + y_dff_ + dy) * 100);
            }
            
            grid_x = std::clamp(grid_x, 0, 199);
            grid_y = std::clamp(grid_y, 0, 499);
        
                if (occupancy_grid[grid_y][grid_x] != 0.0f) {
                    if (dist < min_obstacle_dist) {
                        min_obstacle_dist = dist;
                        obstacle_angle = angle;
                    }
                    count ++;
                }
            }
    }
    
    if (count == 0) {
       // RCLCPP_WARN(logger_, "No obstacles detected, using default factor");
        return NO_OBSTACLE_FACTOR;
    }

    // 安全距离因子
    const double SAFE_DISTANCE = 0.5; 
    double proximity_factor = std::min(1.0, min_obstacle_dist / SAFE_DISTANCE);
    double K_ ;
    //= is_second_path ? 0.65 : 0.75; //0.83
    if(!is_second_path){
        K_ = obs_count > 3 ? 0.6 : 0.8;
    }else{
        K_ = 0.5;
    }

    // 如果障碍物在行驶路径正前方，进一步减速
    if (std::abs(obstacle_angle) < M_PI/3) {
        proximity_factor *= K_; 
    }
    //RCLCPP_WARN(logger_, "Obstacle proximity factor: %.2f (min dist: %.2fm),count: %d", proximity_factor, 
      //  min_obstacle_dist,count);
    return proximity_factor;
}

double SpeedController:: configureBaseKff(const geometry_msgs::msg::PoseStamped& current_pose,
    const std::vector<std::vector<float>>& occupancy_grid,
    double base_gain, double min_gain, double max_gain,bool is_second,bool is_local){
    k_base_ = is_second ? base_gain: base_gain;
    k_min_ = min_gain;
    k_max_ = max_gain;
    position_threshold_ = is_second ? 1.5 : 4.0;
     
    return  computeGain(current_pose,occupancy_grid,is_second);
}

double SpeedController::computeGain(const geometry_msgs::msg::PoseStamped& current_pose,
    const std::vector<std::vector<float>>& occupancy_grid,bool is_second) {
    // 计算当前位置的进度
    const double progress = is_second ? current_pose.pose.position.y + y_dff_
    :current_pose.pose.position.x + y_dff;
    
    // 根据当前位置计算自适应增益
    double gain = k_base_;
    double trackFactor_ = is_second ? 0.85 : 0.3;
    
    // 如果处在起始区域（靠近起点），使用较高的增益增强响应能力
    if (progress < position_threshold_ ) {
        gain = k_max_ - (progress / position_threshold_) * (k_max_ - k_base_);
        
        // 如果附近有障碍物，进一步增加增益以快速避障
        if (obstacleProximityFactor_ < 0.7) {
            gain = std::min(gain * 1.5, k_max_);
            //RCLCPP_WARN(logger_, "Obstacle detected - boosting gain to %.2f", gain);
        }

        if (trackErrorFactor_ > trackFactor_) {
            gain += 0.5;
            //RCLCPP_WARN(logger_, "High curvature - increasing gain to %.2f", gain);
        }
    }
    // 在赛道后半段，使用中等增益
    else {
        gain = k_base_;
        
        if (obstacleProximityFactor_ < 0.7) {
            gain = std::min(gain * 1.1, k_max_);
            //RCLCPP_WARN(logger_, "Obstacle detected - boosting gain to %.2f", gain);
        }

        if (trackErrorFactor_ > trackFactor_) {
            gain += 0.3;
            //RCLCPP_WARN(logger_, "High curvature - increasing gain to %.2f", gain);
        }

    }
    
    // 确保在安全范围内
    gain = std::clamp(gain, k_min_, k_max_);
    
    //RCLCPP_WARN(logger_, "Feedforward gain: %.2f at progress %.2f", gain, progress);
    return gain;
}

bool SyncPlannerNode::isPastLastCone(
    const geometry_msgs::msg::Pose& current_pose,
    const std::vector<std::vector<float>>& occupancy_grid,
    float cone_threshold)         // 锥桶判断阈值（大于此值视为锥桶）
{

    // 2. 找到所有锥桶位置（值 > cone_threshold 视为锥桶）
    std::vector<std::pair<int, int>> cone_positions;
    const int height = occupancy_grid.size();
    const int width = occupancy_grid[0].size();

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (occupancy_grid[y][x] > cone_threshold) {
                cone_positions.emplace_back(x, y);
            }
        }
    }

    // 3. 如果没有锥桶，直接返回true
    if (cone_positions.empty()) {
        return true;
    }

    // 4. 找到最后一个锥桶（假设路径方向是X轴正方向）
    auto last_cone_it = std::max_element(
        cone_positions.begin(),
        cone_positions.end(),
        [](const auto& a, const auto& b) {
            return a.second < b.second; // 按y坐标比较
        });
    
    const int last_cone_x = last_cone_it->first;
    const int last_cone_y = last_cone_it->second;

    if(is_local_.load()){
        hat_x = last_cone_it->first;
        hat_y = last_cone_it->second - 15;
    }

    //RCLCPP_WARN(get_logger(), "last_x: %d, last_y: %d", last_cone_x, last_cone_y);

    // 5. 将当前位姿转换到网格坐标系
    const float world_x = (current_pose.position.y + x_dff)*100;
    const float world_y = (current_pose.position.x + y_dff)*100;

    //if(last_cone_y > 4.2 && (obs_count > 5 || is_local_.load())) is_qr_ok.store(true);
    
    // 6. 判断是否超过最后一个锥桶（X坐标更大）
    return world_y > last_cone_y && last_cone_y < 450;
}

bool SyncPlannerNode::isPastLastCone_first(
    const geometry_msgs::msg::Pose& current_pose,
    const std::vector<std::vector<float>>& occupancy_grid,
    float cone_threshold) {
        // 2. 找到所有锥桶位置（值 > cone_threshold 视为锥桶）
    std::vector<std::pair<int, int>> cone_positions;
    const int height = occupancy_grid.size();
    const int width = occupancy_grid[0].size();

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (occupancy_grid[y][x] > cone_threshold) {
                cone_positions.emplace_back(x, y);
            }
        }
    }

    if (cone_positions.empty()) {
        return true;
    }

    auto first_cone_it = std::min_element(
        cone_positions.begin(),
        cone_positions.end(),
        [](const auto& a, const auto& b) {
            return a.second < b.second;
        });
    
    auto first_cone_it_ = std::min_element(
            cone_positions.begin(),
            cone_positions.end(),
            [](const auto& a, const auto& b) {
                return a.first < b.first;
            });
    const int first_cone_y = first_cone_it->second + 30;
    const int first_cone_x = first_cone_it_->first + 25;

    const float world_y = (current_pose.position.x + y_dff)*100;
    return (world_y > first_cone_y) && first_cone_y < 170 && first_cone_x > 30;
}

std::pair<int,int> SyncPlannerNode::find_PastLastCone(
    const std::vector<std::vector<float>>& occupancy_grid)         // 锥桶判断阈值（大于此值视为锥桶）
{

    // 2. 找到所有锥桶位置（值 > cone_threshold 视为锥桶）
    std::vector<std::pair<int, int>> cone_positions;
    const int height = occupancy_grid.size();
    const int width = occupancy_grid[0].size();

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (occupancy_grid[y][x] > 0.0) {
                cone_positions.emplace_back(x, y);
            }
        }
    }

    // 4. 找到最后一个锥桶（假设路径方向是X轴正方向）
    auto last_cone_it = std::max_element(
        cone_positions.begin(),
        cone_positions.end(),
        [](const auto& a, const auto& b) {
            return a.second < b.second; // 按y坐标比较
        });
    
    const int last_cone_x = last_cone_it->first;
    const int last_cone_y = last_cone_it->second;

    return {last_cone_x,last_cone_y};
}

// Dijkstra 算法计算所有点到终点的最短距离
std::vector<std::vector<double>> SyncPlannerNode:: dijkstra_heuristic(
    const std::pair<int, int>& goal,
    const std::vector<std::vector<float>>& Map) {
    
    const int map_height = Map.size();
    const int map_width = Map[0].size();
    const double INF = std::numeric_limits<double>::max();

    // 初始化启发式网格（所有点初始为无穷大）
    std::vector<std::vector<double>> heuristic_grid(
        map_height, std::vector<double>(map_width, INF));

    // 优先队列（小顶堆）
    std::priority_queue<SyncPlannerNode::DijkstraNode, std::vector<SyncPlannerNode::DijkstraNode>, std::greater<SyncPlannerNode::DijkstraNode>> pq;

    // 终点加入队列（代价为0）
    heuristic_grid[goal.second][goal.first] = 0.0;
    pq.push({goal.first, goal.second, 0.0});

    // 方向数组（8邻域或4邻域）
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1}; // 8邻域
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

    while (!pq.empty()) {
        DijkstraNode current = pq.top();
        pq.pop();

        // 如果当前节点的代价已经过时，跳过
        if (current.cost > heuristic_grid[current.y][current.x]) {
            continue;
        }

        // 遍历邻域
        for (int i = 0; i < 8; i++) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            // 检查边界和障碍物
            if (nx >= 0 && nx < map_width && ny >= 0 && ny < map_height &&
                Map[ny][nx] == 0.0f) { // 假设0.0f是可通行区域
                
                // 计算新代价（欧式距离或曼哈顿距离）
                double step_cost = (i < 4) ? 1.0 : 1.414; // 4邻域=1.0，斜向=1.414
                double new_cost = current.cost + step_cost;

                // 如果找到更短路径，更新并加入队列
                if (new_cost < heuristic_grid[ny][nx]) {
                    heuristic_grid[ny][nx] = new_cost;
                    pq.push({nx, ny, new_cost});
                }
            }
        }
    }

    return heuristic_grid;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 创建节点实例
    auto node = std::make_shared<SyncPlannerNode>();
    
    // 配置多线程执行器（2个线程，FIFO调度，优先级90）
    /*rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),
        2,          // 线程数
        SCHED_FIFO, // 实时调度策略
        90          // 线程优先级
    );*/

    rclcpp::executors::MultiThreadedExecutor executor;
    
    // 添加节点到执行器
    executor.add_node(node);
    
    // 启动执行器
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}

