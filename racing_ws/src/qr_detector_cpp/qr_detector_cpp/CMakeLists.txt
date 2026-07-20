cmake_minimum_required(VERSION 3.5)
project(qr_detector_cpp)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加自定义消息包路径
list(APPEND CMAKE_PREFIX_PATH 
    "/userdata/dev_ws/install/origincar_msg"
)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(origincar_msg REQUIRED)

# 手动设置 ZBar 路径 (基于您提供的路径)
set(ZBAR_INCLUDE_DIRS "/usr/include")
set(ZBAR_LIBRARIES "/usr/lib/aarch64-linux-gnu/libzbar.so")

# 创建可执行文件
add_executable(${PROJECT_NAME} src/qr_detector.cpp)

# 添加包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIRS}
  ${origincar_msg_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(${PROJECT_NAME} 
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBS}
  ${ZBAR_LIBRARIES}
  ${origincar_msg_LIBRARIES}
)

# 添加依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  origincar_msg
)

# 安装
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# 导出所有必要的依赖
ament_package()