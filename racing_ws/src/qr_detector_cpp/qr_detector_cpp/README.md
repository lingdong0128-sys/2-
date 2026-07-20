x5小车先下载一下zbar：

sudo apt install libzbar-dev


1.CMakeLists.txt中:
-----
# 添加自定义消息包路径
list(APPEND CMAKE_PREFIX_PATH 
    "${CMAKE_INSTALL_PREFIX}/../origin_car"
    "/root/ros2_ws/install/origin_car"
)
-----

-----
# 查找 ZBar
find_path(ZBAR_INCLUDE_DIRS NAMES zbar.h
          PATHS /usr/include /usr/local/include)
find_library(ZBAR_LIBRARIES NAMES zbar
             PATHS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu)

if(NOT ZBAR_INCLUDE_DIRS OR NOT ZBAR_LIBRARIES)
    message(FATAL_ERROR "ZBar not found. Please install libzbar-dev")
endif()
-----

# 以上两处地方需要修改一下路径,在终端中输入以下命令：

# 在 src 目录中查找
find ~/ros2_ws/src -name origincar_msg

# 在 install 目录中查找
find ~/ros2_ws/install -name origincar_msg

# 在整个工作空间中查找
find ~/ros2_ws -name origincar_msg

# 查找头文件位置
find /usr -name zbar.h 2>/dev/null

# 查找库文件位置
find /usr -name libzbar* 2>/dev/null

# 原来的CMakeLists.txt交给AI重新编写即可

# 第一次编译可能会有点慢，但是之后编译只需1，2秒

# 运行命令行：
ros2 run qr_detector_cpp qr_detector_cpp --ros-args \
  -p scale_factor:=0.7 \
  -p process_interval:=0.05 \
  -p qos_depth:=15

# scale_factor 越小，越不占内存，但检测效果下降； process_interval 越小检测速度越快，不过CPU占用高；qos_depth 越大，一定时间容纳的图片越多，不过占用内存；