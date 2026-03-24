# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.8)

# 定义包名
project(universal_controller)

# 配置C++编译选项（启用C++17，添加警告）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准为17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包（核心依赖+可选依赖）
find_package(ament_cmake REQUIRED)          # ROS2核心构建工具
find_package(rclcpp REQUIRED)               # ROS2 C++客户端库
find_package(sensor_msgs REQUIRED)          # 传感器消息类型
find_package(rcl_interfaces REQUIRED)       # ROS2接口定义
find_package(geometry_msgs REQUIRED)        # 几何消息类型
find_package(std_msgs REQUIRED)             # 标准消息类型
find_package(ament_copyright REQUIRED)      # 版权信息工具
find_package(ament_lint_auto REQUIRED)      # 自动代码检查工具
find_package(ament_lint_common REQUIRED)    # 常见代码检查工具
find_package(ament_cmake_gtest REQUIRED)    # CMake GTest 工具
find_package(ament_cmake_pytest REQUIRED)   # CMake PyTest 工具


# 包含头文件目录
include_directories(
  include  
)

# 编译节点
add_executable(

)

ament_target_dependencies(
  universal_controller_node
  rclcpp
  sensor_msgs
  rcl_interfaces
  geometry_msgs
  std_msgs
)


# 安装config目录下的yaml配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  PATTERN "*.yaml"  # 只安装yaml配置文件
)


ament_package()