# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.10)

# 定义包名
project(universal_controller)

# 配置C++编译选项（启用C++17，添加警告）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准为17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========== 查找依赖包 ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(sp_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ========== 消息/Action 生成 ==========
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UnifiedInput.msg"
  "action/GimbalControl.action"
  "action/FireControl.action"
  DEPENDENCIES std_msgs geometry_msgs
)

# ========== 包含目录 ==========
include_directories(include)

# ========== 源文件列表 ==========
set(HUB_SOURCES
  src/main.cpp
  src/hub/hub_core.cpp
  src/hub/hub_subscribers.cpp
  src/hub/hub_arbitration.cpp
  src/hub/hub_actions.cpp
  src/hub/rc_hub.cpp
)

set(VTM_INTERPRETER_SOURCES
  src/interpreters/vtm_interpreter.cpp
)

set(NDJ_INTERPRETER_SOURCES
  src/interpreters/ndj_interpreter.cpp
)

set(CONTROLLER_SOURCES
  src/controllers/chassis_controller.cpp
  src/controllers/gimbal_controller.cpp
  src/controllers/fire_controller.cpp
)

set(TOOL_SOURCES
  src/tools/swerve_kinematics.cpp
  src/tools/rc_yaml_parser.cpp
)

# ========== Main Hub Node ==========
add_executable(universal_controller_node
  ${HUB_SOURCES}
  ${VTM_INTERPRETER_SOURCES}
  ${NDJ_INTERPRETER_SOURCES}
  ${CONTROLLER_SOURCES}
  ${TOOL_SOURCES}
)

ament_target_dependencies(universal_controller_node
  rclcpp
  rclcpp_action
  ament_index_cpp
  sensor_msgs
  geometry_msgs
  std_msgs
  custom_msgs
  sp_msgs
)

rosidl_get_typesupport_target(universal_controller_node_typesupport
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(universal_controller_node
  ${universal_controller_node_typesupport}
  yaml-cpp
)

# ========== 安装 ==========
install(TARGETS
  universal_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.py"
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  PATTERN "*.yaml"
)

# ========== 测试 ==========
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
