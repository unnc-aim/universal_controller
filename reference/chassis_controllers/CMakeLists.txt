cmake_minimum_required(VERSION 3.8)
project(chassis_controllers)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/ChassisControl.msg"

        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(dji_full_wheel_chassis_controller src/dji_full_wheel_chassis_controller.cpp src/utils/pid.cpp src/utils/power_limiter.cpp)
add_executable(lk_full_wheel_chassis_controller src/lk_full_wheel_chassis_controller.cpp src/utils/pid.cpp src/utils/power_limiter.cpp)
ament_target_dependencies(dji_full_wheel_chassis_controller rclcpp std_msgs custom_msgs)
ament_target_dependencies(lk_full_wheel_chassis_controller rclcpp std_msgs custom_msgs)

rosidl_get_typesupport_target(
        cpp_typesupport_target
        ${PROJECT_NAME}
        "rosidl_typesupport_cpp"
)
target_link_libraries(
        dji_full_wheel_chassis_controller
        ${cpp_typesupport_target}
        Eigen3::Eigen
)
target_link_libraries(
        lk_full_wheel_chassis_controller
        ${cpp_typesupport_target}
        Eigen3::Eigen
)

target_include_directories(dji_full_wheel_chassis_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(dji_full_wheel_chassis_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(lk_full_wheel_chassis_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(lk_full_wheel_chassis_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS dji_full_wheel_chassis_controller
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS lk_full_wheel_chassis_controller
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
