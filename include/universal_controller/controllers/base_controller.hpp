/**
 * @file base_controller.hpp
 * @brief 控制器基类接口
 *
 * 定义所有控制器的通用接口
 */

#pragma once

#include "universal_controller/common/config.hpp"
#include "universal_controller/common/types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace universal_controller {

/**
 * @brief 控制器基类接口
 *
 * 所有控制器（底盘、云台、发射）都继承此类
 */
class BaseController {
  public:
    virtual ~BaseController() = default;

    /**
     * @brief 初始化控制器
     * @param node ROS2 节点指针
     * @param cfg 配置加载器
     */
    virtual void init(rclcpp::Node *node, const ConfigLoader &cfg) = 0;

    /**
     * @brief 更新控制器（每个控制周期调用）
     * @param dt 时间步长（秒）
     */
    virtual void update(double dt) = 0;

    /**
     * @brief 停止所有输出
     */
    virtual void stop() = 0;

    /**
     * @brief 获取控制器名称
     */
    virtual std::string name() const = 0;

    /**
     * @brief 检查控制器是否已初始化
     */
    bool is_initialized() const {
        return initialized_;
    }

  protected:
    bool initialized_{false};
    rclcpp::Node *node_{nullptr};

    void set_initialized(bool val) {
        initialized_ = val;
    }
    void set_node(rclcpp::Node *node) {
        node_ = node;
    }
};

} // namespace universal_controller
