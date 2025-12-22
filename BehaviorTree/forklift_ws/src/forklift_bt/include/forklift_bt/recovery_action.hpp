#ifndef FORKLIFT_BT_RECOVERY_ACTION_HPP
#define FORKLIFT_BT_RECOVERY_ACTION_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>
#include <chrono>

namespace forklift_bt
{

/**
 * @brief 恢复动作节点
 * 
 */
class RecoveryAction : public BT::StatefulActionNode
{
public:
  RecoveryAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config),
      node_(nullptr)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("recovery_type", "clear_costmap", "恢复类型: clear_costmap, replan_route"),
      BT::InputPort<double>("duration", 1.0, "恢复动作持续时间（秒）")
    };
  }

  BT::NodeStatus onStart() override
  {
    // 获取 ROS 节点
    rclcpp::Node::SharedPtr node_ptr;
    if (!config().blackboard->get("node", node_ptr)) {
      RCLCPP_ERROR(rclcpp::get_logger("RecoveryAction"), 
                   "无法获取 ROS 节点，请确保在黑板中设置 'node'");
      return BT::NodeStatus::FAILURE;
    }
    node_ = node_ptr;

    // 从端口获取参数
    std::string recovery_type = "clear_costmap";
    double duration = 1.0;
    
    getInput("recovery_type", recovery_type);
    getInput("duration", duration);

    // 记录开始时间
    recovery_start_time_ = std::chrono::steady_clock::now();
    recovery_duration_ = std::chrono::milliseconds(static_cast<int>(duration * 1000));
    recovery_type_ = recovery_type;

    RCLCPP_INFO(node_->get_logger(), 
                "[恢复动作] 开始执行: %s (预计耗时: %.1f秒)", 
                recovery_type.c_str(), duration);
    
    // 清除 halt_navigation 标志，让恢复后的重试导航可以正常进行
    if (config().blackboard)
    {
      config().blackboard->set("halt_navigation", false);
      RCLCPP_INFO(node_->get_logger(), "[恢复动作] 已清除导航停止标志，准备重试");
    }
    
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!node_) {
      return BT::NodeStatus::FAILURE;
    }

    // 检查是否完成
    auto elapsed = std::chrono::steady_clock::now() - recovery_start_time_;
    auto elapsed_seconds = std::chrono::duration<double>(elapsed).count();
    auto total_seconds = std::chrono::duration<double>(recovery_duration_).count();
    auto progress = (elapsed_seconds / total_seconds) * 100.0;

    // 每1秒打印一次进度
    static double last_logged_progress = -1.0;
    if (progress - last_logged_progress >= 20.0 || last_logged_progress < 0) {
      RCLCPP_INFO(node_->get_logger(), 
                  "[恢复动作] %s 进行中... %.1f%% (%.1f/%.1f秒)", 
                  recovery_type_.c_str(), progress, elapsed_seconds, total_seconds);
      last_logged_progress = progress;
    }

    if (elapsed >= recovery_duration_) {
      last_logged_progress = -1.0; // 重置，为下次执行准备
      RCLCPP_INFO(node_->get_logger(), 
                  "[恢复动作] 完成: %s (总耗时: %.1f秒)", 
                  recovery_type_.c_str(), total_seconds);
      return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (node_) {
      RCLCPP_WARN(node_->get_logger(), "[恢复动作] 被中断: %s", recovery_type_.c_str());
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::chrono::steady_clock::time_point recovery_start_time_;
  std::chrono::milliseconds recovery_duration_;
  std::string recovery_type_;
};

} // namespace forklift_bt

#endif // FORKLIFT_BT_RECOVERY_ACTION_HPP

