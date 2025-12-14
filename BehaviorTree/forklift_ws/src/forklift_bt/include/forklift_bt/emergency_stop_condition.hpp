#ifndef FORKLIFT_BT_EMERGENCY_STOP_CONDITION_HPP
#define FORKLIFT_BT_EMERGENCY_STOP_CONDITION_HPP

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace forklift_bt
{

/**
 * @brief 检查紧急停止是否未按下的条件节点
 * 
 * 从黑板读取 emergency_stop，如果为 false，返回 SUCCESS，否则返回 FAILURE
 */
class EmergencyStopCondition : public BT::ConditionNode
{
public:
  EmergencyStopCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("emergency_key", "emergency_stop", "紧急停止状态在黑板中的键名")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string emergency_key = "emergency_stop";
    
    // 从端口获取参数
    if (!getInput("emergency_key", emergency_key)) {
      // 使用默认值
    }
    
    // 从黑板读取紧急停止状态（支持 bool 和 string 类型）
    bool emergency_stop = false;
    
    // 先尝试作为字符串读取
    std::string emergency_str;
    if (config().blackboard->get(emergency_key, emergency_str)) {
      // 字符串转换为 bool
      emergency_stop = (emergency_str == "true" || emergency_str == "1" || emergency_str == "True");
    } else if (config().blackboard->get(emergency_key, emergency_stop)) {
      // 直接读取 bool
      // emergency_stop 已经设置
    } else {
      RCLCPP_WARN(rclcpp::get_logger("EmergencyStopCondition"), 
                  "无法从黑板读取紧急停止状态: %s", emergency_key.c_str());
      return BT::NodeStatus::FAILURE;
    }
    
    // 检查紧急停止是否未按下
    if (!emergency_stop) {
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("EmergencyStopCondition"), 
                   "紧急停止已触发!");
      return BT::NodeStatus::FAILURE;
    }
  }
};

} // namespace forklift_bt

#endif // FORKLIFT_BT_EMERGENCY_STOP_CONDITION_HPP

