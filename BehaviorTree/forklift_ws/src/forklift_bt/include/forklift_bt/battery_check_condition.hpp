#ifndef FORKLIFT_BT_BATTERY_CHECK_CONDITION_HPP
#define FORKLIFT_BT_BATTERY_CHECK_CONDITION_HPP

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace forklift_bt
{

/**
 * @brief 检查电池电量是否充足的条件节点
 * 
 * 从黑板读取 battery_level，如果大于等于 min_value，返回 SUCCESS，否则返回 FAILURE
 */
class BatteryCheckCondition : public BT::ConditionNode
{
public:
  BatteryCheckCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_value", 25.0, "最低电量阈值（百分比）"),
      BT::InputPort<std::string>("battery_key", "battery_level", "电池电量在黑板中的键名")
    };
  }

  BT::NodeStatus tick() override
  {
    double min_value = 25.0;
    std::string battery_key = "battery_level";
    
    // 从端口获取参数
    if (!getInput("min_value", min_value)) {
      // 使用默认值
    }
    if (!getInput("battery_key", battery_key)) {
      // 使用默认值
    }
    
    // 从黑板读取电池电量
    double battery_level = 0.0;
    if (!config().blackboard->get(battery_key, battery_level)) {
      RCLCPP_WARN(rclcpp::get_logger("BatteryCheckCondition"), 
                  "无法从黑板读取电池电量: %s", battery_key.c_str());
      return BT::NodeStatus::FAILURE;
    }
    
    // 检查电池电量是否充足
    if (battery_level >= min_value) {
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("BatteryCheckCondition"), 
                  "电池电量不足: %.1f%% < %.1f%%", battery_level, min_value);
      return BT::NodeStatus::FAILURE;
    }
  }
};

} // namespace forklift_bt

#endif // FORKLIFT_BT_BATTERY_CHECK_CONDITION_HPP

