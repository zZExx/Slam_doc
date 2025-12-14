#ifndef FORKLIFT_BT_HAS_CARGO_CONDITION_HPP
#define FORKLIFT_BT_HAS_CARGO_CONDITION_HPP

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace forklift_bt
{

/**
 * @brief 检查是否携带货物的条件节点
 * 
 * 从黑板读取 has_cargo，如果为 true，返回 SUCCESS，否则返回 FAILURE
 */
class HasCargoCondition : public BT::ConditionNode
{
public:
  HasCargoCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("cargo_key", "has_cargo", "货物状态在黑板中的键名")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string cargo_key = "has_cargo";
    
    // 从端口获取参数
    if (!getInput("cargo_key", cargo_key)) {
      // 使用默认值
    }
    
    // 从黑板读取货物状态（支持 bool 和 string 类型）
    bool has_cargo = false;
    
    // 先尝试作为字符串读取
    std::string cargo_str;
    if (config().blackboard->get(cargo_key, cargo_str)) {
      // 字符串转换为 bool
      has_cargo = (cargo_str == "true" || cargo_str == "1" || cargo_str == "True");
    } else if (config().blackboard->get(cargo_key, has_cargo)) {
      // 直接读取 bool
      // has_cargo 已经设置
    } else {
      RCLCPP_WARN(rclcpp::get_logger("HasCargoCondition"), 
                  "无法从黑板读取货物状态: %s", cargo_key.c_str());
      return BT::NodeStatus::FAILURE;
    }
    
    // 检查是否携带货物
    if (has_cargo) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }
};

} // namespace forklift_bt

#endif // FORKLIFT_BT_HAS_CARGO_CONDITION_HPP

