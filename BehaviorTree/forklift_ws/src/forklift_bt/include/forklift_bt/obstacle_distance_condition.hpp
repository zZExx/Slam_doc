#ifndef FORKLIFT_BT_OBSTACLE_DISTANCE_CONDITION_HPP
#define FORKLIFT_BT_OBSTACLE_DISTANCE_CONDITION_HPP

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace forklift_bt
{

/**
 * @brief 检查障碍物距离是否安全的条件节点
 * 
 * 从黑板读取障碍物距离，如果大于等于 min_value，返回 SUCCESS，否则返回 FAILURE
 */
class ObstacleDistanceCondition : public BT::ConditionNode
{
public:
  ObstacleDistanceCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_value", 0.8, "最小安全距离（米）"),
      BT::InputPort<std::string>("distance_key", "obstacle_distance", "障碍物距离在黑板中的键名")
    };
  }

  BT::NodeStatus tick() override
  {
    double min_value = 0.8;
    std::string distance_key = "obstacle_distance";
    
    // 从端口获取参数
    if (!getInput("min_value", min_value)) {
      // 使用默认值
    }
    if (!getInput("distance_key", distance_key)) {
      // 使用默认值
    }
    
    // 从黑板读取障碍物距离
    double obstacle_distance = 0.0;
    if (!config().blackboard->get(distance_key, obstacle_distance)) {
      RCLCPP_WARN(rclcpp::get_logger("ObstacleDistanceCondition"), 
                  "无法从黑板读取障碍物距离: %s", distance_key.c_str());
      return BT::NodeStatus::FAILURE;
    }
    
    // 检查障碍物距离是否安全
    if (obstacle_distance >= min_value) {
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("ObstacleDistanceCondition"), 
                  "障碍物距离过近: %.2fm < %.2fm", obstacle_distance, min_value);
      return BT::NodeStatus::FAILURE;
    }
  }
};

} // namespace forklift_bt

#endif // FORKLIFT_BT_OBSTACLE_DISTANCE_CONDITION_HPP

