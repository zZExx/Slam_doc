#ifndef FORKLIFT_BT_NAVIGATE_TO_GOAL_ACTION_HPP
#define FORKLIFT_BT_NAVIGATE_TO_GOAL_ACTION_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <chrono>

// Nav2 相关（如果可用）
#ifdef NAV2_MSGS_FOUND
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/node_utils.hpp"
#endif

namespace forklift_bt
{

/**
 * @brief 导航到目标点的动作节点（支持Nav2和模拟模式）
 * 
 * 从黑板读取目标位置名称，查找预设位置，然后调用导航
 */
class NavigateToGoalAction : public BT::StatefulActionNode
{
public:
  NavigateToGoalAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config),
      node_(nullptr),
      use_nav2_(false)
  {
    // 初始化预设位置
    initPresetLocations();
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("goal_location", "目标位置名称（如：pickup_zone_A）"),
      BT::InputPort<std::string>("goal_key", "goal_location", "目标位置在黑板中的键名"),
      BT::InputPort<double>("timeout", 2.0, "导航超时时间（秒，模拟模式）")
    };
  }

  BT::NodeStatus onStart() override
  {
    // 获取 ROS 节点（从配置中获取）
    rclcpp::Node::SharedPtr node_ptr;
    if (!config().blackboard->get("node", node_ptr)) {
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToGoalAction"), 
                   "无法获取 ROS 节点，请确保在黑板中设置 'node'");
      return BT::NodeStatus::FAILURE;
    }
    node_ = node_ptr;

    // 从端口获取参数
    std::string goal_location;
    std::string goal_key = "goal_location";
    double timeout = 2.0;
    
    getInput("timeout", timeout);
    getInput("goal_key", goal_key);  // 获取黑板键名
    
    // 优先从端口直接读取 goal_location
    if (!getInput("goal_location", goal_location)) {
      // 如果端口没有，尝试从黑板读取
      if (!config().blackboard->get(goal_key, goal_location)) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "无法获取目标位置，请设置 goal_location 端口或黑板变量 '%s'", goal_key.c_str());
        return BT::NodeStatus::FAILURE;
      }
    }

    // 查找预设位置
    auto it = preset_locations_.find(goal_location);
    if (it == preset_locations_.end()) {
      RCLCPP_ERROR(node_->get_logger(), 
                   "未知的目标位置: %s", goal_location.c_str());
      return BT::NodeStatus::FAILURE;
    }

#ifdef NAV2_MSGS_FOUND
    // 尝试使用 Nav2
    if (!nav2_action_client_) {
      nav2_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        node_, "navigate_to_pose");
    }
    
    // 检查 Nav2 服务是否可用
    if (nav2_action_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
      use_nav2_ = true;
      RCLCPP_INFO(node_->get_logger(), 
                  "[Nav2] 开始导航到: %s (%.2f, %.2f)", 
                  goal_location.c_str(), 
                  it->second[0], 
                  it->second[1]);
      
      // 创建 Nav2 目标
      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp = node_->now();
      goal_msg.pose.pose.position.x = it->second[0];
      goal_msg.pose.pose.position.y = it->second[1];
      goal_msg.pose.pose.position.z = it->second[2];
      goal_msg.pose.pose.orientation.x = it->second[3];
      goal_msg.pose.pose.orientation.y = it->second[4];
      goal_msg.pose.pose.orientation.z = it->second[5];
      goal_msg.pose.pose.orientation.w = it->second[6];
      
      // 发送目标
      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = [this](const auto& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(node_->get_logger(), "[Nav2] 导航成功完成");
        } else {
          RCLCPP_WARN(node_->get_logger(), "[Nav2] 导航失败");
        }
      };
      
      auto future = nav2_action_client_->async_send_goal(goal_msg, send_goal_options);
      if (rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(500)) == 
          rclcpp::FutureReturnCode::SUCCESS) {
        goal_handle_ = future.get();
        if (!goal_handle_) {
          RCLCPP_ERROR(node_->get_logger(), "[Nav2] 发送目标失败");
          use_nav2_ = false;
        }
      } else {
        RCLCPP_WARN(node_->get_logger(), "[Nav2] 等待服务器响应超时，切换到模拟模式");
        use_nav2_ = false;
      }
    } else {
      RCLCPP_INFO(node_->get_logger(), "[Nav2] 服务不可用，使用模拟模式");
      use_nav2_ = false;
    }
#endif

    // 如果 Nav2 不可用，使用模拟模式
    if (!use_nav2_) {
      RCLCPP_INFO(node_->get_logger(), 
                  "[模拟导航] 开始导航到: %s (%.2f, %.2f)", 
                  goal_location.c_str(), 
                  it->second[0], 
                  it->second[1]);
      
      // 模拟导航时间（2秒）
      navigation_start_time_ = std::chrono::steady_clock::now();
      navigation_duration_ = std::chrono::milliseconds(static_cast<int>(timeout * 1000));
      simulated_goal_location_ = goal_location;
    }
    
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!node_) {
      return BT::NodeStatus::FAILURE;
    }

#ifdef NAV2_MSGS_FOUND
    if (use_nav2_ && goal_handle_) {
      // 检查 Nav2 目标状态
      auto status = goal_handle_->get_status();
      
      if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
        return BT::NodeStatus::RUNNING;
      } else if (status == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "[Nav2] 成功到达目标");
        return BT::NodeStatus::SUCCESS;
      } else if (status == rclcpp_action::GoalStatus::STATUS_ABORTED ||
                 status == rclcpp_action::GoalStatus::STATUS_CANCELED) {
        RCLCPP_WARN(node_->get_logger(), "[Nav2] 导航被中止或取消");
        return BT::NodeStatus::FAILURE;
      }
    }
#endif

    // 模拟导航模式
    auto elapsed = std::chrono::steady_clock::now() - navigation_start_time_;
    if (elapsed >= navigation_duration_) {
      RCLCPP_INFO(node_->get_logger(), 
                  "[模拟导航] 成功到达: %s", 
                  simulated_goal_location_.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), "导航被中断");
      
#ifdef NAV2_MSGS_FOUND
      // 取消 Nav2 目标
      if (use_nav2_ && goal_handle_) {
        auto future = nav2_action_client_->async_cancel_goal(goal_handle_);
        rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(100));
      }
#endif
    }
  }

private:
  void initPresetLocations()
  {
    // 预设位置（实际应该从参数服务器或配置文件读取）
    // 格式: {x, y, z, qx, qy, qz, qw}
    preset_locations_["pickup_zone_A"] = {2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    preset_locations_["pickup_zone_B"] = {4.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    preset_locations_["dropoff_zone_A"] = {2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    preset_locations_["dropoff_zone_B"] = {4.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    preset_locations_["charging_station"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    preset_locations_["home"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  }

  rclcpp::Node::SharedPtr node_;
  
  // Nav2 相关
#ifdef NAV2_MSGS_FOUND
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
  bool use_nav2_;
#else
  bool use_nav2_ = false;
#endif
  
  // 模拟导航模式
  std::chrono::steady_clock::time_point navigation_start_time_;
  std::chrono::milliseconds navigation_duration_;
  std::string simulated_goal_location_;
  
  std::map<std::string, std::vector<double>> preset_locations_;
};

} // namespace forklift_bt

#endif // FORKLIFT_BT_NAVIGATE_TO_GOAL_ACTION_HPP
