#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <chrono>
#include <thread>
#include <memory>

// 自定义条件节点
#include "forklift_bt/battery_check_condition.hpp"
#include "forklift_bt/emergency_stop_condition.hpp"
#include "forklift_bt/obstacle_distance_condition.hpp"
#include "forklift_bt/has_cargo_condition.hpp"

// 自定义动作节点
#include "forklift_bt/navigate_to_goal_action.hpp"

class ForkliftBTNode : public rclcpp::Node
{
public:
  ForkliftBTNode() : Node("forklift_bt_node")
  {
    RCLCPP_INFO(this->get_logger(), "=== 无人叉车行为树系统启动 ===");
    
    // 创建定时器来模拟传感器数据
    sensor_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ForkliftBTNode::updateSensorData, this));
  }
  
  void initialize()
  {
    // 延迟初始化行为树，确保shared_from_this()可用
    initializeBehaviorTree();
  }

  void runBehaviorTree()
  {
    if (!tree_.rootNode()) return;
    
    try {
      // 执行一次tick
      BT::NodeStatus status = tree_.tickRoot();
      
      // 打印当前状态
      static int tick_count = 0;
      tick_count++;
      if (tick_count % 1 == 0) { // 每次tick打印一次
        double battery = 0.0;
        if (tree_.rootBlackboard()->get("battery_level", battery)) {
          RCLCPP_INFO(this->get_logger(), "电池: %.1f%% | Tick: %d | 状态: %s", 
            battery, tick_count, 
            status == BT::NodeStatus::SUCCESS ? "SUCCESS" : 
            status == BT::NodeStatus::FAILURE ? "FAILURE" : "RUNNING");
        }
      }
      
      // 如果任务完成，重启树
      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "任务完成! 重启行为树...");
        tree_.haltTree();
        // 重新加载树
        initializeBehaviorTree();
      }
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "行为树执行错误: %s", e.what());
      tree_.haltTree();
    }
  }

private:
  void initializeBehaviorTree()
  {
    try {
      BT::BehaviorTreeFactory factory;
      
      // 注册自定义条件节点
      factory.registerNodeType<forklift_bt::BatteryCheckCondition>("BatteryCheckCondition");
      factory.registerNodeType<forklift_bt::EmergencyStopCondition>("EmergencyStopCondition");
      factory.registerNodeType<forklift_bt::ObstacleDistanceCondition>("ObstacleDistanceCondition");
      factory.registerNodeType<forklift_bt::HasCargoCondition>("HasCargoCondition");
      
      // 注册自定义动作节点
      factory.registerNodeType<forklift_bt::NavigateToGoalAction>("NavigateToGoalAction");
      
      RCLCPP_INFO(this->get_logger(), "已注册自定义条件节点和动作节点");
      
      // 获取XML路径
      std::string package_share_dir = ament_index_cpp::get_package_share_directory("forklift_bt");
      std::string bt_xml_path = package_share_dir + "/bt/forklift_core.xml";
      
      RCLCPP_INFO(this->get_logger(), "加载行为树: %s", bt_xml_path.c_str());
      
      if (!std::filesystem::exists(bt_xml_path)) {
        RCLCPP_ERROR(this->get_logger(), "行为树文件不存在: %s", bt_xml_path.c_str());
        return;
      }
      
      // 加载行为树
      tree_ = factory.createTreeFromFile(bt_xml_path);
      
      // 重新初始化传感器数据
      if (tree_.rootBlackboard()) {
        tree_.rootBlackboard()->set("battery_level", 100.0);
        tree_.rootBlackboard()->set("emergency_stop", false);
        tree_.rootBlackboard()->set("obstacle_distance", 5.0);
        
        // 将 ROS 节点添加到黑板，供动作节点使用
        // 使用 shared_from_this() 获取当前节点的共享指针
        auto node_ptr = std::static_pointer_cast<rclcpp::Node>(shared_from_this());
        tree_.rootBlackboard()->set("node", node_ptr);
      }
      
      // 添加日志记录器
      static bool logger_created = false;
      if (!logger_created && tree_.rootNode()) {
        logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
        logger_created = true;
      }
      
      RCLCPP_INFO(this->get_logger(), "行为树加载成功");
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "行为树初始化失败: %s", e.what());
    }
  }
  
  void updateSensorData()
  {
    if (!tree_.rootBlackboard()) return;
    
    try {
      // 模拟电池消耗
      double battery_level;
      if (tree_.rootBlackboard()->get("battery_level", battery_level)) {
        battery_level = std::max(0.0, battery_level - 1.0); // 每500ms消耗1%
        tree_.rootBlackboard()->set("battery_level", battery_level);
      }
      
      // 模拟障碍物变化
      double obstacle_distance;
      if (tree_.rootBlackboard()->get("obstacle_distance", obstacle_distance)) {
        // 随机模拟障碍物距离变化
        static int cycle = 0;
        cycle++;
        if (cycle % 10 == 0) { // 每5秒模拟一次障碍物
          obstacle_distance = (cycle % 20 < 10) ? 0.5 : 3.0; // 有时近，有时远
          tree_.rootBlackboard()->set("obstacle_distance", obstacle_distance);
          
          if (obstacle_distance < 1.0) {
            RCLCPP_WARN(this->get_logger(), "检测到障碍物! 距离: %.2fm", obstacle_distance);
          }
        }
      }
      
      // 模拟紧急停止（随机触发）
      static int emergency_cycle = 0;
      emergency_cycle++;
      if (emergency_cycle % 30 == 0) { // 每15秒随机触发紧急停止
        bool emergency = (emergency_cycle % 60 < 30); // 50%概率
        tree_.rootBlackboard()->set("emergency_stop", emergency);
        if (emergency) {
          RCLCPP_ERROR(this->get_logger(), "紧急停止被触发!");
        }
      }
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "传感器数据更新错误: %s", e.what());
    }
  }

  rclcpp::TimerBase::SharedPtr sensor_timer_;
  BT::Tree tree_;
  std::unique_ptr<BT::StdCoutLogger> logger_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ForkliftBTNode>();
  
  // 初始化行为树（在shared_from_this()可用后）
  node->initialize();
  
  // 创建行为树执行定时器
  auto bt_timer = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [node]() { node->runBehaviorTree(); });
  
  RCLCPP_INFO(rclcpp::get_logger("main"), "无人叉车行为树系统运行中...");
  RCLCPP_INFO(rclcpp::get_logger("main"), "----------------------------------------");
  RCLCPP_INFO(rclcpp::get_logger("main"), "核心功能演示:");
  RCLCPP_INFO(rclcpp::get_logger("main"), "   - 并行安全监控");
  RCLCPP_INFO(rclcpp::get_logger("main"), "   - 电池管理");
  RCLCPP_INFO(rclcpp::get_logger("main"), "   - 障碍物检测");
  RCLCPP_INFO(rclcpp::get_logger("main"), "   - 紧急停止处理");
  RCLCPP_INFO(rclcpp::get_logger("main"), "   - 多任务执行");
  RCLCPP_INFO(rclcpp::get_logger("main"), "----------------------------------------");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}