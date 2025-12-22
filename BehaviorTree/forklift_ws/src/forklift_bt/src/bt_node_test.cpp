#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <chrono>
#include <memory>
#include <std_srvs/srv/empty.hpp>

#include "forklift_bt/navigate_to_goal_action.hpp"
#include "forklift_bt/emergency_stop_condition.hpp"
#include "forklift_bt/recovery_action.hpp"

class ForkliftBTTestNode : public rclcpp::Node
{
public:
    ForkliftBTTestNode() : Node("forklift_bt_test")
    {
        halt_navigation_ = false;
        tree_halted_ = false;

        halt_service_ = this->create_service<std_srvs::srv::Empty>(
            "halt_navigation",
            std::bind(&ForkliftBTTestNode::haltNavigationCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "创建停止导航服务: /halt_navigation");
    }

    void initialize()
    {
        initializeBehaviorTree();
    }

    void runBehaviorTree()
    {
        // 如果树已被停止（紧急停止），不再执行
        if (tree_halted_)
        {
            return;
        }

        // 每次进入 runBehaviorTree 表示一次 tick
        static int tick_count = 0;
        tick_count++;

        if (!tree_.rootNode())
        {
            RCLCPP_WARN(this->get_logger(), "Tick %d - rootNode is null, skip", tick_count);
            return;
        }

        try
        {
            // 执行一次 tick
            BT::NodeStatus status = tree_.tickRoot();

            // 打印本次 tick 的结果
            const char *status_str =
                (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" :
                (status == BT::NodeStatus::FAILURE) ? "FAILURE" :
                (status == BT::NodeStatus::RUNNING) ? "RUNNING" : "OTHER";

            RCLCPP_INFO(this->get_logger(),
                        "Tick %d - tree status: %s",
                        tick_count, status_str);

            if (status == BT::NodeStatus::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "行为树执行完成");
                tree_.haltTree();
            }
            else if (status == BT::NodeStatus::FAILURE)
            {
                // 检查是否是紧急停止导致的失败
                bool emergency_stop = false;
                if (tree_.rootBlackboard() && tree_.rootBlackboard()->get("emergency_stop", emergency_stop) && emergency_stop)
                {
                    RCLCPP_WARN(this->get_logger(), "紧急停止触发，行为树已永久停止");
                    tree_.haltTree();
                    tree_halted_ = true;  // 设置标志，阻止后续所有 tick
                    // 不清除 emergency_stop，保持停止状态
                    halt_navigation_ = false;
                    tree_.rootBlackboard()->set("halt_navigation", false);
                    // 注意：不清除 emergency_stop，这样即使树重新初始化，也会检测到停止状态
                }
                else if (halt_navigation_)
                {
                    // 导航被停止，Fallback 会自动尝试恢复流程
                    // 恢复动作会在开始时清除 halt_navigation 标志
                    RCLCPP_INFO(this->get_logger(), "导航已停止，Fallback 将尝试恢复流程");
                    // 不清除 halt_navigation，让恢复动作自己清除（在恢复开始时）
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "行为树执行失败（可能是导航失败，将尝试恢复）");
                    // 注意：如果是 Fallback 节点，失败后会尝试恢复流程，所以这里不立即 haltTree
                    // 只有当整个 Fallback 都失败时才 haltTree
                }
                
                // 检查 Fallback 是否完全失败（包括恢复也失败）
                // 如果整个 Fallback 都失败了，才真正停止树
                // 这里我们通过检查是否连续多次失败来判断
                static int failure_count = 0;
                if (status == BT::NodeStatus::FAILURE && !emergency_stop)
                {
                    failure_count++;
                    // 如果连续失败多次（包括恢复也失败），才真正停止
                    if (failure_count >= 3)
                    {
                        RCLCPP_ERROR(this->get_logger(), "导航和恢复都失败，停止行为树");
                        tree_.haltTree();
                        failure_count = 0;
                    }
                }
                else
                {
                    failure_count = 0;  // 重置计数
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "行为树执行错误: %s", e.what());
            tree_.haltTree();
        }
    }

private:
    void initializeBehaviorTree()
    {
        try
        {
            BT::BehaviorTreeFactory factory;

            // 注册导航动作节点
            factory.registerNodeType<forklift_bt::NavigateToGoalAction>("NavigateToGoalAction");

            // 注册紧急停止条件节点
            factory.registerNodeType<forklift_bt::EmergencyStopCondition>("EmergencyStopCondition");

            // 注册恢复动作节点
            factory.registerNodeType<forklift_bt::RecoveryAction>("RecoveryAction");

            RCLCPP_INFO(this->get_logger(), "已注册自定义节点(导航 + 紧急停止 + 恢复动作)");

            std::string package_share_dir = ament_index_cpp::get_package_share_directory("forklift_bt");
            std::string bt_xml_path = package_share_dir + "/bt/test_tree.xml";

            RCLCPP_INFO(this->get_logger(), "加载行为树: %s", bt_xml_path.c_str());

            if (!std::filesystem::exists(bt_xml_path))
            {
                RCLCPP_ERROR(this->get_logger(), "行为树文件不存在: %s", bt_xml_path.c_str());
                return;
            }

            tree_ = factory.createTreeFromFile(bt_xml_path);

            if (tree_.rootBlackboard())
            {
                auto node_ptr = std::static_pointer_cast<rclcpp::Node>(shared_from_this());
                tree_.rootBlackboard()->set("node", node_ptr);
                tree_.rootBlackboard()->set("halt_navigation", false);
                tree_.rootBlackboard()->set("emergency_stop", false);
            }

            logger_ = std::make_unique<BT::StdCoutLogger>(tree_);

            RCLCPP_INFO(this->get_logger(), "行为树加载成功");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "行为树初始化失败: %s", e.what());
        }
    }

    void haltNavigationCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        (void)request;
        (void)response;
        halt_navigation_ = true;
        if (tree_.rootBlackboard())
        {
            // 只设置 halt_navigation，不设置 emergency_stop
            // 这样导航会失败，但 Fallback 可以执行恢复流程
            tree_.rootBlackboard()->set("halt_navigation", true);
        }
        RCLCPP_WARN(this->get_logger(), "收到停止导航请求，导航将失败并触发恢复流程...");
    }

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr halt_service_;
    BT::Tree tree_;
    std::unique_ptr<BT::StdCoutLogger> logger_;
    bool halt_navigation_;
    bool tree_halted_;  // 标志：树是否已被永久停止（紧急停止）
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForkliftBTTestNode>();

    node->initialize();

    auto bt_timer = node->create_wall_timer(
        std::chrono::milliseconds(100),
        [node]() { node->runBehaviorTree(); });

    RCLCPP_INFO(rclcpp::get_logger("main"), "行为树测试节点运行中...");
    RCLCPP_INFO(rclcpp::get_logger("main"), "使用以下命令停止导航:");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  ros2 service call /halt_navigation std_srvs/srv/Empty");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

