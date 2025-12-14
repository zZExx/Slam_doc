```
forklift_bt/
├── bt/                          # 行为树XML文件
│   └── forklift_core.xml        # 核心行为树定义
├── include/forklift_bt/         # 头文件（当前未使用）
├── src/
│   └── forklift_bt_node.cpp     # 主节点实现
├── CMakeLists.txt               # 构建配置
├── package.xml                  # ROS2包描述
├── .gitignore                   # 版本控制忽略规则
```

# 1. 构建项目
1. git clone https://github.com/zZExx/Slam_doc.git
2. sudo apt-get install -y ros-humble-behaviortree-cpp-v3
3. cd ~/Desktop/Slam_doc/BehaviorTree/forklift_ws
4. rm -rf build install log
5. colcon build --packages-select forklift_bt

# 2. 运行节点
source install/setup.bash
ros2 run forklift_bt forklift_bt_node
