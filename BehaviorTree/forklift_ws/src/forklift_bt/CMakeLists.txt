cmake_minimum_required(VERSION 3.8)
project(forklift_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# Nav2 依赖（可选，如果没有安装则使用模拟模式）
find_package(nav2_msgs QUIET)
find_package(nav2_util QUIET)
find_package(geometry_msgs QUIET)
find_package(tf2_ros QUIET)
find_package(rclcpp_action QUIET)

add_executable(forklift_bt_node src/forklift_bt_node.cpp)
target_include_directories(forklift_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 如果找到 Nav2，添加依赖和定义宏
if(nav2_msgs_FOUND)
  target_compile_definitions(forklift_bt_node PRIVATE NAV2_MSGS_FOUND)
  ament_target_dependencies(forklift_bt_node
    rclcpp
    behaviortree_cpp_v3
    nav2_msgs
    geometry_msgs
    rclcpp_action
  )
  message(STATUS "Nav2 已找到，启用 Nav2 导航功能")
else()
  ament_target_dependencies(forklift_bt_node
    rclcpp
    behaviortree_cpp_v3
  )
  message(STATUS "Nav2 未找到，使用模拟导航模式")
endif()

target_link_libraries(forklift_bt_node
  BT::behaviortree_cpp_v3
)

install(TARGETS forklift_bt_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY bt/
  DESTINATION share/${PROJECT_NAME}/bt
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()