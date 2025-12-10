cmake_minimum_required(VERSION 3.8)
project(forklift_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

add_executable(forklift_bt_node src/forklift_bt_node.cpp)
target_link_libraries(forklift_bt_node
  BT::behaviortree_cpp_v3
)
ament_target_dependencies(forklift_bt_node
  rclcpp
  behaviortree_cpp_v3
)

install(TARGETS forklift_bt_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY bt/
  DESTINATION share/${PROJECT_NAME}/bt
)

ament_package()