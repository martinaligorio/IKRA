cmake_minimum_required(VERSION 3.5)
project(robot_arm_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TinyXML2 REQUIRED)

add_executable(robot_arm_node src/robot_arm_node.cpp)
ament_target_dependencies(robot_arm_node rclcpp geometry_msgs Eigen3)

install(TARGETS
  robot_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
