cmake_minimum_required(VERSION 3.5)
project(robot_arm_control)

set(TinyXML2_DIR "/usr/lib/cmake/TinyXML2")
find_package(TinyXML2 REQUIRED)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories("/usr/include/tinyxml2")

add_executable(robot_arm_node src/robot_arm_node.cpp)
ament_target_dependencies(robot_arm_node rclcpp geometry_msgs Eigen3 TinyXML2)

target_link_libraries(robot_arm_node TinyXML2::TinyXML2)
install(TARGETS
  robot_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
