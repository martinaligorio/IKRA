cmake_minimum_required(VERSION 3.5)
project(robot_arm_control)

set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH} /opt/ros/rolling/share/tinyxml2_vendor")
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(${TINYXML2_INCLUDE_DIRS})
link_directories(${TINYXML2_LIBRARY_DIRS})

add_executable(robot_arm_node src/robot_arm_node.cpp)
ament_target_dependencies(robot_arm_node rclcpp geometry_msgs Eigen3)

target_link_libraries(robot_arm_node ${TINYXML2_LIBRARIES})



install(TARGETS
  robot_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
