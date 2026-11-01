cmake_minimum_required(VERSION 3.8)
project(aegis_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(flipper_controller SHARED src/flipper_controller.cpp)
target_include_directories(flipper_controller PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
)
ament_target_dependencies(flipper_controller
  rclcpp
  geometry_msgs
  trajectory_msgs
  gazebo_ros
)
target_link_libraries(flipper_controller ${GAZEBO_LIBRARIES})

install(TARGETS flipper_controller
  LIBRARY DESTINATION lib
)

ament_package()
