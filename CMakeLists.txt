cmake_minimum_required(VERSION 3.8)
project(gimbal_mani)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)

add_subdirectory(src/lower_layer)


add_executable(teleop src/gimbal_teleop_node.cpp)
ament_target_dependencies(teleop
  rclcpp
  std_msgs
  trajectory_msgs
)

add_executable(color_tracker src/gimbal_visual_tracker_color_node.cpp)
ament_target_dependencies(color_tracker
  rclcpp
  trajectory_msgs
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

add_executable(target_viz src/target_visualizer.cpp)
ament_target_dependencies(target_viz
  rclcpp
  sensor_msgs
  visualization_msgs
)


add_executable(ik_mani src/mani_ik_test_node.cpp)
ament_target_dependencies(ik_mani
  rclcpp
  sensor_msgs
  geometry_msgs
  urdf
)

install(TARGETS
  teleop
  color_tracker
  target_viz
  ik_mani
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
