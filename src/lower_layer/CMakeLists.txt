add_library(lower_layer_nodes INTERFACE)

add_executable(jt_command_bridge jt_command_bridge.cpp)
ament_target_dependencies(jt_command_bridge rclcpp trajectory_msgs)

install(TARGETS jt_command_bridge
  DESTINATION lib/${PROJECT_NAME})
