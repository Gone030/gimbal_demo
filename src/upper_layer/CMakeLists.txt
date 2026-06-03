add_executable(arm_upper_layer_main arm_upper_layer_main.cpp)
add_executable(arm_joint_test_node arm_joint_test_node.cpp)
add_executable(gimbal_upper_layer_main gimbal_upper_layer_main.cpp)
add_executable(vision_node vision_node.cpp)
add_executable(teleop_node gimbal_teleop_node.cpp)

ament_target_dependencies(arm_upper_layer_main
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(arm_joint_test_node
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
)

ament_target_dependencies(gimbal_upper_layer_main
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
)

ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

ament_target_dependencies(teleop_node
  rclcpp
  std_msgs
)

target_include_directories(arm_upper_layer_main
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

target_include_directories(gimbal_upper_layer_main
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

target_link_libraries(vision_node
  ${OpenCV_LIBS}
  ${cpp_typesupport_target}
)

target_link_libraries(teleop_node ${cpp_typesupport_target})
target_link_libraries(arm_upper_layer_main ${cpp_typesupport_target})
target_link_libraries(arm_joint_test_node ${cpp_typesupport_target})
target_link_libraries(gimbal_upper_layer_main ${cpp_typesupport_target})

install(TARGETS arm_upper_layer_main
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS arm_joint_test_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS gimbal_upper_layer_main
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS vision_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS teleop_node
  DESTINATION lib/${PROJECT_NAME}
)
