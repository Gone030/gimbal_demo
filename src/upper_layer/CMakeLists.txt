add_executable(arm_upper_layer_main arm_upper_layer_main.cpp)

ament_target_dependencies(arm_upper_layer_main
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
)

target_include_directories(arm_upper_layer_main
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

target_link_libraries(arm_upper_layer_main ${cpp_typesupport_target})

install(TARGETS arm_upper_layer_main
    DESTINATION lib/${PROJECT_NAME}
)
