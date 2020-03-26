cmake_minimum_required(VERSION 3.5)
project(velocity_smoother)

find_package(ament_cmake_ros REQUIRED)
find_package(ecl_build REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

ecl_enable_cxx14_compiler()
ecl_enable_cxx_warnings()

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/velocity_smoother.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "geometry_msgs"
  "nav_msgs"
  "rcl_interfaces"
  "rclcpp"
  "rclcpp_components"
)

add_executable(velocity_smoother_node src/velocity_smoother_node.cpp)
ament_target_dependencies(velocity_smoother_node
  "rclcpp"
)
target_link_libraries(velocity_smoother_node ${PROJECT_NAME})
set_target_properties(velocity_smoother_node PROPERTIES OUTPUT_NAME velocity_smoother)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "velocity_smoother::VelocitySmoother")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS velocity_smoother_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    test/translational_command_profile.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    test/test_translational_smoothing.py
  DESTINATION share/${PROJECT_NAME}/launch_test
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    "test/test_translational_smoothing.py"
    TIMEOUT 30
  )
endif()

ament_package()
