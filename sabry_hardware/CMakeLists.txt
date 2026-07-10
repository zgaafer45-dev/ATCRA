cmake_minimum_required(VERSION 3.8)
project(sabry_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LinearMotor.srv"
  "srv/ChangeTool.srv"
  "action/ChangeTool.action"
)

add_library(${PROJECT_NAME}_system SHARED
  src/sabry_system.cpp
  src/gripper_system.cpp
)
target_include_directories(${PROJECT_NAME}_system PRIVATE
  include
  ${LIBSERIAL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_system
  ${LIBSERIAL_LIBRARIES}
)
target_compile_options(${PROJECT_NAME}_system PRIVATE
  ${LIBSERIAL_CFLAGS_OTHER}
)
target_link_options(${PROJECT_NAME}_system PRIVATE "-Wl,--no-undefined")
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_system)


ament_target_dependencies(${PROJECT_NAME}_system
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
)

pluginlib_export_plugin_description_file(
  hardware_interface sabry_hardware.xml
)

install(TARGETS ${PROJECT_NAME}_system
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  
)

# ament_python_install_package(scripts)

# rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
#   "srv/LinearMotor.srv"
#   "srv/ChangeTool.srv"
#   "action/ChangeTool.action"
# )

# ament_python_install_package(scripts)
install(PROGRAMS
  scripts/linear_motor_node.py
  scripts/tool_change_manager.py
  scripts/operation.py
  scripts/camera_stream.py
  scripts/lifecycle_manager.py
  scripts/inspection_v3.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)
install(FILES sabry_hardware.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
