cmake_minimum_required(VERSION 3.8)
project(lerobot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

add_library(${PROJECT_NAME} SHARED 
  src/lerobot_interface.cpp
)

# ==== Includes ====
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
 "${SERIAL_LDFLAGS}")

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface lerobot_controller.xml)


add_executable(gui_to_controller_bridge src/gui_to_controller_bridge.cpp)
ament_target_dependencies(gui_to_controller_bridge
  rclcpp
  sensor_msgs
  trajectory_msgs
)

install(TARGETS ${PROJECT_NAME} gui_to_controller_bridge
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/
)


install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
