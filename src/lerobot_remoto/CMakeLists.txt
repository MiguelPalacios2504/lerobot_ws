cmake_minimum_required(VERSION 3.8)
project(lerobot_remoto)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lerobot_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(task_server SHARED src/task_server.cpp)
target_include_directories(task_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(task_server
  PRIVATE "TASK_SERVER_CPP_BUILDING_DLL")
ament_target_dependencies(task_server
  "lerobot_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface")
rclcpp_components_register_node(task_server 
  PLUGIN "lerobot_remoto::TaskServer" 
  EXECUTABLE task_server_node
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS 
  task_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(PROGRAMS
  ${PROJECT_NAME}/alexa_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
