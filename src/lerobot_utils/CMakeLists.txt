cmake_minimum_required(VERSION 3.8)
project(lerobot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(lerobot_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(angle_conversion src/angle_conversion.cpp)
ament_target_dependencies(angle_conversion rclcpp lerobot_msgs tf2)

install(TARGETS
  angle_conversion
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
